#!/usr/bin/env python3
"""
Pottery Kiln Controller
Uses Adafruit MAX31856 thermocouple amplifier and Raspberry Pi
Implements PID control with firing schedules and safety features
Web interface for monitoring and control
"""

import board
import digitalio
import adafruit_max31856
import time
import json
from datetime import datetime
from simple_pid import PID
import logging
import threading
from flask import Flask, render_template, jsonify, request
from flask_cors import CORS

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('kiln.log'),
        logging.StreamHandler()
    ]
)

class KilnController:
    def __init__(self, relay_pin=18, max_temp=1300, safety_margin=50):
        """
        Initialize the kiln controller
        
        Args:
            relay_pin: GPIO pin for SSR/relay control
            max_temp: Maximum safe temperature in Celsius
            safety_margin: Temperature margin for safety shutoff
        """
        # Initialize SPI and MAX31856
        spi = board.SPI()
        cs = digitalio.DigitalInOut(board.D5)
        self.thermocouple = adafruit_max31856.MAX31856(spi, cs)
        
        # Set thermocouple type (K-type is common for kilns)
        self.thermocouple.thermocouple_type = adafruit_max31856.ThermocoupleType.K
        
        # Setup relay control (for SSR - Solid State Relay)
        self.relay = digitalio.DigitalInOut(getattr(board, f'D{relay_pin}'))
        self.relay.direction = digitalio.Direction.OUTPUT
        self.relay.value = False
        
        # Safety parameters
        self.max_temp = max_temp
        self.safety_margin = safety_margin
        self.emergency_stop = False
        
        # PID controller setup
        # Tune these values for your specific kiln
        # For high thermal mass systems like kilns, start conservative
        self.pid = PID(1.0, 0.1, 0.5, setpoint=0)
        self.pid.output_limits = (0, 100)  # 0-100% duty cycle
        self.pid.sample_time = 1.0  # Update every second
        
        # Firing schedule
        self.schedule = []
        self.current_segment = 0
        self.firing_active = False
        self.start_time = None
        self.segment_start_time = None
        self.segment_start_temp = None
        
        # Data logging
        self.data_log = []
        
        # Current state for web interface
        self.current_temp = None
        self.current_setpoint = 0
        self.current_output = 0
        self.state_lock = threading.Lock()
        
    def read_temperature(self):
        """Read current temperature from thermocouple"""
        try:
            temp = self.thermocouple.temperature
            if temp is None:
                logging.error("Failed to read temperature")
                return None
            with self.state_lock:
                self.current_temp = temp
            return temp
        except Exception as e:
            logging.error(f"Error reading temperature: {e}")
            return None
    
    def check_safety(self, current_temp):
        """
        Check safety conditions
        Returns True if safe to continue, False if emergency stop needed
        """
        if current_temp is None:
            logging.error("Invalid temperature reading - EMERGENCY STOP")
            return False
        
        if current_temp > self.max_temp + self.safety_margin:
            logging.error(f"Temperature {current_temp}°C exceeds safe limit - EMERGENCY STOP")
            return False
        
        # Check for thermocouple faults
        fault_dict = self.thermocouple.fault
        for fault_type in ['cj_range', 'tc_range', 'cj_high', 'cj_low', 
                          'tc_high', 'tc_low', 'voltage', 'open_tc']:
            if fault_dict.get(fault_type, False):
                logging.error(f"Thermocouple fault: {fault_type} - EMERGENCY STOP")
                return False
        
        return True
    
    def control_relay(self, duty_cycle):
        """
        Control relay using PWM-like behavior
        duty_cycle: 0-100 percentage
        """
        if self.emergency_stop:
            self.relay.value = False
            return
        
        # Simple time-proportional control
        cycle_time = 10.0  # 10 second cycle
        on_time = (duty_cycle / 100.0) * cycle_time
        
        if on_time > 0:
            self.relay.value = True
            time.sleep(on_time)
        
        if on_time < cycle_time:
            self.relay.value = False
            time.sleep(cycle_time - on_time)
    
    def load_schedule(self, schedule_file):
        """
        Load firing schedule from JSON file
        Format: [
            {"rate": 100, "target": 600, "hold": 0},
            {"rate": 150, "target": 1000, "hold": 10},
            {"rate": 100, "target": 1240, "hold": 20}
        ]
        rate: degrees C per hour
        target: target temperature in C
        hold: minutes to hold at target temp
        """
        try:
            with open(schedule_file, 'r') as f:
                self.schedule = json.load(f)
            logging.info(f"Loaded firing schedule with {len(self.schedule)} segments")
            return True
        except Exception as e:
            logging.error(f"Failed to load schedule: {e}")
            return False
    
    def save_schedule(self, schedule_name, schedule_data):
        """Save a firing schedule to file"""
        try:
            filename = f"schedules/{schedule_name}.json"
            import os
            os.makedirs('schedules', exist_ok=True)
            with open(filename, 'w') as f:
                json.dump(schedule_data, f, indent=2)
            logging.info(f"Saved schedule: {schedule_name}")
            return True
        except Exception as e:
            logging.error(f"Failed to save schedule: {e}")
            return False
    
    def list_schedules(self):
        """List all available firing schedules"""
        try:
            import os
            os.makedirs('schedules', exist_ok=True)
            schedules = []
            for filename in os.listdir('schedules'):
                if filename.endswith('.json'):
                    name = filename[:-5]  # Remove .json extension
                    with open(f"schedules/{filename}", 'r') as f:
                        data = json.load(f)
                    schedules.append({
                        'name': name,
                        'segments': len(data),
                        'max_temp': max([seg['target'] for seg in data]) if data else 0
                    })
            return schedules
        except Exception as e:
            logging.error(f"Failed to list schedules: {e}")
            return []
    
    def delete_schedule(self, schedule_name):
        """Delete a firing schedule"""
        try:
            import os
            filename = f"schedules/{schedule_name}.json"
            if os.path.exists(filename):
                os.remove(filename)
                logging.info(f"Deleted schedule: {schedule_name}")
                return True
            return False
        except Exception as e:
            logging.error(f"Failed to delete schedule: {e}")
            return False
    
    def calculate_setpoint(self, current_temp, elapsed_time):
        """
        Calculate current setpoint based on firing schedule
        Returns (setpoint, segment_complete)
        """
        if not self.schedule or self.current_segment >= len(self.schedule):
            return current_temp, True
        
        segment = self.schedule[self.current_segment]
        rate = segment['rate'] / 3600.0  # Convert C/hour to C/second
        target = segment['target']
        hold_time = segment['hold'] * 60  # Convert minutes to seconds
        
        # Calculate expected temperature based on ramp rate
        start_temp = self.segment_start_temp
        
        temp_rise = target - start_temp
        ramp_time = abs(temp_rise) / rate if rate > 0 else 0
        
        if elapsed_time < ramp_time:
            # Still ramping
            setpoint = start_temp + (rate * elapsed_time * (1 if temp_rise > 0 else -1))
        elif elapsed_time < ramp_time + hold_time:
            # Holding at target
            setpoint = target
        else:
            # Move to next segment
            self.current_segment += 1
            if self.current_segment < len(self.schedule):
                self.segment_start_temp = target
                self.segment_start_time = time.time()
                logging.info(f"Moving to segment {self.current_segment + 1}, start temp: {target}°C")
                return self.calculate_setpoint(current_temp, 0)
            else:
                return current_temp, True
        
        return setpoint, False
    
    def run_firing(self):
        """Main firing control loop"""
        if not self.schedule:
            logging.error("No firing schedule loaded")
            return
        
        self.firing_active = True
        self.start_time = time.time()
        self.current_segment = 0
        self.emergency_stop = False
        
        # Initialize segment tracking
        initial_temp = self.read_temperature()
        self.segment_start_time = self.start_time
        self.segment_start_temp = initial_temp if initial_temp else 20.0
        
        logging.info("Starting firing cycle")
        logging.info(f"Schedule loaded with {len(self.schedule)} segments")
        logging.info(f"Schedule: {self.schedule}")
        logging.info(f"Initial temperature: {self.segment_start_temp:.1f}°C")
        
        try:
            while self.firing_active and not self.emergency_stop:
                current_temp = self.read_temperature()
                elapsed_time = time.time() - self.segment_start_time
                
                logging.info(f"Loop iteration - segment elapsed: {elapsed_time:.1f}s, temp: {current_temp}°C")
                
                # Safety check
                if not self.check_safety(current_temp):
                    self.emergency_stop = True
                    self.relay.value = False
                    logging.error("EMERGENCY STOP ACTIVATED")
                    break
                
                # Calculate setpoint
                setpoint, complete = self.calculate_setpoint(
                    current_temp, 
                    elapsed_time
                )
                
                logging.info(f"Calculated setpoint: {setpoint:.1f}°C, complete: {complete}")
                
                if complete:
                    logging.info("Firing schedule complete")
                    self.firing_active = False
                    break
                
                # Update PID
                self.pid.setpoint = setpoint
                control_output = self.pid(current_temp)
                
                logging.info(f"PID output: {control_output:.1f}%")
                
                # Update state for web interface
                with self.state_lock:
                    self.current_setpoint = setpoint
                    self.current_output = control_output
                
                # Log data
                log_entry = {
                    'time': datetime.now().isoformat(),
                    'elapsed': time.time() - self.start_time,
                    'temp': current_temp,
                    'setpoint': setpoint,
                    'output': control_output,
                    'segment': self.current_segment
                }
                self.data_log.append(log_entry)
                logging.info(f"Temp: {current_temp:.1f}°C | Setpoint: {setpoint:.1f}°C | Output: {control_output:.1f}%")
                
                # Save state periodically for crash recovery
                self.save_state()
                
                # Control relay
                logging.info(f"Calling control_relay with duty_cycle: {control_output:.1f}%")
                self.control_relay(control_output)
                
        except KeyboardInterrupt:
            logging.info("Firing interrupted by user")
        finally:
            self.relay.value = False
            self.save_data_log()
            # Clear saved state on normal completion
            try:
                import os
                if os.path.exists('firing_state.json'):
                    os.remove('firing_state.json')
            except:
                pass
            logging.info("Kiln powered off")
    
    def save_data_log(self, filename='kiln_data.json'):
        """Save firing data to file"""
        try:
            with open(filename, 'w') as f:
                json.dump(self.data_log, f, indent=2)
            logging.info(f"Data log saved to {filename}")
        except Exception as e:
            logging.error(f"Failed to save data log: {e}")
    
    def get_state(self):
        """Get current state for web interface"""
        with self.state_lock:
            return {
                'temperature': self.current_temp,
                'setpoint': self.current_setpoint,
                'output': self.current_output,
                'firing_active': self.firing_active,
                'emergency_stop': self.emergency_stop,
                'current_segment': self.current_segment,
                'total_segments': len(self.schedule),
                'timestamp': datetime.now().isoformat(),
                'pid_kp': self.pid.Kp,
                'pid_ki': self.pid.Ki,
                'pid_kd': self.pid.Kd,
                'schedule': self.schedule,
                'start_time': self.start_time,
                'segment_start_temp': self.segment_start_temp
            }
    
    def set_pid_tunings(self, kp, ki, kd):
        """Update PID tuning parameters"""
        self.pid.tunings = (kp, ki, kd)
        logging.info(f"PID tunings updated: Kp={kp}, Ki={ki}, Kd={kd}")
        return True
    
    def get_data_log(self, limit=100):
        """Get recent data log entries"""
        return self.data_log[-limit:] if limit else self.data_log
    
    def save_state(self):
        """Save current firing state for crash recovery"""
        if not self.firing_active:
            return
        
        state = {
            'firing_active': self.firing_active,
            'schedule': self.schedule,
            'current_segment': self.current_segment,
            'start_time': self.start_time,
            'segment_start_time': self.segment_start_time,
            'segment_start_temp': self.segment_start_temp,
            'emergency_stop': self.emergency_stop,
            'data_log': self.data_log
        }
        
        try:
            with open('firing_state.json', 'w') as f:
                json.dump(state, f, indent=2, default=str)
        except Exception as e:
            logging.error(f"Failed to save firing state: {e}")
    
    def load_state(self):
        """Load saved firing state"""
        try:
            import os
            if not os.path.exists('firing_state.json'):
                return None
            
            with open('firing_state.json', 'r') as f:
                state = json.load(f)
            
            # Check if state is recent (within 1 hour)
            if state.get('start_time'):
                state_age = time.time() - float(state['start_time'])
                if state_age > 3600:  # More than 1 hour old
                    logging.info("Saved state is too old, ignoring")
                    return None
            
            return state
        except Exception as e:
            logging.error(f"Failed to load firing state: {e}")
            return None
    
    def resume_firing(self, state):
        """Resume firing from saved state"""
        logging.info("Resuming firing from saved state")
        
        self.schedule = state['schedule']
        self.current_segment = state['current_segment']
        self.start_time = float(state['start_time'])
        self.segment_start_time = float(state['segment_start_time'])
        self.segment_start_temp = state['segment_start_temp']
        self.data_log = state.get('data_log', [])
        
        logging.info(f"Resuming at segment {self.current_segment + 1}/{len(self.schedule)}")
        logging.info(f"Elapsed time: {(time.time() - self.start_time)/60:.1f} minutes")
        
        # Continue firing
        self.firing_active = True
        self.emergency_stop = False
        
        try:
            while self.firing_active and not self.emergency_stop:
                current_temp = self.read_temperature()
                elapsed_time = time.time() - self.segment_start_time
                
                logging.info(f"Loop iteration - segment elapsed: {elapsed_time:.1f}s, temp: {current_temp}°C")
                
                # Safety check
                if not self.check_safety(current_temp):
                    self.emergency_stop = True
                    self.relay.value = False
                    logging.error("EMERGENCY STOP ACTIVATED")
                    break
                
                # Calculate setpoint
                setpoint, complete = self.calculate_setpoint(
                    current_temp, 
                    elapsed_time
                )
                
                logging.info(f"Calculated setpoint: {setpoint:.1f}°C, complete: {complete}")
                
                if complete:
                    logging.info("Firing schedule complete")
                    self.firing_active = False
                    break
                
                # Update PID
                self.pid.setpoint = setpoint
                control_output = self.pid(current_temp)
                
                logging.info(f"PID output: {control_output:.1f}%")
                
                # Update state for web interface
                with self.state_lock:
                    self.current_setpoint = setpoint
                    self.current_output = control_output
                
                # Log data
                log_entry = {
                    'time': datetime.now().isoformat(),
                    'elapsed': time.time() - self.start_time,
                    'temp': current_temp,
                    'setpoint': setpoint,
                    'output': control_output,
                    'segment': self.current_segment
                }
                self.data_log.append(log_entry)
                logging.info(f"Temp: {current_temp:.1f}°C | Setpoint: {setpoint:.1f}°C | Output: {control_output:.1f}%")
                
                # Save state periodically
                self.save_state()
                
                # Control relay
                logging.info(f"Calling control_relay with duty_cycle: {control_output:.1f}%")
                self.control_relay(control_output)
                
        except KeyboardInterrupt:
            logging.info("Firing interrupted by user")
        finally:
            self.relay.value = False
            self.save_data_log()
            # Clear saved state on normal completion
            try:
                import os
                if os.path.exists('firing_state.json'):
                    os.remove('firing_state.json')
            except:
                pass
            logging.info("Kiln powered off")
        """Simple manual control mode"""
        logging.info(f"Starting manual mode: {target_temp}°C for {duration_minutes} minutes")
        
        self.pid.setpoint = target_temp
        start_time = time.time()
        end_time = start_time + (duration_minutes * 60)
        
        try:
            while time.time() < end_time and not self.emergency_stop:
                current_temp = self.read_temperature()
                
                if not self.check_safety(current_temp):
                    self.emergency_stop = True
                    self.relay.value = False
                    break
                
                control_output = self.pid(current_temp)
                logging.info(f"Temp: {current_temp:.1f}°C | Target: {target_temp}°C | Output: {control_output:.1f}%")
                
                self.control_relay(control_output)
                
        except KeyboardInterrupt:
            logging.info("Manual mode interrupted")
        finally:
            self.relay.value = False
            logging.info("Manual mode ended")
    
    def autotune_pid(self):
        """
        Auto-tune PID parameters using step response test
        Returns suggested Kp, Ki, Kd values
        """
        logging.info("Starting PID auto-tune")
        
        # Get starting temperature
        start_temp = self.read_temperature()
        if start_temp is None:
            logging.error("Cannot read starting temperature")
            return None
        
        logging.info(f"Starting temperature: {start_temp:.1f}°C")
        
        # Run step response test - apply 50% power
        test_duration = 600  # 10 minutes
        power_level = 50.0  # 50% power
        
        temps = []
        times = []
        start_time = time.time()
        
        logging.info(f"Applying {power_level}% power for {test_duration}s")
        
        try:
            while time.time() - start_time < test_duration:
                current_temp = self.read_temperature()
                elapsed = time.time() - start_time
                
                if not self.check_safety(current_temp):
                    self.relay.value = False
                    logging.error("Safety check failed during auto-tune")
                    return None
                
                temps.append(current_temp)
                times.append(elapsed)
                
                # Apply constant power
                cycle_time = 10.0
                on_time = (power_level / 100.0) * cycle_time
                
                self.relay.value = True
                time.sleep(on_time)
                self.relay.value = False
                time.sleep(cycle_time - on_time)
                
                logging.info(f"Auto-tune: {elapsed:.0f}s, Temp: {current_temp:.1f}°C")
            
        except KeyboardInterrupt:
            logging.info("Auto-tune interrupted")
            self.relay.value = False
            return None
        finally:
            self.relay.value = False
        
        # Analyze response and calculate PID parameters
        if len(temps) < 10:
            logging.error("Insufficient data for auto-tune")
            return None
        
        # Calculate parameters using simplified Cohen-Coon method
        initial_temp = temps[0]
        final_temp = temps[-1]
        delta_temp = final_temp - initial_temp
        
        if delta_temp < 5:
            logging.error("Temperature rise too small for reliable auto-tune")
            return None
        
        # Find time to reach 63.2% of final value (time constant)
        target_63 = initial_temp + 0.632 * delta_temp
        tau = None
        for i, temp in enumerate(temps):
            if temp >= target_63:
                tau = times[i]
                break
        
        if tau is None or tau < 10:
            logging.error("Could not determine time constant")
            return None
        
        # Process gain (change in temp per % power)
        process_gain = delta_temp / power_level
        
        # Calculate PID parameters (conservative tuning for kilns)
        # Using modified Ziegler-Nichols for slow processes
        kp = 0.9 / process_gain
        ki = kp / (3.0 * tau)
        kd = kp * tau / 10.0
        
        # Apply limits for safety
        kp = max(0.1, min(3.0, kp))
        ki = max(0.01, min(1.0, ki))
        kd = max(0.1, min(2.0, kd))
        
        logging.info(f"Auto-tune complete: Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}")
        logging.info(f"Process gain: {process_gain:.3f}°C/%, Time constant: {tau:.1f}s")
        
        return {'kp': kp, 'ki': ki, 'kd': kd}


# Global kiln controller instance
kiln = None

# Flask web application
app = Flask(__name__)
CORS(app)

@app.route('/')
def index():
    """Serve the main dashboard page"""
    return render_template('dashboard.html')

@app.route('/api/status')
def get_status():
    """Get current kiln status"""
    if kiln:
        return jsonify(kiln.get_state())
    return jsonify({'error': 'Kiln not initialized'}), 500

@app.route('/api/data')
def get_data():
    """Get historical data"""
    if kiln:
        limit = request.args.get('limit', 100, type=int)
        return jsonify(kiln.get_data_log(limit))
    return jsonify({'error': 'Kiln not initialized'}), 500

@app.route('/api/schedules', methods=['GET'])
def list_schedules():
    """List all available schedules"""
    if kiln:
        schedules = kiln.list_schedules()
        return jsonify(schedules)
    return jsonify({'error': 'Kiln not initialized'}), 500

@app.route('/api/schedules/<name>', methods=['GET', 'POST', 'DELETE'])
def manage_schedule(name):
    """Get, save, or delete a specific schedule"""
    if not kiln:
        return jsonify({'error': 'Kiln not initialized'}), 500
    
    if request.method == 'GET':
        try:
            with open(f"schedules/{name}.json", 'r') as f:
                schedule = json.load(f)
            return jsonify(schedule)
        except Exception as e:
            return jsonify({'error': str(e)}), 404
    
    elif request.method == 'POST':
        schedule_data = request.json
        if kiln.save_schedule(name, schedule_data):
            return jsonify({'success': True, 'message': f'Schedule {name} saved'})
        return jsonify({'error': 'Failed to save schedule'}), 500
    
    elif request.method == 'DELETE':
        if kiln.delete_schedule(name):
            return jsonify({'success': True, 'message': f'Schedule {name} deleted'})
        return jsonify({'error': 'Failed to delete schedule'}), 500

@app.route('/api/load-schedule/<name>', methods=['POST'])
def load_schedule_endpoint(name):
    """Load a schedule as the active schedule"""
    if not kiln:
        return jsonify({'error': 'Kiln not initialized'}), 500
    
    if kiln.load_schedule(f"schedules/{name}.json"):
        return jsonify({'success': True, 'message': f'Schedule {name} loaded'})
    return jsonify({'error': 'Failed to load schedule'}), 500

@app.route('/api/schedule', methods=['GET', 'POST'])
def handle_schedule():
    """Get or set firing schedule"""
    if not kiln:
        return jsonify({'error': 'Kiln not initialized'}), 500
    
    if request.method == 'POST':
        schedule = request.json
        kiln.schedule = schedule
        logging.info(f"Schedule updated via web interface: {len(schedule)} segments")
        return jsonify({'success': True})
    else:
        return jsonify(kiln.schedule)

@app.route('/api/start', methods=['POST'])
def start_firing():
    """Start a firing cycle"""
    if not kiln:
        return jsonify({'error': 'Kiln not initialized'}), 500
    
    if kiln.firing_active:
        return jsonify({'error': 'Firing already active'}), 400
    
    # Start firing in a separate thread
    firing_thread = threading.Thread(target=kiln.run_firing)
    firing_thread.daemon = True
    firing_thread.start()
    
    return jsonify({'success': True, 'message': 'Firing started'})

@app.route('/api/stop', methods=['POST'])
def stop_firing():
    """Stop the current firing cycle"""
    if kiln:
        kiln.firing_active = False
        kiln.emergency_stop = True
        return jsonify({'success': True, 'message': 'Firing stopped'})
    return jsonify({'error': 'Kiln not initialized'}), 500

@app.route('/api/pid', methods=['GET', 'POST'])
def handle_pid():
    """Get or set PID tuning parameters"""
    if not kiln:
        return jsonify({'error': 'Kiln not initialized'}), 500
    
    if request.method == 'GET':
        return jsonify({
            'kp': kiln.pid.Kp,
            'ki': kiln.pid.Ki,
            'kd': kiln.pid.Kd
        })
    
    elif request.method == 'POST':
        data = request.json
        kp = float(data.get('kp', kiln.pid.Kp))
        ki = float(data.get('ki', kiln.pid.Ki))
        kd = float(data.get('kd', kiln.pid.Kd))
        
        if kiln.set_pid_tunings(kp, ki, kd):
            return jsonify({'success': True, 'message': 'PID tunings updated'})
        return jsonify({'error': 'Failed to update PID tunings'}), 500

@app.route('/api/autotune', methods=['POST'])
def autotune():
    """Run PID auto-tune"""
    if not kiln:
        return jsonify({'error': 'Kiln not initialized'}), 500
    
    if kiln.firing_active:
        return jsonify({'error': 'Cannot auto-tune while firing is active'}), 400
    
    # Run auto-tune in a separate thread
    def run_autotune():
        result = kiln.autotune_pid()
        if result:
            # Store result for retrieval
            kiln.autotune_result = result
    
    autotune_thread = threading.Thread(target=run_autotune)
    autotune_thread.daemon = True
    autotune_thread.start()
    
    # Wait for completion (with timeout)
    autotune_thread.join(timeout=700)  # 11 minutes max
    
    if hasattr(kiln, 'autotune_result') and kiln.autotune_result:
        result = kiln.autotune_result
        del kiln.autotune_result
        return jsonify({
            'success': True,
            'kp': result['kp'],
            'ki': result['ki'],
            'kd': result['kd']
        })
    else:
        return jsonify({'error': 'Auto-tune failed or timed out'}), 500

@app.route('/api/resume', methods=['GET', 'POST'])
def handle_resume():
    """Check for saved state or resume firing"""
    if not kiln:
        return jsonify({'error': 'Kiln not initialized'}), 500
    
    if request.method == 'GET':
        # Check if there's a saved state
        state = kiln.load_state()
        if state:
            return jsonify({
                'has_saved_state': True,
                'schedule_length': len(state.get('schedule', [])),
                'current_segment': state.get('current_segment', 0),
                'elapsed_minutes': (time.time() - float(state.get('start_time', time.time()))) / 60
            })
        return jsonify({'has_saved_state': False})
    
    elif request.method == 'POST':
        # Resume from saved state
        state = kiln.load_state()
        if not state:
            return jsonify({'error': 'No saved state found'}), 404
        
        if kiln.firing_active:
            return jsonify({'error': 'Firing already active'}), 400
        
        # Resume in a separate thread
        resume_thread = threading.Thread(target=kiln.resume_firing, args=(state,))
        resume_thread.daemon = True
        resume_thread.start()
        
        return jsonify({'success': True, 'message': 'Firing resumed'})

@app.route('/api/history', methods=['GET'])
def get_history():
    """Get full data log from current/last firing"""
    if kiln:
        return jsonify({
            'data': kiln.data_log,
            'firing_active': kiln.firing_active
        })
    return jsonify({'error': 'Kiln not initialized'}), 500
    """Start the Flask web server"""
    logging.info(f"Starting web server on port {port}")
    app.run(host='0.0.0.0', port=port, debug=False, use_reloader=False)


if __name__ == "__main__":
    # Initialize kiln controller
    kiln = KilnController(relay_pin=23, max_temp=1300)
    
    # Check for saved state from previous run
    saved_state = kiln.load_state()
    if saved_state and saved_state.get('firing_active'):
        logging.warning("Found saved firing state from previous run")
        logging.warning("Resume firing via web interface at http://[your-pi-ip]:5000")
    
    # Start web server in a separate thread
    web_thread = threading.Thread(target=start_web_server, args=(5000,))
    web_thread.daemon = True
    web_thread.start()
    
    # Example: Load and run a firing schedule
    # kiln.load_schedule('bisque_schedule.json')
    # kiln.run_firing()
    
    # Example: Manual mode
    # kiln.manual_mode(target_temp=500, duration_minutes=30)
    
    # Simple temperature monitoring
    logging.info("Starting temperature monitoring (Ctrl+C to stop)")
    logging.info("Web interface available at http://[your-pi-ip]:5000")
    try:
        while True:
            temp = kiln.read_temperature()
            if temp:
                logging.info(f"Temperature: {temp:.1f}°C")
            time.sleep(2)
    except KeyboardInterrupt:
        logging.info("Monitoring stopped")