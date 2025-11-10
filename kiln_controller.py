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
import os
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

# Default configuration
DEFAULT_CONFIG = {
    "hardware": {
        "relay_pin": 23,
        "thermocouple_cs_pin": "D5",
        "thermocouple_type": "K"
    },
    "safety": {
        "max_temp": 1300,
        "safety_margin": 50
    },
    "pid": {
        "kp": 1.0,
        "ki": 0.1,
        "kd": 0.5,
        "sample_time": 1.0,
        "output_limits": [0, 100]
    },
    "control": {
        "relay_cycle_time": 10.0,
        "temp_update_interval": 2.0
    },
    "web": {
        "host": "0.0.0.0",
        "port": 5000
    }
}

def load_config(config_file='config.json'):
    """Load configuration from file, create with defaults if doesn't exist"""
    if os.path.exists(config_file):
        try:
            with open(config_file, 'r') as f:
                config = json.load(f)
            logging.info(f"Configuration loaded from {config_file}")
            return config
        except Exception as e:
            logging.error(f"Failed to load config: {e}, using defaults")
            return DEFAULT_CONFIG.copy()
    else:
        # Create default config file
        try:
            with open(config_file, 'w') as f:
                json.dump(DEFAULT_CONFIG, f, indent=4)
            logging.info(f"Created default configuration file: {config_file}")
            return DEFAULT_CONFIG.copy()
        except Exception as e:
            logging.error(f"Failed to create config file: {e}, using defaults")
            return DEFAULT_CONFIG.copy()

def save_config(config, config_file='config.json'):
    """Save configuration to file"""
    try:
        with open(config_file, 'w') as f:
            json.dump(config, f, indent=4)
        logging.info(f"Configuration saved to {config_file}")
        return True
    except Exception as e:
        logging.error(f"Failed to save config: {e}")
        return False

class KilnController:
    def __init__(self, config=None):
        """
        Initialize the kiln controller
        
        Args:
            config: Configuration dictionary (uses defaults if None)
        """
        # Load configuration
        self.config = config if config else DEFAULT_CONFIG.copy()
        
        # Extract config values
        hw_config = self.config['hardware']
        safety_config = self.config['safety']
        pid_config = self.config['pid']
        control_config = self.config['control']
        
        # Initialize SPI and MAX31856
        spi = board.SPI()
        cs_pin = hw_config['thermocouple_cs_pin']
        cs = digitalio.DigitalInOut(getattr(board, cs_pin))
        self.thermocouple = adafruit_max31856.MAX31856(spi, cs)
        
        # Set thermocouple type
        tc_type = hw_config['thermocouple_type']
        tc_type_map = {
            'B': adafruit_max31856.ThermocoupleType.B,
            'E': adafruit_max31856.ThermocoupleType.E,
            'J': adafruit_max31856.ThermocoupleType.J,
            'K': adafruit_max31856.ThermocoupleType.K,
            'N': adafruit_max31856.ThermocoupleType.N,
            'R': adafruit_max31856.ThermocoupleType.R,
            'S': adafruit_max31856.ThermocoupleType.S,
            'T': adafruit_max31856.ThermocoupleType.T
        }
        self.thermocouple.thermocouple_type = tc_type_map.get(tc_type, adafruit_max31856.ThermocoupleType.K)
        
        # Setup relay control (for SSR - Solid State Relay)
        relay_pin = hw_config['relay_pin']
        self.relay = digitalio.DigitalInOut(getattr(board, f'D{relay_pin}'))
        self.relay.direction = digitalio.Direction.OUTPUT
        self.relay.value = False
        
        # Safety parameters
        self.max_temp = safety_config['max_temp']
        self.safety_margin = safety_config['safety_margin']
        self.emergency_stop = False
        
        # Control parameters
        self.relay_cycle_time = control_config['relay_cycle_time']
        
        # PID controller setup
        self.pid = PID(
            pid_config['kp'],
            pid_config['ki'],
            pid_config['kd'],
            setpoint=0
        )
        self.pid.output_limits = tuple(pid_config['output_limits'])
        self.pid.sample_time = pid_config['sample_time']
        
        logging.info(f"PID initialized: Kp={pid_config['kp']}, Ki={pid_config['ki']}, Kd={pid_config['kd']}")
        logging.info(f"Hardware: Relay=GPIO{relay_pin}, Thermocouple={tc_type}-type on {cs_pin}")
        logging.info(f"Safety: Max={self.max_temp}°C, Margin={self.safety_margin}°C")
        
        # Firing schedule
        self.schedule = []
        self.current_schedule_name = None  # Track which schedule is loaded
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
        cycle_time = self.relay_cycle_time
        on_time = (duty_cycle / 100.0) * cycle_time
        
        if on_time > 0:
            self.relay.value = True
            time.sleep(on_time)
        
        if on_time < cycle_time:
            self.relay.value = False
            time.sleep(cycle_time - on_time)
    
    def load_schedule(self, schedule_file, schedule_name=None):
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
            # Extract schedule name from file path if not provided
            if schedule_name is None:
                schedule_name = os.path.basename(schedule_file).replace('.json', '')
            self.current_schedule_name = schedule_name
            logging.info(f"Loaded firing schedule '{schedule_name}' with {len(self.schedule)} segments")
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

        # Check if we're resuming from a saved state
        # Only resume if firing was already active (e.g., after system restart)
        # If firing_active is False, this is a fresh start even if old state exists
        resuming = (self.firing_active and
                    self.start_time is not None and
                    self.segment_start_time is not None)

        if not resuming:
            # Fresh start - initialize everything
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
        else:
            # Resuming - preserve existing state, only set firing_active
            self.firing_active = True
            current_temp = self.read_temperature()
            logging.info("Resuming firing cycle")
            logging.info(f"Schedule: {self.current_schedule_name}")
            logging.info(f"Resuming from segment {self.current_segment + 1}/{len(self.schedule)}")
            logging.info(f"Segment start temp: {self.segment_start_temp:.1f}°C")
            logging.info(f"Current temperature: {current_temp:.1f}°C")

        # Counter for periodic state saves
        iteration_count = 0
        state_save_interval = 5  # Save state every 5 iterations (~50 seconds with 10s relay cycle)
        data_save_interval = 30  # Save data log every 30 iterations (~5 minutes)

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

                # Periodically save firing state and data log
                iteration_count += 1
                if iteration_count % state_save_interval == 0:
                    self.save_firing_state()

                if iteration_count % data_save_interval == 0:
                    self.save_data_log()
                    logging.info(f"Progress: {len(self.data_log)} data points logged")

                # Control relay
                logging.info(f"Calling control_relay with duty_cycle: {control_output:.1f}%")
                self.control_relay(control_output)

        except KeyboardInterrupt:
            logging.info("Firing interrupted by user")
        finally:
            self.relay.value = False
            self.save_data_log()
            self.clear_firing_state()  # Clear state after successful completion or stop
            logging.info("Kiln powered off")
    
    def save_data_log(self, filename='kiln_data.json'):
        """Save firing data to file"""
        try:
            with open(filename, 'w') as f:
                json.dump(self.data_log, f, indent=2)
            logging.info(f"Data log saved to {filename}")
        except Exception as e:
            logging.error(f"Failed to save data log: {e}")

    def load_data_log(self, filename='kiln_data.json'):
        """Load firing data from file (for crash recovery)"""
        try:
            if os.path.exists(filename):
                with open(filename, 'r') as f:
                    self.data_log = json.load(f)
                logging.info(f"Loaded {len(self.data_log)} data points from {filename}")
                return True
            return False
        except Exception as e:
            logging.error(f"Failed to load data log: {e}")
            return False

    def save_firing_state(self, filename='firing_state.json'):
        """Save current firing state to disk for crash recovery"""
        try:
            state = {
                'firing_active': self.firing_active,
                'start_time': self.start_time,
                'current_segment': self.current_segment,
                'segment_start_time': self.segment_start_time,
                'segment_start_temp': self.segment_start_temp,
                'schedule': self.schedule,
                'schedule_name': self.current_schedule_name,  # Save the schedule name
                'emergency_stop': self.emergency_stop,
                'timestamp': datetime.now().isoformat()
            }
            with open(filename, 'w') as f:
                json.dump(state, f, indent=2)
            logging.debug(f"Firing state saved: segment {self.current_segment}/{len(self.schedule)}")
        except Exception as e:
            logging.error(f"Failed to save firing state: {e}")

    def load_firing_state(self, filename='firing_state.json'):
        """Load saved firing state from disk"""
        try:
            if not os.path.exists(filename):
                return None

            with open(filename, 'r') as f:
                state = json.load(f)

            # Validate state
            if not state.get('firing_active'):
                logging.info("Previous firing was not active, clearing state file")
                os.remove(filename)
                return None

            logging.info(f"Found saved firing state from {state.get('timestamp')}")
            logging.info(f"  Segment: {state.get('current_segment')}/{len(state.get('schedule', []))}")
            logging.info(f"  Elapsed time: {time.time() - state.get('start_time', 0):.1f}s")

            return state
        except Exception as e:
            logging.error(f"Failed to load firing state: {e}")
            return None

    def resume_firing(self, state):
        """Resume firing from saved state"""
        try:
            self.schedule = state['schedule']
            self.current_schedule_name = state.get('schedule_name')  # Restore the schedule name
            self.current_segment = state['current_segment']
            self.start_time = state['start_time']
            self.segment_start_time = state['segment_start_time']
            self.segment_start_temp = state['segment_start_temp']
            self.emergency_stop = state.get('emergency_stop', False)

            logging.info("Firing state restored successfully")
            logging.info(f"Schedule: {self.current_schedule_name}")
            logging.info(f"Resuming from segment {self.current_segment + 1}/{len(self.schedule)}")

            return True
        except Exception as e:
            logging.error(f"Failed to resume firing state: {e}")
            return False

    def clear_firing_state(self, filename='firing_state.json'):
        """Clear saved firing state (called after successful completion)"""
        try:
            if os.path.exists(filename):
                os.remove(filename)
                logging.info("Firing state cleared")
        except Exception as e:
            logging.error(f"Failed to clear firing state: {e}")
    
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
                'autotune_active': getattr(self, 'autotune_active', False),
                'start_time': datetime.fromtimestamp(self.start_time).isoformat() if self.start_time else None,
                'schedule': self.schedule
            }
    
    def set_pid_tunings(self, kp, ki, kd):
        """Update PID tuning parameters and save to config"""
        self.pid.tunings = (kp, ki, kd)
        self.config['pid']['kp'] = kp
        self.config['pid']['ki'] = ki
        self.config['pid']['kd'] = kd
        save_config(self.config)
        logging.info(f"PID tunings updated and saved: Kp={kp}, Ki={ki}, Kd={kd}")
        return True
    
    def autotune_pid(self, target_temp=100, test_duration=900):
        """
        Auto-tune PID using relay method (Ziegler-Nichols)
        
        Args:
            target_temp: Temperature to heat to for testing (default 100°C)
            test_duration: Maximum test duration in seconds (default 15 min)
        """
        self.autotune_active = True
        logging.info(f"Starting PID auto-tune: target={target_temp}°C, duration={test_duration}s")
        
        try:
            # Heat to target with relay control
            start_time = time.time()
            temps = []
            times = []
            
            # Phase 1: Heat up with relay on
            logging.info("Auto-tune Phase 1: Heating")
            while time.time() - start_time < test_duration / 2:
                current_temp = self.read_temperature()
                if current_temp is None:
                    logging.error("Auto-tune failed: cannot read temperature")
                    return None
                
                temps.append(current_temp)
                times.append(time.time() - start_time)
                
                if current_temp >= target_temp:
                    break
                
                # Full power heating
                self.relay.value = True
                time.sleep(2)
            
            self.relay.value = False
            peak_temp = max(temps) if temps else target_temp
            
            # Phase 2: Let it cool slightly and oscillate
            logging.info("Auto-tune Phase 2: Oscillation detection")
            oscillation_data = []
            relay_state = False
            
            while time.time() - start_time < test_duration:
                current_temp = self.read_temperature()
                if current_temp is None:
                    break
                
                elapsed = time.time() - start_time
                oscillation_data.append((elapsed, current_temp))
                
                # Simple relay control around target
                if current_temp < target_temp - 5:
                    relay_state = True
                elif current_temp > target_temp + 5:
                    relay_state = False
                
                self.relay.value = relay_state
                time.sleep(2)
                
                # Stop if we have enough data (3+ oscillations)
                if len(oscillation_data) > 60:  # ~2 minutes of data
                    break
            
            self.relay.value = False
            
            # Analyze oscillations to find period and amplitude
            if len(oscillation_data) < 20:
                logging.error("Auto-tune failed: insufficient data")
                return None
            
            # Simple peak detection
            temps_only = [t[1] for t in oscillation_data]
            peaks = []
            for i in range(1, len(temps_only) - 1):
                if temps_only[i] > temps_only[i-1] and temps_only[i] > temps_only[i+1]:
                    peaks.append(i)
            
            if len(peaks) < 2:
                logging.warning("Auto-tune: could not detect oscillations, using conservative defaults")
                # Conservative defaults for kilns
                return (0.8, 0.05, 1.2)
            
            # Calculate oscillation period
            peak_times = [oscillation_data[p][0] for p in peaks]
            periods = [peak_times[i+1] - peak_times[i] for i in range(len(peak_times)-1)]
            avg_period = sum(periods) / len(periods) if periods else 60
            
            # Calculate oscillation amplitude
            peak_temps = [temps_only[p] for p in peaks]
            amplitude = (max(peak_temps) - min(peak_temps)) / 2 if len(peak_temps) > 1 else 10
            
            # Ziegler-Nichols tuning (conservative for kilns)
            # Ultimate gain estimation
            Ku = 4.0 / (3.14159 * amplitude) * 100  # Relay amplitude = 100%
            Tu = avg_period
            
            # Conservative PID settings (60% of Z-N recommendations for high thermal mass)
            Kp = 0.45 * Ku
            Ki = 0.54 * Ku / Tu
            Kd = 0.075 * Ku * Tu
            
            # Clamp values to reasonable ranges for kilns
            Kp = max(0.1, min(3.0, Kp))
            Ki = max(0.01, min(1.0, Ki))
            Kd = max(0.1, min(2.0, Kd))
            
            logging.info(f"Auto-tune results: Kp={Kp:.2f}, Ki={Ki:.3f}, Kd={Kd:.2f}")
            logging.info(f"Oscillation: Period={Tu:.1f}s, Amplitude={amplitude:.1f}°C")
            
            return (round(Kp, 1), round(Ki, 2), round(Kd, 1))
            
        except Exception as e:
            logging.error(f"Auto-tune error: {e}")
            return None
        finally:
            self.relay.value = False
            self.autotune_active = False
            logging.info("Auto-tune complete")
    
    def get_data_log(self, limit=100):
        """Get recent data log entries"""
        return self.data_log[-limit:] if limit else self.data_log
    
    def manual_mode(self, target_temp, duration_minutes=60):
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

    if kiln.load_schedule(f"schedules/{name}.json", schedule_name=name):
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
        kiln.current_schedule_name = 'Custom'  # Mark as custom schedule
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
def autotune_pid():
    """Run PID auto-tune"""
    if not kiln:
        return jsonify({'error': 'Kiln not initialized'}), 500
    
    if kiln.firing_active:
        return jsonify({'error': 'Cannot auto-tune while firing'}), 400
    
    data = request.json or {}
    target_temp = data.get('target_temp', 100)
    test_duration = data.get('test_duration', 900)
    
    # Run auto-tune in separate thread
    def run_autotune():
        result = kiln.autotune_pid(target_temp, test_duration)
        if result:
            kp, ki, kd = result
            kiln.set_pid_tunings(kp, ki, kd)
    
    autotune_thread = threading.Thread(target=run_autotune)
    autotune_thread.daemon = True
    autotune_thread.start()
    
    return jsonify({
        'success': True,
        'message': f'Auto-tune started: heating to {target_temp}°C for up to {test_duration/60:.0f} minutes'
    })

@app.route('/api/config', methods=['GET'])
def get_config():
    """Get current configuration"""
    if kiln:
        return jsonify(kiln.config)
    return jsonify({'error': 'Kiln not initialized'}), 500

@app.route('/api/config/pid-advanced', methods=['POST'])
def update_pid_advanced():
    """Update PID advanced settings"""
    if not kiln:
        return jsonify({'error': 'Kiln not initialized'}), 500

    data = request.json
    sample_time = float(data.get('sample_time', kiln.pid.sample_time))
    output_limits = data.get('output_limits', kiln.config['pid']['output_limits'])

    try:
        # Update PID sample time
        kiln.pid.sample_time = sample_time
        kiln.config['pid']['sample_time'] = sample_time

        # Update output limits
        kiln.pid.output_limits = tuple(output_limits)
        kiln.config['pid']['output_limits'] = output_limits

        # Save to config file
        save_config(kiln.config)

        logging.info(f"PID advanced settings updated: sample_time={sample_time}, output_limits={output_limits}")
        return jsonify({'success': True, 'message': 'PID advanced settings updated'})
    except Exception as e:
        logging.error(f"Failed to update PID advanced settings: {e}")
        return jsonify({'error': str(e)}), 500

@app.route('/api/config/safety', methods=['POST'])
def update_safety():
    """Update safety settings"""
    if not kiln:
        return jsonify({'error': 'Kiln not initialized'}), 500

    data = request.json
    max_temp = float(data.get('max_temp', kiln.max_temp))
    safety_margin = float(data.get('safety_margin', kiln.safety_margin))

    try:
        # Update kiln safety settings
        kiln.max_temp = max_temp
        kiln.safety_margin = safety_margin

        # Update config
        kiln.config['safety']['max_temp'] = max_temp
        kiln.config['safety']['safety_margin'] = safety_margin

        # Save to config file
        save_config(kiln.config)

        logging.info(f"Safety settings updated: max_temp={max_temp}, safety_margin={safety_margin}")
        return jsonify({'success': True, 'message': 'Safety settings updated'})
    except Exception as e:
        logging.error(f"Failed to update safety settings: {e}")
        return jsonify({'error': str(e)}), 500

@app.route('/api/config/control', methods=['POST'])
def update_control():
    """Update control settings"""
    if not kiln:
        return jsonify({'error': 'Kiln not initialized'}), 500

    data = request.json
    relay_cycle_time = float(data.get('relay_cycle_time', kiln.relay_cycle_time))
    temp_update_interval = float(data.get('temp_update_interval', kiln.config['control']['temp_update_interval']))

    try:
        # Update kiln control settings
        kiln.relay_cycle_time = relay_cycle_time

        # Update config
        kiln.config['control']['relay_cycle_time'] = relay_cycle_time
        kiln.config['control']['temp_update_interval'] = temp_update_interval

        # Save to config file
        save_config(kiln.config)

        logging.info(f"Control settings updated: relay_cycle_time={relay_cycle_time}, temp_update_interval={temp_update_interval}")
        return jsonify({'success': True, 'message': 'Control settings updated'})
    except Exception as e:
        logging.error(f"Failed to update control settings: {e}")
        return jsonify({'error': str(e)}), 500

@app.route('/api/current-firing', methods=['GET'])
def get_current_firing():
    """Get current firing information if a firing is active"""
    if not kiln:
        return jsonify({'error': 'Kiln not initialized'}), 500

    if not kiln.firing_active:
        return jsonify({'firing_active': False})

    # Return current firing information
    return jsonify({
        'firing_active': True,
        'schedule_name': kiln.current_schedule_name,
        'schedule': kiln.schedule,
        'data_log': kiln.data_log,
        'start_time': kiln.start_time,
        'current_segment': kiln.current_segment,
        'total_segments': len(kiln.schedule)
    })

@app.route('/api/resume-check', methods=['GET'])
def check_resume():
    """Check if there's a saved firing state that can be resumed"""
    if not kiln:
        return jsonify({'error': 'Kiln not initialized'}), 500

    state = kiln.load_firing_state()
    if state:
        return jsonify({
            'can_resume': True,
            'state': {
                'schedule_name': state.get('schedule_name', 'Unknown'),
                'segment': state.get('current_segment'),
                'total_segments': len(state.get('schedule', [])),
                'timestamp': state.get('timestamp'),
                'elapsed_time': time.time() - state.get('start_time', 0)
            }
        })
    return jsonify({'can_resume': False})

@app.route('/api/resume', methods=['POST'])
def resume_firing():
    """Resume a previously interrupted firing"""
    if not kiln:
        return jsonify({'error': 'Kiln not initialized'}), 500

    if kiln.firing_active:
        return jsonify({'error': 'Firing already active'}), 400

    # Load saved state
    state = kiln.load_firing_state()
    if not state:
        return jsonify({'error': 'No saved firing state found'}), 404

    # Restore the state
    if not kiln.resume_firing(state):
        return jsonify({'error': 'Failed to restore firing state'}), 500

    # Load previous data log
    kiln.load_data_log()

    # Start firing in a separate thread (it will continue from saved state)
    firing_thread = threading.Thread(target=kiln.run_firing)
    firing_thread.daemon = True
    firing_thread.start()

    return jsonify({
        'success': True,
        'message': f'Firing resumed from segment {state["current_segment"] + 1}/{len(state["schedule"])}',
        'schedule_name': state.get('schedule_name'),
        'schedule': state['schedule'],
        'data_log': kiln.data_log,
        'start_time': state['start_time']
    })

@app.route('/api/clear-resume', methods=['POST'])
def clear_resume():
    """Clear saved firing state (discard resume capability)"""
    if kiln:
        kiln.clear_firing_state()
        return jsonify({'success': True, 'message': 'Resume state cleared'})
    return jsonify({'error': 'Kiln not initialized'}), 500

def start_web_server(port=5000, host='0.0.0.0'):
    """Start the Flask web server"""
    logging.info(f"Starting web server on {host}:{port}")
    app.run(host=host, port=port, debug=False, use_reloader=False)


if __name__ == "__main__":
    # Load configuration
    config = load_config('config.json')
    
    # Initialize kiln controller with config
    kiln = KilnController(config=config)
    
    # Get web server settings from config
    web_config = config['web']
    control_config = config['control']
    
    # Start web server in a separate thread
    web_thread = threading.Thread(
        target=start_web_server, 
        args=(web_config['port'], web_config['host'])
    )
    web_thread.daemon = True
    web_thread.start()
    
    # Simple temperature monitoring
    update_interval = control_config['temp_update_interval']
    logging.info("Starting temperature monitoring (Ctrl+C to stop)")
    logging.info(f"Web interface available at http://[your-pi-ip]:{web_config['port']}")
    try:
        while True:
            temp = kiln.read_temperature()
            if temp:
                logging.info(f"Temperature: {temp:.1f}°C")
            time.sleep(update_interval)
    except KeyboardInterrupt:
        logging.info("Monitoring stopped")