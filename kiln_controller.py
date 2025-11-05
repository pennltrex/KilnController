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
        self.pid = PID(2.0, 0.5, 1.0, setpoint=0)
        self.pid.output_limits = (0, 100)  # 0-100% duty cycle
        self.pid.sample_time = 1.0  # Update every second
        
        # Firing schedule
        self.schedule = []
        self.current_segment = 0
        self.firing_active = False
        self.start_time = None
        
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
        for fault_type in ['cj_range', 'tc_range', 'cj_high', 'cj_low', 
                          'tc_high', 'tc_low', 'voltage', 'open_tc']:
            fault = getattr(self.thermocouple.fault, fault_type)
            if fault:
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
        if self.current_segment == 0:
            start_temp = current_temp if elapsed_time < 60 else 20  # Assume room temp start
        else:
            start_temp = self.schedule[self.current_segment - 1]['target']
        
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
            logging.info(f"Moving to segment {self.current_segment + 1}")
            return self.calculate_setpoint(current_temp, 0)
        
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
        
        logging.info("Starting firing cycle")
        
        try:
            while self.firing_active and not self.emergency_stop:
                current_temp = self.read_temperature()
                elapsed_time = time.time() - self.start_time
                
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
                
                if complete:
                    logging.info("Firing schedule complete")
                    self.firing_active = False
                    break
                
                # Update PID
                self.pid.setpoint = setpoint
                control_output = self.pid(current_temp)
                
                # Update state for web interface
                with self.state_lock:
                    self.current_setpoint = setpoint
                    self.current_output = control_output
                
                # Log data
                log_entry = {
                    'time': datetime.now().isoformat(),
                    'elapsed': elapsed_time,
                    'temp': current_temp,
                    'setpoint': setpoint,
                    'output': control_output,
                    'segment': self.current_segment
                }
                self.data_log.append(log_entry)
                logging.info(f"Temp: {current_temp:.1f}°C | Setpoint: {setpoint:.1f}°C | Output: {control_output:.1f}%")
                
                # Control relay
                self.control_relay(control_output)
                
        except KeyboardInterrupt:
            logging.info("Firing interrupted by user")
        finally:
            self.relay.value = False
            self.save_data_log()
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
                'timestamp': datetime.now().isoformat()
            }
    
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

def start_web_server(port=5000):
    """Start the Flask web server"""
    logging.info(f"Starting web server on port {port}")
    app.run(host='0.0.0.0', port=port, debug=False, use_reloader=False)


if __name__ == "__main__":
    # Initialize kiln controller
    kiln = KilnController(relay_pin=23, max_temp=1300)
    
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