#!/usr/bin/env python3
"""
Pottery Kiln Controller
Uses Adafruit MAX31856 thermocouple amplifier and Raspberry Pi
Implements PID control with firing schedules and safety features
"""

import board
import digitalio
import adafruit_max31856
import time
import json
from datetime import datetime
from simple_pid import PID
import logging

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
        
    def read_temperature(self):
        """Read current temperature from thermocouple"""
        try:
            temp = self.thermocouple.temperature
            if temp is None:
                logging.error("Failed to read temperature")
                return None
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


if __name__ == "__main__":
    # Example usage
    kiln = KilnController(relay_pin=18, max_temp=1300)
    
    # Example: Load and run a firing schedule
    # kiln.load_schedule('bisque_schedule.json')
    # kiln.run_firing()
    
    # Example: Manual mode
    # kiln.manual_mode(target_temp=500, duration_minutes=30)
    
    # Simple temperature monitoring
    print("Monitoring kiln temperature (Ctrl+C to stop)")
    try:
        while True:
            temp = kiln.read_temperature()
            if temp:
                print(f"Temperature: {temp:.1f}°C")
            time.sleep(2)
    except KeyboardInterrupt:
        print("\nMonitoring stopped")