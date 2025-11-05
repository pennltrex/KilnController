#!/usr/bin/env python3
"""
Simple firing test to verify kiln heating
Tests the complete control loop with aggressive parameters
"""

import board
import digitalio
import adafruit_max31856
import time
from simple_pid import PID

def test_firing():
    """Test firing with aggressive PID settings"""
    
    print("=" * 60)
    print("Kiln Controller Firing Test")
    print("=" * 60)
    print("\n⚠️  WARNING: This will turn on your kiln!")
    print("Make sure your kiln is safe to operate.\n")
    
    duration = input("Enter test duration in minutes (default 5): ").strip()
    duration = int(duration) if duration else 5
    
    target_temp = input("Enter target temperature in °C (default 100): ").strip()
    target_temp = float(target_temp) if target_temp else 100.0
    
    print(f"\nTest parameters:")
    print(f"  Duration: {duration} minutes")
    print(f"  Target: {target_temp}°C")
    print(f"  Relay: GPIO 23\n")
    
    input("Press Enter to start heating...")
    
    # Initialize hardware
    print("\nInitializing hardware...")
    
    # Setup thermocouple
    spi = board.SPI()
    cs = digitalio.DigitalInOut(board.D5)
    thermocouple = adafruit_max31856.MAX31856(spi, cs)
    thermocouple.thermocouple_type = adafruit_max31856.ThermocoupleType.K
    print("✓ Thermocouple initialized")
    
    # Setup relay
    relay = digitalio.DigitalInOut(board.D23)
    relay.direction = digitalio.Direction.OUTPUT
    relay.value = False
    print("✓ Relay initialized")
    
    # Setup PID with aggressive tuning for testing
    pid = PID(5.0, 1.0, 2.0, setpoint=target_temp)
    pid.output_limits = (0, 100)
    pid.sample_time = 1.0
    print("✓ PID controller initialized")
    
    print("\n" + "=" * 60)
    print("STARTING HEATING TEST")
    print("=" * 60)
    print("Time  | Current | Target | Output | Relay | Status")
    print("-" * 60)
    
    start_time = time.time()
    end_time = start_time + (duration * 60)
    
    try:
        while time.time() < end_time:
            elapsed = time.time() - start_time
            
            # Read temperature
            current_temp = thermocouple.temperature
            if current_temp is None:
                print("ERROR: Failed to read temperature!")
                break
            
            # Calculate PID output
            control_output = pid(current_temp)
            
            # Simple on/off control based on PID output
            # Using 5 second cycles for visibility
            cycle_time = 5.0
            on_time = (control_output / 100.0) * cycle_time
            
            # Turn relay on
            if on_time > 0:
                relay.value = True
                relay_state = "ON "
            else:
                relay.value = False
                relay_state = "OFF"
            
            # Status message
            temp_diff = target_temp - current_temp
            if temp_diff > 10:
                status = "Heating..."
            elif temp_diff > 2:
                status = "Approaching target"
            else:
                status = "At target!"
            
            # Print status
            print(f"{int(elapsed):4d}s | {current_temp:6.1f}°C | {target_temp:6.1f}°C | "
                  f"{control_output:5.1f}% | {relay_state} | {status}")
            
            # Wait for on_time
            if on_time > 0:
                time.sleep(on_time)
            
            # Turn relay off for remainder of cycle
            if on_time < cycle_time:
                relay.value = False
                time.sleep(cycle_time - on_time)
        
        print("-" * 60)
        print("Test complete!")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    
    except Exception as e:
        print(f"\nERROR: {e}")
    
    finally:
        # Always turn off
        relay.value = False
        print("\n✓ Relay turned OFF")
        print("✓ Test finished\n")

if __name__ == "__main__":
    test_firing()