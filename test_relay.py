#!/usr/bin/env python3
"""
Simple relay test script for GPIO 23
Tests the SSR/relay connection for the kiln controller
"""

import board
import digitalio
import time

def test_relay():
    """Test the relay by turning it on and off"""
    
    print("=" * 50)
    print("Kiln Controller Relay Test")
    print("=" * 50)
    print("\nThis script will test the relay on GPIO 23")
    print("⚠️  WARNING: Ensure your kiln is NOT connected")
    print("    or is in a safe state for testing!\n")
    
    input("Press Enter to continue...")
    
    # Setup relay on GPIO 23
    print("\nInitializing GPIO 23...")
    relay = digitalio.DigitalInOut(board.D23)
    relay.direction = digitalio.Direction.OUTPUT
    relay.value = False
    print("✓ GPIO 23 initialized (relay OFF)")
    
    try:
        # Test sequence
        print("\nStarting test sequence...")
        print("-" * 50)
        
        for i in range(3):
            print(f"\nTest {i+1}/3:")
            
            # Turn ON
            print("  → Turning relay ON")
            relay.value = True
            print("  ✓ Relay should be ON now")
            print("    (Check for LED on SSR or click sound)")
            time.sleep(2)
            
            # Turn OFF
            print("  → Turning relay OFF")
            relay.value = False
            print("  ✓ Relay should be OFF now")
            time.sleep(2)
        
        print("\n" + "-" * 50)
        print("\nTest sequence complete!")
        print("\nDid you observe the relay switching?")
        print("  - SSR LED should blink on/off")
        print("  - Mechanical relay should click")
        print("  - Multimeter should show switching\n")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    
    finally:
        # Always turn off relay
        relay.value = False
        print("\n✓ Relay turned OFF")
        print("✓ GPIO cleaned up")
        print("\nTest script finished.\n")

if __name__ == "__main__":
    test_relay()