#!/usr/bin/env python3
"""
Test script for Cone Fire Mode functionality
Demonstrates the cone fire schedule generation without requiring hardware
"""

from pyrometric_cones import ORTON_CONES
import json


def generate_cone_fire_schedule(cone_number, speed='medium', preheat_hours=0, hold_minutes=0):
    """
    Generate a cone fire schedule based on Orton standards.

    Args:
        cone_number: Cone number (e.g., '06', '6', '10')
        speed: Firing speed - 'slow', 'medium', or 'fast'
        preheat_hours: Optional preheat/candling time in hours at low temperature
        hold_minutes: Optional hold time at peak temperature in minutes

    Returns:
        List of schedule segments or None if cone not found
    """
    # Validate cone number
    if cone_number not in ORTON_CONES:
        print(f"Error: Invalid cone number: {cone_number}")
        return None

    # Get target temperature for the cone
    target_temp_c = ORTON_CONES[cone_number]

    # Define ramp rates for different speeds (in °C/hour)
    speed_rates = {
        'slow': {'water_smoking': 28, 'mid_ramp': 56, 'final_ramp': 44},
        'medium': {'water_smoking': 44, 'mid_ramp': 111, 'final_ramp': 67},
        'fast': {'water_smoking': 56, 'mid_ramp': 167, 'final_ramp': 111}
    }

    if speed not in speed_rates:
        print(f"Error: Invalid speed: {speed}. Must be 'slow', 'medium', or 'fast'")
        return None

    rates = speed_rates[speed]

    # Build the schedule
    schedule = []

    # Optional preheat/candling segment
    if preheat_hours > 0:
        preheat_temp = 95  # °C (about 200°F)
        schedule.append({
            "rate": rates['water_smoking'],
            "target": preheat_temp,
            "hold": int(preheat_hours * 60)
        })

    # Water smoking phase
    water_smoking_temp = 121  # °C
    schedule.append({
        "rate": rates['water_smoking'],
        "target": water_smoking_temp,
        "hold": 0
    })

    # Mid-fire ramp
    mid_temp = 538  # °C
    if target_temp_c > mid_temp:
        schedule.append({
            "rate": rates['mid_ramp'],
            "target": mid_temp,
            "hold": 0
        })

    # Final ramp to cone temperature
    schedule.append({
        "rate": rates['final_ramp'],
        "target": target_temp_c,
        "hold": hold_minutes
    })

    return schedule


def print_schedule_info(cone_number, speed, preheat_hours, hold_minutes):
    """Print detailed information about a cone fire schedule"""
    print(f"\n{'='*70}")
    print(f"Cone Fire Schedule: Cone {cone_number} ({speed} speed)")
    print(f"{'='*70}")

    target_temp_c = ORTON_CONES[cone_number]
    target_temp_f = int(target_temp_c * 9/5 + 32)

    print(f"\nTarget Cone: {cone_number}")
    print(f"Target Temperature: {target_temp_c}°C ({target_temp_f}°F)")
    print(f"Firing Speed: {speed}")
    print(f"Preheat Time: {preheat_hours} hours")
    print(f"Hold at Peak: {hold_minutes} minutes")

    schedule = generate_cone_fire_schedule(cone_number, speed, preheat_hours, hold_minutes)

    if schedule is None:
        print("Failed to generate schedule")
        return

    print(f"\nSchedule Segments ({len(schedule)} total):")
    print("-" * 70)

    total_time = 0
    for i, seg in enumerate(schedule):
        if i == 0:
            start_temp = 20  # Room temperature
        else:
            start_temp = schedule[i-1]['target']

        ramp_time = abs(seg['target'] - start_temp) / seg['rate']  # Hours
        total_time += ramp_time + (seg['hold'] / 60)

        print(f"\nSegment {i+1}:")
        print(f"  Ramp from {start_temp}°C to {seg['target']}°C at {seg['rate']}°C/hr")
        print(f"  Ramp time: {ramp_time:.2f} hours")
        print(f"  Hold time: {seg['hold']} minutes")

    print(f"\n{'='*70}")
    print(f"Total estimated firing time: {total_time:.2f} hours ({int(total_time*60)} minutes)")
    print(f"{'='*70}\n")


def main():
    """Run test examples"""
    print("Cone Fire Mode - Test Examples")
    print("=" * 70)

    # Test 1: Bisque firing to cone 04 with preheat
    print_schedule_info('04', 'medium', preheat_hours=2, hold_minutes=30)

    # Test 2: Glaze firing to cone 6 without preheat
    print_schedule_info('6', 'medium', preheat_hours=0, hold_minutes=15)

    # Test 3: Low-fire glaze to cone 06 fast
    print_schedule_info('06', 'fast', preheat_hours=0, hold_minutes=10)

    # Test 4: High-fire to cone 10 slow
    print_schedule_info('10', 'slow', preheat_hours=0, hold_minutes=20)

    # List all available cones
    print("\nAvailable Cones:")
    print("=" * 70)
    for cone_name in sorted(ORTON_CONES.keys(), key=lambda x: ORTON_CONES[x]):
        temp_c = ORTON_CONES[cone_name]
        temp_f = int(temp_c * 9/5 + 32)
        print(f"Cone {cone_name:>3}: {temp_c:>4}°C ({temp_f:>4}°F)")
    print("=" * 70)


if __name__ == '__main__':
    main()
