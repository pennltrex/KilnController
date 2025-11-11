#!/usr/bin/env python3
"""
Diagnostic script to test if the pyrometric cone module works.
Run this on your Raspberry Pi to diagnose import issues.
"""

import sys
import os

print("=" * 60)
print("DIAGNOSTIC TEST FOR PYROMETRIC CONES")
print("=" * 60)
print()

# Test 1: Check Python version
print(f"1. Python version: {sys.version}")
print()

# Test 2: Check working directory
print(f"2. Current directory: {os.getcwd()}")
print(f"   Files in directory: {os.listdir('.')[:10]}")
print()

# Test 3: Check if pyrometric_cones.py exists
print("3. Checking for pyrometric_cones.py...")
if os.path.exists('pyrometric_cones.py'):
    print("   ✓ File exists")
    print(f"   File size: {os.path.getsize('pyrometric_cones.py')} bytes")
else:
    print("   ✗ FILE NOT FOUND!")
    print("   You need to pull the latest changes from git")
    sys.exit(1)
print()

# Test 4: Try importing the module
print("4. Testing import of pyrometric_cones...")
try:
    from pyrometric_cones import ConeCalculator
    print("   ✓ Import successful")
except Exception as e:
    print(f"   ✗ Import failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
print()

# Test 5: Try creating a ConeCalculator instance
print("5. Testing ConeCalculator initialization...")
try:
    calc = ConeCalculator()
    print("   ✓ Initialization successful")
    print(f"   Cones available: {len(calc.cone_heat_work_values)}")
except Exception as e:
    print(f"   ✗ Initialization failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
print()

# Test 6: Try updating with temperature
print("6. Testing cone calculation...")
try:
    import time
    calc.update(100.0, time.time())
    print("   ✓ Update successful")
    print(f"   Heat work: {calc.heat_work}")
    print(f"   Current cone: {calc.get_current_cone()}")
except Exception as e:
    print(f"   ✗ Update failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
print()

# Test 7: Check if kiln_controller.py was updated
print("7. Checking kiln_controller.py for cone import...")
try:
    with open('kiln_controller.py', 'r') as f:
        content = f.read()
        if 'from pyrometric_cones import ConeCalculator' in content:
            print("   ✓ Import statement found in kiln_controller.py")
        else:
            print("   ✗ Import statement NOT found!")
            print("   You may need to pull the latest changes")
except Exception as e:
    print(f"   ✗ Error reading kiln_controller.py: {e}")
print()

print("=" * 60)
print("ALL TESTS PASSED!")
print("The pyrometric cone module is working correctly.")
print("=" * 60)
print()
print("If the service still won't start, run this to see the actual error:")
print("  sudo journalctl -u kiln-controller.service -n 100 --no-pager")
print()
print("Or try running the controller directly:")
print("  cd /home/noexit/KilnController")
print("  ./venv/bin/python kiln_controller.py")
