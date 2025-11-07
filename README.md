# KilnController

# Navigate to your project directory
cd ~
mkdir kiln_controller
cd kiln_controller

# Create a virtual environment
python3 -m venv venv

# Activate the virtual environment
source venv/bin/activate

# Now install the dependencies (no warnings!)
pip install adafruit-circuitpython-max31856
pip install simple-pid
pip install flask
pip install flask_cors

# When you're done working, deactivate:
# deactivate