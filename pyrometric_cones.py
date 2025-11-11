"""
Pyrometric Cone Calculations for Kiln Controller

This module provides calculations for estimating pyrometric cone values based on
heat-work (temperature over time). Uses the Orton cone system which is standard
in North America.

The heat-work calculation is based on a simplified Arrhenius-type equation where
the rate of cone deformation increases exponentially with temperature.
"""

import math
from typing import Dict, List, Optional, Tuple


# Orton Cone temperature data (in Celsius)
# These are the temperatures at which cones bend at a standard heating rate (150°C/hour)
ORTON_CONES = {
    # Low temperature cones
    '022': 586,
    '021': 614,
    '020': 635,
    '019': 656,
    '018': 696,
    '017': 727,
    '016': 764,
    '015': 791,
    '014': 815,
    '013': 837,
    '012': 861,
    '011': 894,
    '010': 919,
    '09': 955,
    '08': 983,
    '07': 1008,
    '06': 1023,
    '05': 1046,
    '04': 1060,
    '03': 1101,
    '02': 1120,
    '01': 1137,
    # Mid-range cones
    '1': 1154,
    '2': 1164,
    '3': 1170,
    '4': 1186,
    '5': 1196,
    '6': 1222,
    '7': 1240,
    '8': 1263,
    '9': 1280,
    '10': 1305,
    '11': 1315,
    '12': 1326,
    '13': 1346,
    '14': 1366,
    # High temperature cones
    '15': 1431,
    '16': 1491,
    '17': 1512,
    '18': 1522,
    '19': 1541,
    '20': 1564,
}

# Activation energy constant for heat-work calculation
# This is an empirical value that controls how temperature affects heat-work
# Higher values mean temperature has more effect
ACTIVATION_ENERGY = 15000  # in Kelvin


class ConeCalculator:
    """
    Calculates pyrometric cone values based on heat-work accumulated during firing.

    Heat-work is calculated using an Arrhenius-type equation:
    HW = ∫ exp(-E/T) dt

    where:
    - HW is heat-work
    - E is activation energy (empirical constant)
    - T is temperature in Kelvin
    - t is time
    """

    def __init__(self):
        self.heat_work = 0.0
        self.last_update_time = None
        self.cone_heat_work_values = self._calculate_cone_heat_work_values()

    def _calculate_cone_heat_work_values(self) -> Dict[str, float]:
        """
        Calculate the heat-work value for each cone.

        This is done by simulating a standard firing to the cone temperature
        at 150°C/hour and calculating the accumulated heat-work.
        """
        cone_values = {}

        for cone_name, temp_c in ORTON_CONES.items():
            # Simulate firing from room temperature to cone temperature at 150°C/hour
            start_temp = 20  # °C
            rate = 150  # °C/hour

            # Calculate time to reach cone temperature (in hours)
            time_hours = (temp_c - start_temp) / rate

            # Integrate heat-work using numerical integration (trapezoidal rule)
            # Use 1-minute intervals
            intervals = int(time_hours * 60)
            dt = 1.0 / 60.0  # 1 minute in hours

            heat_work = 0.0
            for i in range(intervals):
                temp1_c = start_temp + (rate * i * dt)
                temp2_c = start_temp + (rate * (i + 1) * dt)

                temp1_k = temp1_c + 273.15
                temp2_k = temp2_c + 273.15

                # Arrhenius term
                rate1 = math.exp(-ACTIVATION_ENERGY / temp1_k)
                rate2 = math.exp(-ACTIVATION_ENERGY / temp2_k)

                # Trapezoidal integration (dt is in hours)
                heat_work += (rate1 + rate2) / 2.0 * dt * 3600  # Convert to seconds

            cone_values[cone_name] = heat_work

        return cone_values

    def reset(self):
        """Reset the heat-work accumulator."""
        self.heat_work = 0.0
        self.last_update_time = None

    def update(self, temperature_c: float, current_time: float) -> float:
        """
        Update heat-work based on current temperature and time.

        Args:
            temperature_c: Current temperature in Celsius
            current_time: Current time in seconds (typically time.time())

        Returns:
            Current accumulated heat-work value
        """
        if self.last_update_time is not None:
            dt = current_time - self.last_update_time  # seconds

            if dt > 0 and temperature_c > 20:  # Only accumulate above room temperature
                temp_k = temperature_c + 273.15

                # Arrhenius rate equation
                rate = math.exp(-ACTIVATION_ENERGY / temp_k)

                # Accumulate heat-work (dt is in seconds)
                self.heat_work += rate * dt

        self.last_update_time = current_time
        return self.heat_work

    def get_current_cone(self) -> Optional[str]:
        """
        Get the current cone value based on accumulated heat-work.

        Returns:
            Cone number as a string (e.g., '06', '10') or None if below cone 022
        """
        if self.heat_work <= 0:
            return None

        # Find the cone that matches current heat-work
        # We return the cone whose heat-work is closest to (but not exceeding) current
        closest_cone = None
        min_diff = float('inf')

        for cone_name in sorted(self.cone_heat_work_values.keys(),
                               key=lambda x: self.cone_heat_work_values[x]):
            cone_hw = self.cone_heat_work_values[cone_name]

            if cone_hw <= self.heat_work:
                diff = abs(self.heat_work - cone_hw)
                if diff < min_diff:
                    min_diff = diff
                    closest_cone = cone_name

        return closest_cone

    def get_cone_progress(self, target_cone: str) -> Optional[float]:
        """
        Get progress towards a specific cone as a percentage.

        Args:
            target_cone: Cone number (e.g., '06', '10')

        Returns:
            Progress as a percentage (0-100) or None if cone not found
        """
        if target_cone not in self.cone_heat_work_values:
            return None

        target_hw = self.cone_heat_work_values[target_cone]

        if target_hw <= 0:
            return 100.0

        progress = (self.heat_work / target_hw) * 100.0
        return min(progress, 100.0)

    def estimate_cone_between(self) -> Tuple[Optional[str], Optional[str], float]:
        """
        Estimate the current cone position between two reference cones.

        Returns:
            Tuple of (lower_cone, upper_cone, fraction)
            where fraction is 0.0-1.0 indicating position between the two cones
        """
        if self.heat_work <= 0:
            return (None, None, 0.0)

        # Get sorted list of cones by heat-work
        sorted_cones = sorted(self.cone_heat_work_values.items(),
                            key=lambda x: x[1])

        # Find the two cones we're between
        lower_cone = None
        upper_cone = None
        fraction = 0.0

        for i, (cone_name, cone_hw) in enumerate(sorted_cones):
            if cone_hw > self.heat_work:
                upper_cone = cone_name
                if i > 0:
                    lower_cone = sorted_cones[i-1][0]
                    lower_hw = sorted_cones[i-1][1]

                    # Calculate fraction between the two cones
                    if cone_hw > lower_hw:
                        fraction = (self.heat_work - lower_hw) / (cone_hw - lower_hw)
                break
            else:
                lower_cone = cone_name

        return (lower_cone, upper_cone, fraction)

    def get_all_cones(self) -> List[Dict]:
        """
        Get a list of all available cones with their temperatures.

        Returns:
            List of dicts with cone information
        """
        cones = []
        for cone_name in sorted(ORTON_CONES.keys(),
                               key=lambda x: ORTON_CONES[x]):
            cones.append({
                'name': cone_name,
                'temperature_c': ORTON_CONES[cone_name],
                'temperature_f': int(ORTON_CONES[cone_name] * 9/5 + 32),
                'heat_work': self.cone_heat_work_values[cone_name]
            })
        return cones

    def get_cone_info(self, cone_name: str) -> Optional[Dict]:
        """
        Get detailed information about a specific cone.

        Args:
            cone_name: Cone number (e.g., '06', '10')

        Returns:
            Dict with cone information or None if not found
        """
        if cone_name not in ORTON_CONES:
            return None

        temp_c = ORTON_CONES[cone_name]
        return {
            'name': cone_name,
            'temperature_c': temp_c,
            'temperature_f': int(temp_c * 9/5 + 32),
            'heat_work': self.cone_heat_work_values[cone_name],
            'current_progress': self.get_cone_progress(cone_name)
        }
