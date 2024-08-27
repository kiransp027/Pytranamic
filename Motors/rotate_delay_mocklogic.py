import sys
import os
import time
import logging
from unittest.mock import Mock, patch

# Setup logging configuration to write to log.txt
logging.basicConfig(filename="motor_log.txt", 
                    level=logging.INFO, 
                    format="%(asctime)s - %(message)s")

# Add the directory where Motors is located to the system path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Import DelayLine and ConnectionManager (which will be mocked for testing)
from Motors.trinamic_controller import TMCM3212
from pytrinamic.connections import ConnectionManager
from Motors.Delay_line import DelayLine

def main():
    try:
        # Mock the connection manager to bypass hardware requirement
        with patch('pytrinamic.connections.ConnectionManager.connect') as mock_connect:
            # Mock the interface returned by the connect method
            mock_interface = mock_connect.return_value.__enter__.return_value

            # Create a mock for the DelayLine
            mock_delay = Mock(DelayLine)
            
            # Create a mock for the motor inside DelayLine
            mock_motor = Mock()
            mock_delay.motor = mock_motor

            # Simulate home sensor initial state (not at home)
            mock_motor.get_axis_parameter.side_effect = lambda param: 1 if param == mock_motor.AP.HomeSwitch else 0

            logging.info('Starting homing procedure')

            # Simulate setting parameters for homing
            mock_motor.set_axis_parameter = Mock()
            mock_motor.AP = Mock()  # Mock the AP (axis parameter) object to avoid missing attribute errors
            
            # Set mock attributes to simulate behavior
            mock_motor.AP.HomeSwitch = 1
            mock_motor.AP.ActualPosition = 2

            # Start homing process: Set some mock parameters
            mock_delay.go_to_home_position()

            # Log initial sensor state
            logging.info(f"Initial Home Sensor State: {mock_motor.get_axis_parameter(mock_motor.AP.HomeSwitch)}")
            logging.info(f"Setting parameters for homing: MaxVelocity = 30000, MaxAcceleration = 30000")

            # Simulate motor's movement towards home and then reaching the home position
            time.sleep(2)  # Simulate the motor moving for 2 seconds
            
            # Simulate the motor reaching the home position by inverting the home sensor signal
            mock_motor.get_axis_parameter.side_effect = lambda param: 0 if param == mock_motor.AP.HomeSwitch else 0

            logging.info(f"Home Sensor State after reaching home: {mock_motor.get_axis_parameter(mock_motor.AP.HomeSwitch)}")
            logging.info(f"Motor reached home position, setting position to 0")

            # Set motor's actual position to 0
            mock_motor.actual_position = 0

            # Simulate setting the position to 0
            mock_motor.set_axis_parameter.assert_called_with(mock_motor.AP.ActualPosition, 0)
            logging.info(f"Motor actual position set to 0")

    except Exception as e:
        logging.error(f"An error occurred: {e}")
        print(f"An error occurred: {e}")

print('\nReady')

if __name__ == "__main__":
    main()
