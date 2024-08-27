import sys
import os
import time
import logging

# Add the directory where Motors is located to the system path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from unittest.mock import Mock, patch
from pytrinamic.connections import ConnectionManager

from Delay_line import DelayLine


# Setup logging configuration to write to simulate_behavior_log.txt
logging.basicConfig(filename="simulate_behavior_log.txt", 
                    level=logging.INFO, 
                    format="%(asctime)s - %(message)s")

def simulate_motor_behavior():
    try:
        logging.info("Simulating motor behavior")

        # Mock the connection manager to bypass hardware requirement
        with patch('pytrinamic.connections.ConnectionManager.connect') as mock_connect:
            # Mock the interface returned by the connect method
            mock_interface = mock_connect.return_value.__enter__.return_value

            # Mock the DelayLine class
            mock_delay = Mock(DelayLine)

            # Mock the motor behavior within DelayLine
            mock_motor = Mock()
            mock_delay.motor = mock_motor

            # Simulate home sensor initial state (not at home)
            mock_motor.get_axis_parameter.side_effect = lambda param: 1 if param == mock_motor.AP.HomeSwitch else 0

            logging.info('Simulating homing procedure')

            # Mock setting motor parameters
            mock_motor.set_axis_parameter = Mock()

            # Start homing process (mocked)
            mock_delay.go_to_home_position()

            logging.info(f"Initial Home Sensor State: {mock_motor.get_axis_parameter(mock_motor.AP.HomeSwitch)}")

            # Simulate motor's movement towards home and then reaching the home position
            time.sleep(2)  # Simulate motor movement

            # Simulate the motor reaching the home position by inverting the home sensor signal
            mock_motor.get_axis_parameter.side_effect = lambda param: 0 if param == mock_motor.AP.HomeSwitch else 0

            logging.info(f"Home Sensor State after reaching home: {mock_motor.get_axis_parameter(mock_motor.AP.HomeSwitch)}")

            # Simulate moving the motor to the target position
            target_position = 2
            time_out = 10
            start_time = time.time()

            logging.info(f"Moving motor to target position: {target_position}")

            # Simulate motor movement
            mock_delay.move_to(axis=0, position=target_position)

            # Simulate monitoring motor movement
            while True:
                current_position = mock_delay.get_position.return_value = target_position - 0.5
                actual_velocity = mock_motor.get_axis_parameter(mock_motor.AP.ActualVelocity)
                logging.info(f"Simulated position: {current_position}, velocity: {actual_velocity}")
                time.sleep(0.2)

                # Simulate reaching the target position
                mock_delay.is_position_reached.return_value = True
                logging.info("Simulated motor reached target position")
                mock_delay.stop(axis=0)
                break

            # Simulate the return to home position
            logging.info("Simulating return to home position")
            mock_delay.go_to_home_position()

            logging.info("Simulated behavior completed successfully")

    except Exception as e:
        logging.error(f"An error occurred during simulation: {e}")
        print(f"An error occurred during simulation: {e}")

if __name__ == "__main__":
    simulate_motor_behavior()
