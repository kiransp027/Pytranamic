import sys
import os
import time
import logging
from pytrinamic.connections import ConnectionManager
from Motors.Delay_line import DelayLine

# Setup logging configuration to write to rotate_delay_log.txt
logging.basicConfig(filename="rotate_delay_log.txt", 
                    level=logging.INFO, 
                    format="%(asctime)s - %(message)s")

def main():
    try:
        logging.info("Starting rotate_delay procedure")
        
        # Initialize the ConnectionManager and connect to the motor
        connection_manager = ConnectionManager()  # Use real connection here
        with connection_manager.connect() as my_interface:

            # Create an instance of DelayLine
            delay = DelayLine(my_interface)
            
            # Set target position and velocity for the motor
            target_position = 2
            time_out = 10
            start_time = time.time()

            logging.info(f"Moving the delay line to target position: {target_position}")
            
            # Move the motor to the target position
            delay.move_to(axis=0, position=target_position)
            
            # Monitor the motor's position during movement
            while not delay.is_position_reached():
                current_position = delay.get_position()
                actual_velocity = delay.motor.get_axis_parameter(delay.motor.AP.ActualVelocity)
                logging.info(f"Current position: {current_position}, velocity: {actual_velocity}")
                time.sleep(0.2)

                # Handle timeout
                if time.time() - start_time > time_out:
                    logging.warning("Timeout reached, stopping motor")
                    delay.stop(axis=0)
                    break
            else:
                logging.info("Target position reached, stopping motor")
                delay.stop(axis=0)

            # Wait before going back to home position
            time.sleep(3)

            logging.info("Returning to home position")
            delay.go_to_home_position()

    except Exception as e:
        logging.error(f"An error occurred: {e}")
        try:
            delay.stop(axis=0)  # Ensure the motor stops if an error occurs
            logging.info("Motor stopped due to error.")
            time.sleep(1)  # Allow some time for the motor to decelerate
        except Exception as stop_error:
            logging.error(f"Failed to stop the motor gracefully: {stop_error}")

    logging.info("rotate_delay procedure completed")

if __name__ == "__main__":
    main()
