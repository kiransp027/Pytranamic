import sys
import os
import time
import logging
# Add the directory where Motors is located to the system path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from pytrinamic.connections import ConnectionManager
from Motors.trinamic_controller import TMCM3212
#from pytrinamic.features import LinearRamp, StallGuard2Module, CoolStepModule


# Setup logging configuration to write to delaylinelog.txt
logging.basicConfig(filename="delaylinelog.txt", 
                    level=logging.INFO, 
                    format="%(asctime)s - %(message)s")

class DelayLine(TMCM3212):
    def __init__(self, connection, module_id=1, ap_index_bitwidth=8, step_angle=1.8, lead_pitch=5.08):
        super().__init__(connection, module_id, ap_index_bitwidth)
        self.connection = connection
        self.ap_index_bit_width = ap_index_bitwidth
        self.module_id = module_id
        self.module = TMCM3212(self.connection)
        self.motor = self.module.motors[0]

        # Calculate the steps in each revolution
        self.steps_rev = int((360 / step_angle) * self.motor.MR.microstep_resolution_256_microsteps)
        self.lead_pitch = lead_pitch

        # Set motor min and max positions
        self.min_position = 0
        self.max_position = int((10 * self.steps_rev) / self.lead_pitch)

        # Set home position of the delay line
        self.home_position = 1

        # Set motor parameters
        logging.info('Setting motor parameters')
        self.motor.drive_settings.max_current = 8  # Set to 0.40A
        self.motor.drive_settings.standby_current = 0  # Typically half of max current
        self.motor.drive_settings.boost_current = 0  # Set if additional current is needed
        self.motor.drive_settings.microstep_resolution = self.motor.ENUM.microstep_resolution_256_microsteps
        logging.info(f'Drive Settings: {self.motor.drive_settings}')

        self.motor.set_axis_parameter(self.motor.AP.RunCurrent, 8)
        logging.info(f'Run current: {self.motor.get_axis_parameter(self.motor.AP.RunCurrent)}')

        # Set initial motor position
        self.motor.actual_position = 0

        # Set velocity and acceleration
        self.motor.set_axis_parameter(self.motor.AP.MaxAcceleration, 30000)
        self.motor.set_axis_parameter(self.motor.AP.MaxVelocity, 30000)
        logging.info(f'Linear Ramp: {self.motor.linear_ramp}')

        logging.info(f'StallGuard2: {self.motor.stallguard2}')
        logging.info(f'Load Value: {self.motor.get_axis_parameter(self.motor.AP.LoadValue)}')

        # Set the freewheeling mode
        self.motor.set_axis_parameter(self.motor.AP.FreewheelingMode, value=3)

        # Set PWM parameters
        self.motor.set_axis_parameter(self.motor.AP.PWMGrad, value=1)
        self.motor.set_axis_parameter(self.motor.AP.PWMAmplitude, value=64)
        self.motor.set_axis_parameter(self.motor.AP.PWMAutoscale, value=1)
        self.motor.set_axis_parameter(self.motor.AP.PWMThresholdSpeed, value=51200)
        logging.info(f'PWM Mode: {self.motor.get_axis_parameter(self.motor.AP.PWMMode)}')
        logging.info(f'PWM Frequency: {self.motor.get_axis_parameter(self.motor.AP.PWMFrequency)}')
        logging.info(f'PWM Threshold Speed: {self.motor.get_axis_parameter(self.motor.AP.PWMThresholdSpeed)}')

    def rotate(self, axis, velocity):
        logging.info(f'Rotating axis {axis} with velocity {velocity}')
        self.connection.rotate(axis, velocity, self.module_id)

    def move_to(self, axis, position, velocity=None):
        steps = int((position * self.steps_rev) / self.lead_pitch)
        if steps < self.min_position:
            steps = self.min_position
            logging.warning("Minimum position reached, can't move any further.")
        elif steps > self.max_position:
            steps = self.max_position
            logging.warning("Maximum position reached, can't move any further.")
    
        if velocity:
            self.motors[axis].linear_ramp.max_velocity = velocity
        logging.info(f'Moving to position {position} with steps {steps}')
        self.connection.move_to(axis, steps, self.module_id)

    def move_by(self, axis, difference, velocity=None):
        target_position = self.get_position() + difference
        if velocity:
            self.motor.linear_ramp.max_velocity = velocity
        logging.info(f'Moving by difference {difference}, target position {target_position}')
        self.move_to(axis, target_position, self.module_id)

    def stop(self, axis):
        logging.info(f'Stopping axis {axis}')
        self.connection.stop(axis, self.module_id)
    
    def get_position(self):
        position = self.motor.get_axis_parameter(self.motor.AP.ActualPosition)
        logging.info(f'Current motor position: {position}')
        return position
       
    def is_position_reached(self):
        position_reached = self.motor.get_axis_parameter(self.motor.AP.PositionReachedFlag)
        logging.info(f'Position reached flag: {position_reached}')
        return position_reached
    
    def go_to_home_position(self):
        time_out = 10
        start_time = time.time()
        logging.info('Starting homing procedure')

        # Set reference speed
        self.motor.set_axis_parameter(self.motor.AP.ReferenceSearchMode, 7)
        self.motor.set_axis_parameter(self.motor.AP.ReferenceSearchSpeed, 10000)
        self.motor.set_axis_parameter(self.motor.AP.RefSwitchSpeed, 500)

        home_state = self.motor.get_axis_parameter(self.motor.AP.HomeSwitch)
        logging.info(f'Initial home state: {home_state}')

        try:
            initial_status = self.get_reference_search_status(motor=0)
            logging.info(f'Reference search status initially: {initial_status}')
            
            # Start reference search
            self.start_reference_search(motor=0, mode=7)
            logging.info('Starting reference search')

            while True:
                current_home_state = self.motor.get_axis_parameter(self.motor.AP.HomeSwitch)
                logging.info(f'Current home state: {current_home_state}')

                # Check if home state is 0 (meaning home is reached)
                if current_home_state == 0:
                    logging.info('Home position reached')
                    self.motor.set_axis_parameter(self.motor.AP.ActualPosition, 0)
                    logging.info('Motor actual position set to 0')
                    break

                if time.time() - start_time > time_out:
                    logging.warning('Homing procedure timed out')
                    self.stop(0)
                    break

                time.sleep(0.2)

            logging.info('Homing procedure completed')

        except Exception as e:
            logging.error(f"An error occurred during homing: {e}")
            try:
                self.stop_reference_search(motor=0)  # Stop reference search if an error occurs
                logging.info("Stopped reference search.")
            except Exception as stop_error:
                logging.error(f"Failed to stop the motor gracefully: {stop_error}")
