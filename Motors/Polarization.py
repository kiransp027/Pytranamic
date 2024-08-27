from pytrinamic.features.linear_ramp import LinearRamp
from pytrinamic.features.drive_setting import DriveSetting 
from Motors.trinamic_controller import TMCM3212 
import time
from pytrinamic.tmcl import TMCLCommand

class PolarizationPaddler(TMCM3212):

    def __init__(self,connection,module_id=1,ap_index_bit_width=8,step_angle=0.9):
        super().__init__(connection,module_id,ap_index_bit_width)
        self.connection=connection
        self.ap_index_bit_width=ap_index_bit_width
        self.module_id=module_id
        self.module=TMCM3212(self.connection)
        self.motor=self.module.motors[1]

        #Calculate the steps in each revolution
        self.steps_rev=int((360/step_angle)*self.motor.MR.microstep_resolution_256_microsteps)
        
        #Assigning the minimum and maximum positions of the motor
        self.min_position=(-180*self.steps_rev)/360
        self.max_position=(180*self.steps_rev)/360

        # Set motor parameters
        print('Setting motor parameters')
        self.motor.drive_settings.max_current=40 # Set to 0.40A
        self.motor.drive_settings.standby_current=0  # Typically half of max current
        self.motor.drive_settings.boost_current=0  # Set if additional current is needed
        self.motor.drive_settings.microstep_resolution = self.motor.ENUM.microstep_resolution_256_microsteps
        print(self.motor.drive_settings)

        self.motor.actual_position=0
        
        #Setting the velocity and acceleration of the axis
        self.motor.set_axis_parameter(self.motor.AP.MaxAcceleration,30000)
        self.motor.set_axis_parameter(self.motor.AP.MaxVelocity,30000)
        print(self.motor.linear_ramp)

    def rotate(self, axis,velocity):
        self.connection.rotate(axis, velocity, self.module_id)

    def move_to(self, axis, position, velocity=None):
        steps=int((position*self.steps_rev)/360)
        if steps < self.min_position:
            steps = self.min_position
            print("Minimum position reached, can't move any further.")
        elif steps > self.max_position:
            steps = self.max_position
            print("Maximum position reached, can't move any further.")
    
        if velocity:
            self.motors[axis].linear_ramp.max_velocity = velocity
        self.connection.move_to(axis, steps, self.module_id)

    def move_by(self, axis, difference, velocity=None):
        target_position = self.get_position() + difference
        if velocity:
            self.motor.linear_ramp.max_velocity = velocity
        self.move_to(axis, target_position, velocity)

    def stop(self, axis):
        self.connection.stop(axis, self.module_id)

    def get_position(self):
        return self.motor.get_axis_parameter(self.motor.AP.ActualPosition)
    
    def is_position_reached(self):
        return self.motor.get_axis_parameter(self.motor.AP.PositionReachedFlag)
    
    def is_home_position(self,home_status):
        return self.motor.get_axis_parameter(self.motor.AP.HomeSwitch)
    
    # def go_to_home_position(self):
    #     time_out=10
    #     start_time=time.time()
    #     print('Starting homing procedure')
    #     # Set reference speedspeed
    #     self.motor.set_axis_parameter(self.motor.AP.ReferenceSearchSpeed, 1000)
    #     self.motor.set_axis_parameter(self.motor.AP.RefSwitchSpeed, 500)

    #     # Check home sensor state
    #     #self.motor.set_axis_parameter(self.motor.AP.HomeSwitch, value=0)
    #     initial_home_sensor_state = self.motor.get_axis_parameter(self.motor.AP.HomeSwitch)
    #     print(f"Home sensor state: {initial_home_sensor_state}")

    #     # if initial_home_sensor_state:
    #     #     print("Already at home position, moving away slightly...")
    #     #     self.motor.move_by(-1000)  # Move by a small amount away from the home position
    #     #     time.sleep(2)  # Wait for the move to complete
    #     #     initial_home_sensor_state = self.motor.get_axis_parameter(self.motor.AP.HomeSwitch)
    #     #     print(f"Home sensor state after moving away: {initial_home_sensor_state}")

    #     # Set reference search mode based on sensor state
    #     if initial_home_sensor_state==0:
    #         self.motor.set_axis_parameter(self.motor.AP.ReferenceSearchMode, 8)
    #     else:
    #         self.motor.set_axis_parameter(self.motor.AP.ReferenceSearchMode, 132)

    #     # Start reference search
    #     self.motor.reference_search(command_type=0,motor=1)

    #     # Loop to monitor the homing process
    #     # while not self.is_home_position():
    #     #     print(f'Approaching home position: {self.motor.actual_position}')
    #     #     time.sleep(0.2)
    #     while not self.motor.reference_search(command_type=2, motor = 1):
    #         home_sensor_state=self.motor.get_axis_parameter(self.motor.AP.HomeSwitch)
    #         print(self.motor.get_axis_parameter(self.motor.AP.HomeSwitch))
    #         if home_sensor_state==0:
    #             print("Home sensor detected. Stopping the motor.")
    #             self.connection.reference_search(command_type=1, motor=1)  # Stop the homing process
    #             break
    #         # Check if timeout is exceeded
    #         if time.time() - start_time > time_out:
    #             print("Homing procedure timed out. Stopping the motor.")
    #             self.connection.reference_search(command_type=1,motor=1)
    #             break

    #         current_position = self.motor.get_axis_parameter(self.motor.AP.ActualPosition)
    #         current_velocity = self.motor.get_axis_parameter(self.motor.AP.ActualVelocity)
    #         print(f"Current position: {current_position}, velocity: {current_velocity}, home sensor state: {home_sensor_state}")

    #         #Small sleep to prevent excessive CPU usage
    #         time.sleep(0.2)

    #     # Set the motor's actual position to 0 after successful homing
    #     self.motor.actual_position = 0
    #     print("Homing procedure completed successfully.")

    def go_to_home_position(self):
        time_out=10
        start_time=time.time()
        print('Starting homing procedure')
        # Set reference speed
        self.motor.set_axis_parameter(self.motor.AP.ReferenceSearchSpeed, 5000)
        self.motor.set_axis_parameter(self.motor.AP.RefSwitchSpeed, 500)

        self.motor.set_axis_parameter(self.motor.AP.ReferenceSearchMode, 8)

        initial_status=self.connection.reference_search(command_type=2,motor=1)
        print(f'Reference search status initially: {initial_status}')
        #Start reference search
        self.connection.reference_search(command_type=0,motor=1)
        print('Starting reference search')
        while True:
            status=self.connection.reference_search(command_type=2,motor=1)
            if status == 0:
                print('Reference search completed')
                self.connection.reference_search(command_type=1,motor=1)
                break
            time.sleep(0.2)
        # if TMCLCommand.WAIT:
        #     while self.connection.reference_search(command_type=27,motor=1):
        #         print(f'Reference status: {self.connection.reference_search(command_type=2,motor=1)}')
        #         time.sleep(0.2)

            if time.time() - start_time > time_out:
                print("Homing procedure timed out. Stopping the motor.")
                self.motor.stop()

        self.motor.set_axis_parameter(self.motor.AP.ActualPosition, 0)





