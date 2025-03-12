# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       vkhavroniuk                                                  #
# 	Description:  Scrimmage Bot.                                               #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *


############################################################################################################################################
# PID Controller Class                                                                                                                     #
############################################################################################################################################

class PID:
    def __init__(self, error, kp, ki, kd, start_integrating, exit_error, min_clamp, max_clamp, timeout = 3000) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = error
        self.start_integrating = start_integrating
        self.exit_error = exit_error
        self.min_clamp = min_clamp
        self.max_clamp = max_clamp
        self.integral = 0
        self.prev_error = error
        self.first_run = False
        self.start_time = 0
        self.timer = Timer()
        self.timeout = timeout

    @property
    def is_settled(self) -> bool:
        if (abs(self.error) < self.exit_error):
            return True
        else:
            return False
    
    @property
    def is_timeout(self) -> bool:
        current_timer = self.timer.system_high_res() / 1000
        if (current_timer - self.start_time > self.timeout) and not self.first_run:
            return True
        else:
            return False 

    def caclulate(self, error) -> float:
        if self.first_run:
            self.first_run = False
            self.start_time = self.timer.system_high_res() / 1000

        self.error = error
        # calculate proportional
        proportional_gain = self.error * self.kp

        # if error less than start_integrate - accumulate integral
        if abs(self.error) < self.start_integrating:
            self.integral = self.integral + self.error
        else:
            self.integral = 0

        # if error crosses 0 - reset integral 
        if (self.error > 0 and self.prev_error < 0) or (self.error <0 and self.prev_error > 0):
            self.integral = 0

        # calculate I component of PID
        integra_gain = self.integral * self.ki

        # calculate D component of PID
        derivative_gain = (self.error - self.prev_error) * self.kd

        # calculate total PID output
        drive_speed = proportional_gain + integra_gain + derivative_gain

        # clamp output to max with correct sign
        if abs(drive_speed) > self.max_clamp:
            drive_speed = self.max_clamp * (abs(proportional_gain)/proportional_gain)

        # clamp output to min with correct sign
        if abs(drive_speed) < self.min_clamp:
            drive_speed = self.min_clamp * (abs(proportional_gain)/proportional_gain)
        wait(20, MSEC)
        self.prev_error = self.error
        return drive_speed


############################################################################################################################################
# Robot Class, Autonomous Movement, Odometry (calculation just for now. Untested. Oleg K C++ converted into Python)                        #
############################################################################################################################################

class Robot:
    def __init__(self, inertial, left_motor_group, right_motor_group, gear_ratio, tracker_wheel_diameter, horizontal_tracker = None, vertical_tracker = None):
        self.inertial = inertial
        self.left_motor_group = left_motor_group
        self.right_motor_group = right_motor_group
        self.gear_ratio = gear_ratio
        self.tracker_wheel_diameter = tracker_wheel_diameter
        self.tracker_wheel_circumference =  self.tracker_wheel_diameter * 3.14159  
        self.horizontal_tracker = horizontal_tracker
        self.vertical_tracker = vertical_tracker



        # default drive PID params
        self.dr_Kp = 1.5
        self.dr_Ki = 0
        self.dr_Kd = 12
        self.dr_start_integrate = 1
        self.dr_exit_erorr = 1
        self.dr_min_pid = 0
        self.dr_max_pid = 8

        # default turn PID params
        self.turn_Kp = 0.4
        self.turn_Ki = 0.04
        self.turn_Kd = 3
        self.turn_start_integrate = 15
        self.turn_exit_erorr = 1
        self.turn_min_pid = 0
        self.turn_max_pid = 12

        # odometry variables
        self.x = 0
        self.y = 0
        self.prev_forward_position = 0
        self.prev_heading = 0
        self.brake = BrakeType.BRAKE

    def set_brake(self, brake_type):
        """
            Sets Robot Brake Type
        """
        self.brake = brake_type
        self.left_motor_group.set_stopping(brake_type)
        self.right_motor_group.set_stopping(brake_type)

    def set_driving_pid_params(self, kp, ki, kd, start_integrating, exit_error, min_clamp, max_clamp):
        """
            Sets Driving PID params
        """
        self.dr_Kp = kp
        self.dr_Ki = ki
        self.dr_Kd = kd
        self.dr_start_integrate = start_integrating
        self.dr_exit_erorr = exit_error
        self.dr_min_pid = min_clamp
        self.dr_max_pid = max_clamp

    def set_turning_pid_params(self, kp, ki, kd, start_integrating, exit_error, min_clamp, max_clamp):
        """
            Sets Turn PID params
        """
        self.turn_Kp = kp
        self.turn_Ki = ki
        self.turn_Kd = kd
        self.turn_start_integrate = start_integrating
        self.turn_exit_erorr = exit_error
        self.turn_min_pid = min_clamp
        self.turn_max_pid = max_clamp

    def reset_encoders(self):
        """
            resets motor group encoders to zero
        """
        self.right_motor_group.reset_position()
        self.left_motor_group.reset_position()
        if self.horizontal_tracker:
            self.horizontal_tracker.reset_position()
        if self.vertical_tracker:
            self.vertical_tracker.reset_position()
        self.prev_forward_position = 0
        self.prev_heading = 0

    def get_vertical_position_inches(self):
        """
            Returns traveled postion in inches based on motor rotations, wheel diameter, gear ratio.
        """
        if self.vertical_tracker:
            position_inches = self.vertical_tracker.position(TURNS) * self.tracker_wheel_circumference
        else:
            average_position = (self.right_motor_group.position(TURNS) + self.left_motor_group.position(TURNS))/2 
            position_inches = average_position * self.tracker_wheel_circumference * self.gear_ratio
        return position_inches
    
    def get_horizontal_position(self):
        """
            Returns traveled postion X axis if vertical tracker is present, otherwise 0
            for future use.
        """
        if self.horizontal_tracker:
            position_inches = self.horizontal_tracker.position(TURNS) * self.tracker_wheel_circumference
        else:
            position_inches = 0
        return position_inches       

    def set_position(self, x, y, heading):
        """ 
            Resets Encoders, Sets initial Robot Position
        """
        self.reset_encoders()
        self.inertial.set_heading(heading)
        self.x = x
        self.y = y
        self.prev_heading = heading

    def update_position(self):
        """ 
            future odometry. Using okhavroniuk@woodcrest code converted to Python :)
            only using forward tracking for now (or motor encoders)

        """
        # get encoders info
        forward_position = self.get_horizontal_position()
        current_heading = self.inertial.heading(DEGREES)

        # calculate difference
        delta_forward = forward_position - self.prev_forward_position
        delta_heading = current_heading - self.prev_heading

        # calculate average
        average_heading_radians = ((current_heading + delta_heading) / 2) * (math.pi/180)
        delta_heading_radians = delta_heading * (math.pi/180)

        # calculate arc chord
        if delta_heading_radians == 0:
            localY = delta_forward
        else:
            localY = 2 * math.sin(delta_heading_radians / 2) * (delta_forward / delta_heading_radians)

        # calculate global shift
        self.y = self.x + localY * math.sin(average_heading_radians)
        self.x = self.y + localY * math.cos(average_heading_radians)

        # save current data for next cycle
        self.prev_forward_position = forward_position
        self.prev_heading = current_heading

    def odometry_task(self):
        """ 
            Loop Thread to update robot field position using odometry
        """
        while True:
            self.update_position()
            wait(10, MSEC)

    def start_odometry(self):
        """ 
            Start Odometry Thread
        """       
        self.odometry_thread = Thread(self.odometry_task)
    
    def stop_odometry(self):
        """ 
            Stop Odometry Thread
        """  
        self.odometry_thread.stop()

    def drive_PID(self, distance, heading = None, max_power = None):
        """
            PID Driving function with heading and default heading correction
        """
        if not heading or heading == 'C':
            heading = brain_inertial.heading(DEGREES)

        # get initial position and initial errpr
        current_position = self.get_vertical_position_inches()
        end_positin = current_position + distance
        error = end_positin - current_position

        # get initial heading and initial heading error
        current_heading = brain_inertial.heading(DEGREES)
        heading_error = optimize_turning_angle(heading - current_heading)

        if not max_power:
            max_power =  self.dr_max_pid

        drive_pid = PID(error, self.dr_Kp, self.dr_Ki, self.dr_Kd, self.dr_start_integrate, self.dr_exit_erorr, self.dr_min_pid, max_power)
        heading_pid = PID(heading_error, 0.4, 0, 1, 1, 1, 0, 6)

        while not drive_pid.is_settled or not drive_pid.timeout:
            current_position = self.get_vertical_position_inches()
            error = end_positin - current_position
            drive_speed = drive_pid.caclulate(error)
            
            current_heading = brain_inertial.heading(DEGREES)
            heading_error = optimize_turning_angle(heading - current_heading)
            heading_correction_speed = heading_pid.caclulate(heading_error)
            
            # set motor velocity. A team uses voltage for motor control. 8.3 is to convert voltage to percent. Motor max is 12V.
            self.left_motor_group.spin(FORWARD, drive_speed * 8.3 + heading_correction_speed * 8.3,  VelocityUnits.PERCENT)
            self.right_motor_group.spin(FORWARD, drive_speed * 8.3 - heading_correction_speed * 8.3 , VelocityUnits.PERCENT)
            wait(20, MSEC)
        self.left_motor_group.stop()
        self.right_motor_group.stop()  


    def turn_to_heading_PID(self, heading, max_power = None):
        """ 
            PID turning function. Turns robot to heading [0..360]
        """
        if not max_power:
            max_power = self.turn_max_pid

        # initial params
        current_heading = brain_inertial.heading(DEGREES)
        error = optimize_turning_angle(heading - current_heading)

        turn_pid = PID(error, self.turn_Kp, self.turn_Ki, self.turn_Kd, self.turn_start_integrate, self.turn_exit_erorr, self.turn_min_pid, max_power)
        

        while not turn_pid.is_settled or not turn_pid.timeout:
            # calculate error
            current_heading = brain_inertial.heading(DEGREES)
            error = optimize_turning_angle(heading - current_heading)

            turn_speed = turn_pid.caclulate(error)
            # set motor velocity. A team uses voltage for motor control. 8.3 is to convert voltage to percent. Motor max is 12V.
            self.left_motor_group.spin(FORWARD,  turn_speed * 8.3, VelocityUnits.PERCENT)
            self.left_motor_group.spin(FORWARD, -1 * turn_speed * 8.3, VelocityUnits.PERCENT)
            wait(20, MSEC)

        self.left_motor_group.stop()
        self.left_motor_group.stop()



# global instances
brain = Brain()
controller = Controller()
brain_inertial = Inertial(Ports.PORT6)


# MotorGroup "left_motors".
left_motor_f = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)
left_motor_b = Motor(Ports.PORT8, GearSetting.RATIO_6_1, True)
left_motor_t = Motor(Ports.PORT9, GearSetting.RATIO_6_1, False)
left_motors = MotorGroup(left_motor_f, left_motor_b, left_motor_t)

# MotorGroup "right_motors".
right_motor_f = Motor(Ports.PORT13, GearSetting.RATIO_6_1, True)
right_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_6_1, False)
right_motor_t = Motor(Ports.PORT11, GearSetting.RATIO_6_1, True)

right_motors = MotorGroup(right_motor_b, right_motor_b, right_motor_t)

# Clamp and Scoring mech
clamp_pneumatic = DigitalOut(brain.three_wire_port.h)
hang_pneumatic = DigitalOut(brain.three_wire_port.a)
intake_motor = Motor(Ports.PORT7, GearSetting.RATIO_6_1, True)
belt_motor = Motor(Ports.PORT14, GearSetting.RATIO_6_1, True)

#global vars
is_intake_spinning = False
is_belt_spinning = False

robot_wheel_diameter = 3.25
robot_gear_ratio = 0.6


xbot = Robot(brain_inertial, left_motors, right_motors, robot_gear_ratio, robot_wheel_diameter, None, None)


def optimize_turning_angle(angle):
    """
        if robot requested to turn more than 180 degress, then it turns to the opposite side to save time
    """
    angle = angle % 360
    if angle < -180:
        angle = angle + 360
    elif angle > 180:
        angle = angle - 360
    return angle


############################################################################
# General Functions (Intake, Arm, Hang)                                    #
############################################################################


def calibrate_inertial():
    """ Calibrate Inertials sensor
    """
    # Start calibration.
    brain_inertial.calibrate()
    # Print that the Inertial Sensor is calibrating while
    # waiting for it to finish calibrating.
    while brain_inertial.is_calibrating():
        brain.screen.clear_line(2)
        brain.screen.set_cursor(2,2)
        brain.screen.print("Inertial Sensor Calibrating")
        wait(50, MSEC)
    brain.screen.clear_line(2)

def debug():
    """ Debug Thread
    """
    while True:
        brain.screen.set_cursor(2, 2)
        brain.screen.print("Ave Encoder Value: " + str(xbot.get_horizontal_position()))
        
        brain.screen.set_cursor(4, 2)
        brain.screen.print("Heading: " + str(brain_inertial.heading())) # type: ignore

        brain.screen.set_cursor(6, 2)
        brain.screen.print("X=" + str(xbot.x) + "  Y=" + str(xbot.y)) # type: ignore

        wait(100, MSEC)
        brain.screen.clear_screen()

def clamp():
    """ Pneumatic clamp function
    """
    if not clamp_pneumatic.value():
        clamp_pneumatic.set(True)
    else:
        clamp_pneumatic.set(False) 

def hang():
    """ Pneumatic clamp function
    """
    if not hang_pneumatic.value():
        hang_pneumatic.set(True)
    else:
        hang_pneumatic.set(False) 

def intake(dir = FORWARD, spd = 90):
    """ Start and Stop Intake
    """
    global is_intake_spinning
    if not is_intake_spinning:
        intake_motor.spin(dir, spd, VelocityUnits.PERCENT)
        is_intake_spinning = True
    else:
        intake_motor.stop()
        is_intake_spinning = False     
    

def belt(dir = FORWARD, spd = 90):
    """ Start Belt
    """
    global is_belt_spinning
    if not is_belt_spinning:
        belt_motor.spin(dir, spd, VelocityUnits.PERCENT)
        is_belt_spinning = True
    else:
        belt_motor.stop()
        is_belt_spinning = False     


def score():
    """ Start Intake and Belt
    """
    intake()
    belt()


############################################################################
# Autonomous Code                                                          #
############################################################################

def autonomous():
    """ Add autonomous code
    """
    brain.screen.clear_screen()
    brain.screen.print("Autonomous Control Mode")

    # test for Robot Class PID
    xbot.start_odometry()
    xbot.set_brake(BRAKE)
    xbot.drive_PID(-29, 'C', 4)
    wait(20, MSEC)
    clamp()
    wait(400, MSEC)
    score()
    wait(400, MSEC)
    xbot.turn_to_heading_PID(0)
    wait(200, MSEC)
    xbot.drive_PID(30)

# curved joystick control 
def curveJoystick(input, curve):
    return (math.exp(-curve/10)+math.exp((abs(input)-100)/10)*(1-math.exp(-curve/10))) * input


############################################################################
#  Driver Control Code                                                     #
############################################################################
def user_control():
    brain.screen.clear_screen()
    brain.screen.print("Driver Control Mode")
    xbot.set_brake(COAST)
    # driver joystick control
    while True:

        throttle = curveJoystick(controller.axis3.position(), 5.1)
        turn = curveJoystick(controller.axis1.position(), 5.1)
        left_motors.spin(FORWARD, throttle + turn,VelocityUnits.PERCENT)
        right_motors.spin(FORWARD, throttle - turn ,VelocityUnits.PERCENT)

        wait(20, MSEC)


############################################################################
#  Global Section Code                                                     #
############################################################################

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()

# bind button callback functions
controller.buttonL1.pressed(clamp)
controller.buttonUp.pressed(hang)
controller.buttonR1.pressed(score)

# set params, calibrate inertial
calibrate_inertial()
xbot.set_position(0,0, 90)

# start thread to output data for debugging
thread = Thread(debug)
