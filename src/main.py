# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       vkhavroniuk                                                  #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

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

# smartdrive
# smartdrive = SmartDrive(left_motors, right_motors, brain_inertial, wheelTravel, trackWidth, wheelBase, INCHES, externalGearRatio)
smartdrive = SmartDrive(left_motors, right_motors, brain_inertial, 156, 340, 230, MM, 0.6)


# Clamp and Scoring mech
clamp_pneumatic = DigitalOut(brain.three_wire_port.h)
intake_motor = Motor(Ports.PORT7, GearSetting.RATIO_6_1, True)
belt_motor = Motor(Ports.PORT14, GearSetting.RATIO_6_1, True)

#global vars
is_intake_spinning = False
is_belt_spinning = False

def reset_motor_encoders():
    """
        resets motor group encoders to zero
    """
    right_motors.reset_position()
    left_motors.reset_position()

def get_position_inches():
    """
        Returns traveled postion in inches based on motor rotations, wheel diameter, gear ratio. 
    """
    wheel_diameter = 3.25
    gear_ratio = 0.6
    average_position = (right_motors.position(TURNS) + left_motors.position(TURNS))/2 
    wheel_circumference =  wheel_diameter * 3.14159  
    average_position_inches = average_position * wheel_circumference * gear_ratio
    return average_position_inches

def drive_PID(distance, max_power=8):
    """
        Simple PID driving function. No heading correction yet
    """
    # set drive PID params
    dr_Kp = 1.5
    dr_Ki = 0
    dr_Kd = 8
    dr_start_integrate = 1
    dr_exit_erorr = 1
    integral = 0
    
    # reset encoders to zero
    reset_motor_encoders()

    # initial params
    error = distance - get_position_inches()
    prev_error = error

    while abs(error) > dr_exit_erorr:
        # calculate error
        error = distance - get_position_inches()
        
        # calculate proportional
        proportional_gain = error * dr_Kp

        # if error less than start_integrate - accumulate integral
        if abs(error) < dr_start_integrate:
            integral = integral + error
        else:
            integral = 0

        # if error crosses 0 - reset integral 
        if (error > 0 and prev_error < 0) or (error <0 and prev_error > 0):
            integral = 0

        # calculate I component of PID
        integra_gain = integral * dr_Ki

        # calculate D component of PID
        derivative_gain = (error - prev_error) * dr_Kd

        # calculate total PID output
        drive_speed = proportional_gain + integra_gain + derivative_gain

        # clamp output to max with correct sign
        if abs(drive_speed) > max_power:
             drive_speed = max_power * (abs(proportional_gain)/proportional_gain)

        # set motor velocity. A team uses voltage for motor control. 8.3 is to convert voltage to percent. Motor max is 12V.
        left_motors.spin(FORWARD, drive_speed * 8.3, VelocityUnits.PERCENT)
        right_motors.spin(FORWARD, drive_speed * 8.3, VelocityUnits.PERCENT)
        wait(20, MSEC)
        prev_error = error
        
    left_motors.stop(BRAKE)
    right_motors.stop(BRAKE)


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


def turn_to_heading_PID(heading, max_power = 12):
    """ 
        Simple PID turning function. Turns robot to specified heading [0..360]
    """
    # set turn PID params
    turn_Kp = 0.4
    turn_Ki = 0.04
    turn_Kd = 3
    turn_start_integrate = 15
    turn_exit_erorr = 1
    integral = 0
    
   # initial params
    current_heading = brain_inertial.heading(DEGREES)
    error = optimize_turning_angle(heading - current_heading)
    prev_error = error

    while abs(error) > turn_exit_erorr:
        # calculate error
        current_heading = brain_inertial.heading(DEGREES)
        error = optimize_turning_angle(heading - current_heading)

        # calculate proportional
        proportional_gain = error * turn_Kp

        # if error less than start_integrate - accumulate integral
        if abs(error) < turn_start_integrate:
            integral = integral + error
        else:
            integral = 0

        # if error crosses 0 - reset integral 
        if (error > 0 and prev_error < 0) or (error <0 and prev_error > 0):
            integral = 0

        # calculate I component of PID
        integra_gain = integral * turn_Ki

        # calculate D component of PID
        derivative_gain = (error - prev_error) * turn_Kd

        # calculate total PID output
        turn_speed = proportional_gain + integra_gain + derivative_gain

        # clamp output to max with correct sign
        if abs(turn_speed) > max_power:
             turn_speed = max_power * (abs(proportional_gain)/proportional_gain)

        # set motor velocity. A team uses voltage for motor control. 8.3 is to convert voltage to percent. Motor max is 12V.
        left_motors.spin(FORWARD,  turn_speed * 8.3, VelocityUnits.PERCENT)
        right_motors.spin(FORWARD, -1 * turn_speed * 8.3, VelocityUnits.PERCENT)

        prev_error = error
        wait(20, MSEC)

    left_motors.stop(BRAKE)
    right_motors.stop(BRAKE)



def set_initial_params():
    """ Set initial params
    """
    brain_inertial.set_heading(90)
    # P - Gain. Smartdrive uses simple P controller
    # set Kp for P controller
    smartdrive.set_turn_constant(0.5)
    # set P controller exit error marging
    smartdrive.set_turn_threshold(2)


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


def show_me_info():
    """ Debug Thread
    """
    while True:
        brain.screen.set_cursor(2, 2)
        brain.screen.print("Average Position: " + str(get_position_inches()))
        
        brain.screen.set_cursor(4, 2)
        brain.screen.print("Heading: " + str(brain_inertial.heading())) # type: ignore

        wait(100, MSEC)
        brain.screen.clear_screen()

def clamp():
    """ Pneumatic clamp function
    """
    if not clamp_pneumatic.value():
        clamp_pneumatic.set(True)
    else:
        clamp_pneumatic.set(False) 


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

def autonomous():
    """ Add autonomous code
    """
    # test for adjusted embedded P controller Kp
    smartdrive.drive_for(25, INCHES)
    wait(0.2, SECONDS)
    smartdrive.turn_to_heading(180)

    # test for external PID
    drive_PID(25)
    wait(1, SECONDS)
    drive_PID(-25)

    wait(1, SECONDS)
    turn_to_heading_PID(180)
    wait(1, SECONDS)
    turn_to_heading_PID(270)
    wait(1, SECONDS)
    turn_to_heading_PID(0)


def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")
    # driver joystick control
    while True:

        throttle = controller.axis3.position()
        turn = controller.axis1.position()
        left_motors.spin(FORWARD, throttle + turn,VelocityUnits.PERCENT)
        right_motors.spin(FORWARD, throttle - turn ,VelocityUnits.PERCENT)

        wait(20, MSEC)

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()

# bind button callback functions
controller.buttonL1.pressed(clamp)
controller.buttonR1.pressed(score)

# set params, calibrate inertial
calibrate_inertial()
set_initial_params()

# start thread to output data for debug
thread = Thread(show_me_info)
