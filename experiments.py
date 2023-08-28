from hub import light_matrix
from hub import button
from hub import light
from hub import port
from hub import sound
from hub import motion_sensor

import motor_pair
import motor
import runloop
import math
import color_sensor

LEFT_WHEEL_PORT = port.A
RIGHT_WHEEL_PORT = port.E
LEFT_LIGHT_SENSOR = port.F
RIGHT_LIGHT_SENSOR = port.F
FRONT_MOTOR = port.D
REAR_MOTOR = port.C
ACCELERATION = 1000
STOP_BEHAVIOR = motor.SMART_BRAKE
ROTATION_DISTANCE = 27.4
ARM_SPEED = 100

# speed is in degrees of rotation per second 
MAX_SPEED = 1000

def yaw_angle():
    return motion_sensor.tilt_angles()[0] / 10

async def reset_yaw_angle():
    motion_sensor.reset_yaw(0)
    # wait for yaw to actually reset
    while motion_sensor.tilt_angles()[0] != 0:
        runloop.sleep_ms(10)


class Motion:
    def __init__(self, left_motor_port, right_motor_port, rotation_distance, left_sensor_port, right_sensor_port):
        self.pair = motor_pair.PAIR_1
        self.left_motor_port = left_motor_port
        self.right_motor_port = right_motor_port
        motor_pair.pair(self.pair, left_motor_port, right_motor_port)
        self.rotation_distance = rotation_distance
        self.default_stop_behavior = STOP_BEHAVIOR
        self.acceleration = ACCELERATION
        self.left_sensor_port = left_sensor_port
        self.right_sensor_port = right_sensor_port
        self.left_start_value = 0
        self.right_start_value = 0

        self.pRegler = 0.0
        self.iRegler = 0.0
        self.dRegler = 0.0

        self.pReglerLight = 0.0
        self.iReglerLight = 0.0
        self.dReglerLight = 0.0

        runloop.sleep_ms(2000)

    def pid_calculation_light(self, speed):
        #Sets the PID values for the lineFollower based on current speed. Allows for accurate and fast driving
        #Important note: these PID values are experimental and based on our design for the robot. You will need to adjust them. See above on how to do so

        #self.pReglerLight = -0.04 * speed + 4.11
        #self.dReglerLight = 0.98 * speed - 34.2
        self.dReglerLight = 5
        self.pReglerLight = 0.3
        #set hard bottom for d value, as otherwise the values don't work
        if self.dReglerLight < 5:
            self.dReglerLight = 5
        

    def get_driven_distance(self):
        driven_distance = (
                    abs(motor.relative_position(self.left_motor_port) - self.left_start_value) +
                    abs(motor.relative_position(self.right_motor_port) - self.right_start_value)) / 2
        return driven_distance

    def speed_calculation(self, speed, start_speed, max_speed, end_speed, accelerate_distance, deccelerate_distance, brake_start_value, driven_distance, old_driven_distance):
        """
            Used to calculate all the speeds in out programs. Done seperatly to reduce redundancy. Brakes and accelerates
            Parameters
            -------------
            speed: The current speed the robot has
            start_speed: Speed the robot starts at. Type: Integer. Default: No default value.
            max_speed: The maximum speed the robot reaches. Type: Integer. Default: No default value.
            end_speed: Speed the robot aims for while braking, minimum speed at the end of the program. Type: Integer. Default: No default value.
            add_speed: Percentage of the distance after which the robot reaches the maximum speed. Type: Integer. Default: No default value.
            brake_start_value: Percentage of the driven distance after which the robot starts braking. Type: Integer. Default: No default value.
            driven_distance: Calculation of the driven distance in degrees. Type: Integer. Default: No default value.
        """
        if end_speed == 0:
            end_speed = 1
        add_speed_per_degree = (max_speed - start_speed) / accelerate_distance
        sub_speed_per_degree = (max_speed - end_speed) / deccelerate_distance
        subtraction =  max(abs(driven_distance) - abs(old_driven_distance), 1) * sub_speed_per_degree
        addition = max(abs(driven_distance) - abs(old_driven_distance), 1) * add_speed_per_degree

        if abs(driven_distance) > abs(brake_start_value):
            if abs(speed) > abs(end_speed) and speed * end_speed >= 0:
                speed = speed - subtraction
        elif abs(speed) < abs(max_speed):
            speed = speed + addition
        return speed

    def simple_follow_line(self, distance):
        self.follow_line(distance, 100, 100, 100, self.left_sensor_port, "left")

    def follow_line(self, distance, start_speed, max_speed, end_speed, sensor_port, side, add_speed=0.2, brake_start=0.7, stop_method=None, stop=True):
        """
            This is the function used to let the robot follow a line until either the entered distance has been achieved or the other sensor of the robot senses a line.
            Like all functions that drive the robot this function has linear acceleration and breaking. It also uses PID values that are automatically set depending on the
            current speed of the robot (See function PIDCalculationLight)
            Parameters
            -------------
            distance: The value tells the program the distance the robot has to drive. Type: Integer. Default: No default value
            startspeed: The speed which the robot is supposed to start at. Type: Integer. Default: No default value
            maxspeed: The highest speed at which the robot drives. Type: Integer. Default: No default value
            endspeed: The speed which the robot achieves at the end of the function. Type: Integer. Default: No default value
            side: the side of the line to drive on, "left" or "right"
            addspeed: The percentage after which the robot reaches its maxspeed. Type: Float. Default: No default value
            brakeStart: The value which we use to tell the robot after what percentage of the distance we need to slow down. Type: Float. Default: No default value
            stopMethod: the Stopmethod the robot uses to stop. If no stopMethod is passed stopDistance is used instead. Default: stopDistance
            stop: the boolean that determines whether the robot should stop the motors after driving or not. Default: True
        """
        speed = start_speed
        # reset PID values
        error = 0
        old_error = 0
        integral = 0
        
        loop=True
        final_distance = (distance / self.rotation_distance) * 360
        accelerate_distance = final_distance * add_speed
        deccelerate_distance = final_distance * ( 1 - brake_start)

        invert = 1
        if side == "right":
            invert == -1

        self.left_start_value = motor.relative_position(self.left_motor_port)
        self.right_start_value = motor.relative_position(self.right_motor_port)
        driven_distance = self.get_driven_distance()

        brake_start_value = brake_start * final_distance
        loopCount = 0
        while loop:
            #Checks the driven distance as an average of both motors for increased accuracy
            old_driven_distance = driven_distance
            driven_distance = self.get_driven_distance()
            if final_distance < driven_distance:
                break

            #Calculates target value for Robot as the edge of black and white lines
            old_error = error
            error = color_sensor.reflection(sensor_port) - 50
            integral += error

            p_fix = error * 0.3
            i_fix = 0
            d_fix = 0
            #Steering factor calculation using PID, sets new I value
            #steering = (((error * self.pReglerLight) + (integral * self.iReglerLight) + (self.dReglerLight * (error - old_error)))) * invert
            steering = p_fix + i_fix + d_fix
            speed = self.speed_calculation(speed, start_speed, max_speed, end_speed, accelerate_distance, deccelerate_distance, brake_start_value, driven_distance, old_driven_distance)

            # calculate PID
            self.pid_calculation_light(speed)

            steering = max(-100, min(steering, 100))


            if loopCount % 100 == 0:
                print("%d. driven_distance %.1f change %.1f steering %.0f speed %.1f" % (loopCount, driven_distance * self.rotation_distance/360, error, steering, speed))

            motor_pair.move(self.pair, int(steering), velocity=math.ceil(speed))

            
            
            loopCount += 1

        if stop:
            motor_pair.stop(self.pair)

        return
        


        

    # steering is -100 to 100
    def move_for_degrees(self, degrees, steering, velocity = 360, acceleration = 1000, deceleration = 1000):
        motor_pair.move_for_degrees(self.pair, degrees, steering, velocity=velocity, stop=self.default_stop_behavior, acceleration=self.acceleration, deceleration=deceleration)

    # Robot moves ahead "distance" centimeters
    async def move_distance(self, distance, velocity = 360, acceleration = ACCELERATION, deceleration = ACCELERATION):
        degrees = (int) (distance / self.rotation_distance * 360)
        await motor_pair.move_for_degrees(self.pair, degrees, 0, stop=self.default_stop_behavior, velocity=velocity, acceleration=acceleration, deceleration=deceleration)

    # Robot turns in place "degrees" 
    async def turn_relative(self, degrees):
            degrees = -degrees # make positive rotation clockwise
            max_pct = 30
            feedforward = 5
            p_gain = 0.15
            stop_error = 0.3
            await reset_yaw_angle()
            error = degrees
            velocity = 0
            wheel_pct = 0
            while stop_error < abs(error):
                wheel_pct = int(min(max_pct, feedforward + abs(error * p_gain)))
                direction = 1 if error > 0 else -1
                velocity = -direction * (wheel_pct * MAX_SPEED)//100
                motor_pair.move(self.pair, 100, velocity=velocity, acceleration=self.acceleration)
                error = degrees - yaw_angle()
            motor_pair.stop(self.pair, stop=self.default_stop_behavior)
            
            print("FINAL wheel_pct:%d        error:%f    yaw_angle:%f    velocity:%f" % (wheel_pct, error, yaw_angle(), velocity))



def blink():
    runloop.sleep_ms(3000)
    light_matrix.show_image(light_matrix.IMAGE_HAPPY)
    runloop.sleep_ms(1000)
    light_matrix.show_image(light_matrix.IMAGE_SMILE)
    runloop.sleep_ms(1000)
    light_matrix.show_image(light_matrix.IMAGE_ANGRY)

class Arm:
    def __init__(self, port):
        self.port = port

    async def move_arm_position(self, position, speed_percentage=50):
        speed = speed_percentage * ARM_SPEED
        await motor.run_to_relative_position(self.port, position, speed)
    


def configure():
    global MOTION
    global FRONT_ARM
    global REAR_ARM
    MOTION = Motion(LEFT_WHEEL_PORT, RIGHT_WHEEL_PORT, 27.4, LEFT_LIGHT_SENSOR, RIGHT_LIGHT_SENSOR)
    FRONT_ARM = Arm(FRONT_MOTOR)
    REAR_ARM = Arm(REAR_MOTOR)


async def mission_one():
    await FRONT_ARM.move_arm_position(100)
    await MOTION.move_distance(25)
    await MOTION.turn_relative(-90)
    await MOTION.move_distance(33)
    await MOTION.turn_relative(-15)
    await FRONT_ARM.move_arm_position(-80)
    await FRONT_ARM.move_arm_position(100)

def test_color_sensors():
    print("Left sensor reflection %d )" % color_sensor.reflection(LEFT_LIGHT_SENSOR))
    print("Right sensor reflection %d )" % color_sensor.reflection(RIGHT_LIGHT_SENSOR))
    print("Left sensor color %d )" % color_sensor.color(LEFT_LIGHT_SENSOR))
    print("Right sensor color %d )" % color_sensor.color(RIGHT_LIGHT_SENSOR))

# This is the main function. It is like the "when program starts" block in scratch
async def main():
    print('main loop') # This text is printed to the terminal

    # write your code here
    # The await keyboard makes the app wait until the line is done before moving to the next line
    await light_matrix.write("loser")
    configure()
    
    test_color_sensors()
    #MOTION.simple_follow_line(50)
    #await MOTION.move_distance(30)
    #await MOTION.turn_relative(90)
    #await FRONT_ARM.move_arm_position(-100)

    #mission_one()
 




    print('all done\n\n\n')

    





# This is the line that calls the main function. It has to be at the end of the file.
runloop.run(main())
