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

LEFT_WHEEL_PORT = port.A
RIGHT_WHEEL_PORT = port.E
LEFT_LIGHT_SENSOR_LEFT = port.B
RIGHT_LIGHT_SENSOR = port.F
FRONT_MOTOR = port.D
REAR_MOTOR = port.C
ACCELERATION = 1000
STOP_BEHAVIOR = motor.SMART_BRAKE
ROTATION_DISTANCE = 27.4

# speed is in degrees of rotation per second 
MAX_SPEED = 1000

# returns absolute value of num
def abs(num):
    if num > 0:
        return num
    return -num

def min(a, b):
    if a < b:
        return a
    return b

def yaw_angle():
    return motion_sensor.tilt_angles()[0] / 10

async def reset_yaw_angle():
    motion_sensor.reset_yaw(0)
    # wait for yaw to actually reset
    while motion_sensor.tilt_angles()[0] != 0:
        runloop.sleep_ms(10)
        
class Motion:
    def __init__(self, left_motor_port, right_motor_port, rotation_distance):
        self.pair = motor_pair.PAIR_1

        motor_pair.pair(self.pair, left_motor_port, right_motor_port)
        self.rotation_distance = rotation_distance
        self.default_stop_behavior = STOP_BEHAVIOR
        self.acceleration = ACCELERATION

        runloop.sleep_ms(2000)

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


def configure():
    global MOTION
    MOTION = Motion(LEFT_WHEEL_PORT, RIGHT_WHEEL_PORT, 27.4)

# This is the main function. It is like the "when program starts" block in scratch
async def main():
    print('main loop') # This text is printed to the terminal

    # write your code here
    
    

    # The await keyboard makes the app wait until the line is done before moving to the next line
    await light_matrix.write("Hi!")
    blink()

    configure()
    
    await MOTION.move_distance(30)
    await MOTION.turn_relative(135)
    await MOTION.move_distance(42.42)
    await MOTION.turn_relative(135)
    await MOTION.move_distance(30)
    await MOTION.turn_relative(90)
    blink()






    print('all done\n\n\n')

    





# This is the line that calls the main function. It has to be at the end of the file.
runloop.run(main())
