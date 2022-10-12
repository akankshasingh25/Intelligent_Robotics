"""Bug0 Algorithm"""

from controller import Robot, Motor, DistanceSensor
from helper_functions import *

robot = Robot()

# CONSTANT DECLARATION
TIME_STEP = 64
MAX_SPEED = 6.28
GOAL_POSITION = [0.25, 0.50, 0.0]
REACHED_GOAL = 0.05
ENCOUNTERED_OBSTACLE = 100
GOAL_IN_RANGE = 0.05

# motors initialization
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# devices initialization

# proximity sensor to know goal or obstacle in range
proximity_sensor = []
proximity_sensor_names = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]
for i in range(8):
    proximity_sensor.append(robot.getDevice(proximity_sensor_names[i]))
    proximity_sensor[i].enable(TIME_STEP)

proximity_sensor_values = [0 for i in range(8)]

# gps to get the location
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

# compass to get the sense of direction
compass = robot.getDevice('compass')
compass.enable(TIME_STEP)

state = 'start'

# main loop
while robot.step(TIME_STEP) != -1:

    left_speed  = 0.5 * MAX_SPEED
    right_speed = 0.5 * MAX_SPEED

    for i in range(8):
        proximity_sensor_values[i] = proximity_sensor[i].getValue() 
           
    current_position = gps.getValues()
    current_angle = get_bearing_in_degrees(compass.getValues())
  
    # start of time
    if state == 'start':
        start_position = gps.getValues()
        aligned_to_goal = angle_of(current_position, GOAL_POSITION) > 0.95*current_angle and angle_of(current_position, GOAL_POSITION) < 1.05*current_angle
         
        if aligned_to_goal:
            state = 'go_to_goal'
        else:
            print('Robot status: Aligning to the goal')
            left_speed  = -0.5 * MAX_SPEED
            right_speed = 0.5 * MAX_SPEED
            state = 'start'
            
    elif state == 'go_to_goal':

        obstacle_detected = proximity_sensor_values[0] > ENCOUNTERED_OBSTACLE and proximity_sensor_values[7] > ENCOUNTERED_OBSTACLE

        if obstacle_detected:
            hit_point = gps.getValues()
            hit_angle = get_bearing_in_degrees(compass.getValues())
            state = 'follow_obstacle'

        elif distance_between(current_position, GOAL_POSITION) <= REACHED_GOAL:
            state = 'end'

        elif not on_line(current_position, start_position, GOAL_POSITION):
            # move back on line
            heading_angle = current_angle
            goal_angle = angle_of(start_position, GOAL_POSITION)
            
            if (heading_angle - goal_angle) > GOAL_IN_RANGE:
                print('Robot status: Aligning to the goal')
                left_speed  = 0.5 * MAX_SPEED
                right_speed = 0.1 * MAX_SPEED

            elif (heading_angle - goal_angle) < -GOAL_IN_RANGE:
                print('Robot status: Aligning to the goal')
                left_speed  = 0.1 * MAX_SPEED
                right_speed = 0.5 * MAX_SPEED

        else:
            print('Robot status: Moving to goal')
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)
            
    elif state == 'follow_obstacle':
        print('Robot status: Following Obstacle Boundary')
        front_clear = max(proximity_sensor_values[0:1]) < ENCOUNTERED_OBSTACLE and max(proximity_sensor_values[6:7]) < ENCOUNTERED_OBSTACLE
        aligned_to_goal = angle_of(current_position, GOAL_POSITION) > 0.95*current_angle and angle_of(current_position, GOAL_POSITION) < 1.05*current_angle

        if front_clear and aligned_to_goal:
            print('Robot status: Goal Reachable')
            state = 'go_to_goal'
            left_speed  = 0.20 * MAX_SPEED
            right_speed = 0.50 * MAX_SPEED
            start_position = current_position
            continue
  
        right_side_covered = proximity_sensor_values[2] > ENCOUNTERED_OBSTACLE
        if not right_side_covered:
            left_speed  = -0.5 * MAX_SPEED
            right_speed = 0.5 * MAX_SPEED

        else:
            right_value = max(proximity_sensor_values[0:2])
            left_value = max(proximity_sensor_values[5:7])

            if right_value > 2.0*ENCOUNTERED_OBSTACLE:
                left_speed  = 0.20 * MAX_SPEED
                right_speed = 0.50 * MAX_SPEED

            elif right_value < ENCOUNTERED_OBSTACLE and left_value < ENCOUNTERED_OBSTACLE:
                left_speed  = 0.50 * MAX_SPEED
                right_speed = 0.20 * MAX_SPEED

            else:
                left_speed  = 0.50 * MAX_SPEED
                right_speed = 0.50 * MAX_SPEED
                
    elif state == 'end':
        print('Robot status: Goal Reached')
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        break
        
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
