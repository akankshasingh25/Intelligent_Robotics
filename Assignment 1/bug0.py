"""Bug 0 Algorithm"""

from controller import Robot, Motor, DistanceSensor
from helper_functions import *

#def run_bug0(robot):
robot = Robot()

timestep = 64
max_speed = 6.28
obs_prox = 150
pos_epsilon = 0.05
angle_epsilon = 0.05
goal_pos = [0,0,0]

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor') 

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

prox_sensors = []
prox_sensor_names = ['ps0', 'ps1', 'ps2', 'ps3',
                    'ps4', 'ps5','ps6','ps7']

for i in range(8):
    prox_sensors.append(robot.getDevice(prox_sensor_names[i]))
    prox_sensors[i].enable(timestep)

prox_sensors_values = [0 for i in range(8)]

state = 'start'

# Main loop:
while robot.step(timestep) != -1:
    
    left_motor_speed = max_speed *0.5
    right_motor_speed = max_speed*0.5
    
    for i in range(8):
        prox_sensors_values[i] = proxc_sensors[i].getValues()
        
    curr_pos = gps.getValues()
    current_angle = get_bearing_in_degrees(compass.getValues())
    
    if state == 'start':
        start_pos = gps.getValues()
        aligned_to_goal = (angle_of(curr_pos, goal_pos)> curr_angle*0.98) and (angle_of(curr_pos, goal_pos)< curr_angle*1.02)
        
        if not aligned_to_goal:
            print('Aligning to goal')
            left_motor_speed = -max_speed*0.5
            right_motor_speed = max_speed*0.5
            state = 'start'
        else:
            state = 'go to goal'
           
    elif state == 'go to goal':
                
        obstacle_is_detected = (prox_sensors_values[0] and prox_sensors_values[7]) > obs_prox        
        if obstacle_is_detected:
            hit_point = gps.getValues()
            hit_point_angle = get_bearing_in_degrees(compass.getValues())
            state = 'follow the obstacle'
            
        elif not_on_line(start_pos, goal_pos, curr_pos):
            heading_angle = curr_angle
            goal_angle = angle_of(start_pos, goal_pos)
            
            if (heading_angle-goal_angle)> angle_epsilon:
                left_motor_speed = 0.3*max_speed
                right_motor_speed = 0.1*max_speed
                print('Aligning to the goal')
                
            elif (heading_angle-goal_angle) < -angle_epsilon:
                left_motor_speed = 0.1*max_speed
                right_motor_speed = 0.3*max_speed
                print('Aligning to the goal')  
                  
        elif distance_between(curr_pos, goal_pos) < pos_epsilon:
            state = 'end'
            
        else:
            print('Moving to the goal')
            left_motor.setVelocity(left_motor_speed)
            right_motor.setVelocity(right_motor_speed)
            
            
    elif state == 'follow the obstacle':
        print('Following the obstacle')
        
        front_clear = max(prox_sensors_values[6:1]) < obs_prox
        aligned_to_goal = (angle_of(curr_pos, goal_pos)> curr_angle*0.98) and (angle_of(curr_pos, goal_pos)< curr_angle*1.02)
        
        if front_clear and aligned_to_goal:
            state = 'go to goal'
            print('Goal is reachable')
            left_motor_speed = max_speed*0.1
            right_motor_speed = max_speed*0.3
            start_pos = curr_pos
            continue
            
        right_side_covered = prox_sensors_values[2] > obs_prox
        if right_side_covered:
            state = 'follow the obsatcle'
        elif not right_side_covered:
            left_motor_speed = max_speed*(-0.5)
            right_motor_speed = max_speed*(0.5)
        else:
            right_value = max(prox_sensors_values[0:2])
            left_value = max(prox_sensors_values[5:7])
            if right_value > 2.0*obs_prox:
                left_motor_speed  = 0.20 * max_speed
                right_motor_speed = 0.50 * max_speed
            elif right_value < obs_prox and left_value < obs_prox:
                left_motor_speed  = 0.50 * max_speed
                right_motor_speed = 0.20 * max_speed
            else:
                left_motor_speed  = 0.50 * max_speed
                right_motor_speed = 0.50 * max_speed
                
    elif state == 'end':
        print('Goal reached')
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        break
        
    left_motor.setVelocity(left_motor_speed)
    right_motor.setVelocity(right_motor_speed)

# Enter here exit cleanup code.
"""
if __name__ == 'main':
    robot = Robot()
    run_bug0(robot)"""
