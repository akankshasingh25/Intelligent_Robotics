from controller import Robot
from controller import Supervisor
from controller import Supervisor
import sys
import math
from math import *

def angle_trunc(a):
    while a < 0.0:
        a += pi * 2
    return a

def getAngleBetweenPoints(x_orig, y_orig, x_landmark, y_landmark):
    deltaY = y_landmark - y_orig
    deltaX = x_landmark - x_orig
    return angle_trunc(atan2(abs(deltaY), abs(deltaX)))
def get_distance(goal_x,goal_y,x_current,y_current):
    return ((goal_y-y_current)**2+(goal_x-x_current)**2)**(1/2)
    
def get_bearing_in_degrees(north):
  rad = math.atan2(north[1], north[0])
  bearing = (rad - 1.5708) / 3.14 * 180.0
  if (bearing < 0.0):
    bearing = bearing+360    
  return bearing

def run_robot(robot):      
    TIME_STEP = 32
    timestep =1
    max_speed = 6.67 
      
    # enable motor
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    print("left motor",left_motor)
    print("right motor",right_motor)
    
    right_sensor = robot.getDevice('right wheel sensor')
    right_sensor.enable(timestep)    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)    
    
    lidar_sensor = robot.getDevice("LDS-01")
    print("lidar_sensor",lidar_sensor)
    lidar_sensor.enable(timestep)
    lidar_sensor.enablePointCloud()
    
    # trans_field = position_sensor.getField("translation")
    # values = trans_field.getSFVec3f()
    # print("MY_ROBOT is at position: %g %g %g" % (values[0], values[1], values[2]))
      
    gps = robot.getDevice("gps")
    print("gps",gps)
    gps.enable(timestep)
    
    compass = robot.getDevice("compass")
    compass.enable(timestep)
    
    print("compass|||||||||||||||||||||",compass)    
    # goal_x = goal_x
    # goal_y = goal_y
        
    angles = []
    left_speed = max_speed
    right_speed = max_speed
    while robot.step(timestep) != -1:
        compass_value = compass.getValues()
        bearing_in_degrees= get_bearing_in_degrees(compass_value)
        bearing_in_degrees = 360-bearing_in_degrees
        # print("heading angle",bearing_in_degrees)
        
        gps_val = gps.getValues()
        gps_x = gps_val[0]
        gps_y = gps_val[1]
        print("robot location:",gps_x,gps_y)
        point1_x = -0.7
        point1_y = -1.2
        
        point2_x= -0.168
        point2_y = -1.11
        
        point3_x = -0.186
        point3_y = 1.4
        
        point4_x = 0.135
        point4_y = 1.345
        
        point5_x = 0.12
        point5_y = -1.23
        
        point6_x = 0.7
        point6_y = -1.23
        range_image = lidar_sensor.getRangeImage()
        print('Lidar ***********',min(range_image) )     
            
        point_4_distance = get_distance(point4_x,point4_y,gps_x,gps_y)
        point_5_distance = get_distance(point5_x,point5_y,gps_x,gps_y)
        for x in range_image[:]:
        # for y in 
            if x==inf:
            # if x<0.65:
                left_speed=-max_speed
                right_speed=max_speed
            elif get_distance(point1_x,point1_y,gps_x,gps_y)<0.05 and bearing_in_degrees>1:
                print("robot at point 1")
                left_speed = max_speed
                right_speed = max_speed
            elif get_distance(point2_x,point2_y,gps_x,gps_y)<0.05 and bearing_in_degrees<90:
                print("robot at point 2")
                left_speed = -max_speed
                right_speed = max_speed
            elif get_distance(point3_x,point3_y,gps_x,gps_y)<0.05 and bearing_in_degrees>1:
                print("robot at point 3")
                left_speed = max_speed
                right_speed = -max_speed
            elif point_4_distance<0.06 and (bearing_in_degrees<1 or bearing_in_degrees>270) :
                print("robot at point 4")
                left_speed = max_speed
                right_speed = -max_speed*0.25
            elif point_5_distance<0.05 and (bearing_in_degrees>269 or bearing_in_degrees<0.40):
                print("robot at point 5")
                left_speed = -max_speed
                right_speed = max_speed
            elif get_distance(point6_x,point6_y,gps_x,gps_y)<0.05:
                print("robot reached at target point")
                left_speed = 0
                right_speed = 0
            
            else :
                left_speed = max_speed
                right_speed = max_speed
                
          
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        # print("   success")
        
        ###########OBSTACLE AVOIDANCE BEHAVIOUR#####################
        """
          # closest obstacle to the right
        if right_value < 3.0 and right_value < left_value and right_value < front_value:
        left_speed -= 0.25 * MAX_SPEED * (3.0 - right_value) / 3.0
        right_speed += 0.25 * MAX_SPEED * (3.0 - right_value) / 3.0

        # closest obstacle to the left
        if left_value < 3.0 and left_value < right_value and left_value < front_value:
        left_speed += 0.25 * MAX_SPEED * (3.0 - left_value) / 3.0
        right_speed -= 0.25 * MAX_SPEED * (3.0 - right_value) / 3.0

        # if obstacle in front, turn away from it
        if front_value < 3.0:
        if left_value > right_value:
            left_speed = -0.5 * MAX_SPEED
            right_speed = 0.5 * MAX_SPEED
        else:
            left_speed = 0.5 * MAX_SPEED
            right_speed = -0.5 * MAX_SPEED
       """
if __name__=="__main__":
    robot = Robot()
    run_robot(robot)
     
        
        
# myrobot = Robot()
# coordinate_list = [(-0.14,-1.11819)]
# for corr in coordinate_list:
    # run_robot(myrobot,corr[0],corr[1])
    
    
# #gps_val = gps.getValues()
        # compass_value = compass.getValues()
        # bearing_in_degrees= get_bearing_in_degrees(compass_value)
        # bearing_in_degrees = 360-bearing_in_degrees
        
        
        # #angle = getAngleBetweenPoints(float("{:.2f}".format(gps_val[0])),float("{:.1f}".format(gps_val[1])),goal_x,goal_y)
        
        # print(float("{:.2f}".format(gps_val[0])),float("{:.1f}".format(gps_val[1])),goal_x,goal_y)
        # angle= math.degrees(angle)
        # compass_val = []
        # for i in compass_value:
            # compass_val.append(float("{:.5f}".format(i)))
           
           
        # #print("gps_val",gps_val)

        # gps_x = gps_val[0]
        # gps_y = gps_val[1]
        
        # distance = get_distance(goal_x,goal_y,gps_x,gps_y)
        # #print(goal_x,goal_y,gps_x,gps_y)
        
        # difference = angle-bearing_in_degrees
        # #print("diffrence",difference)
        # #print("angle",angle,bearing_in_degrees)
        # print("angle",angle)
        # print("distance",distance)
        # angles.append(angle)
        
        
        
        # if distance>0.05:
        
            # if abs(difference) >2:
                # if difference < 0:
                    # left_speed = max_speed
                    # right_speed = -max_speed
                # else:
                    # left_speed = -max_speed
                    # right_speed = max_speed
            # else :
                # left_speed = max_speed
                # right_speed = max_speed
        # else :
            # left_speed = 0
            # right_speed = 0
            # left_motor.setVelocity(left_speed)
            # right_motor.setVelocity(right_speed)
            # return " "