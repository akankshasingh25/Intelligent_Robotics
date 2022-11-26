"""Supervisor controller."""
from controller import Supervisor
from controller import Node
from controller import Robot, Motor, DistanceSensor
import math

def angle(robot_pos, goal_pos):
    x = robot_pos[0] - goal_pos[0]
    y = robot_pos[1] - goal_pos[1]
    rad = math.atan2(y, x)
    return rad + math.pi
    
def dist_betn(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 \
            + (p1[1] - p2[1]) ** 2)

robot = Supervisor()
############## Moving Obstacles ##############
dynamic_obstacle1 = robot.getFromDef("dynamic_obs1")
dynamic_obstacle2 = robot.getFromDef("dynamic_obs2")
print(dynamic_obstacle1)

position_1 = dynamic_obstacle1.getPosition()
position_2 = dynamic_obstacle2.getPosition()

tran_1 = dynamic_obstacle1.getField('translation')
tran_2 = dynamic_obstacle2.getField('translation')

target_1 = [-0.7, 0.25, 0.05]
target_2 = [-0.6, 0.3, 0.05]

velocity_1 = dist_betn(target_1, position_1)/8000
velocity_2 = dist_betn(target_2, position_2)/10000 

angle_1 = angle(position_1, target_1)
angle_2 = angle(position_2, target_2)

timestep = 64

while robot.step(timestep) != -1:

    position_1 = dynamic_obstacle1.getPosition()
    position_2 = dynamic_obstacle2.getPosition()
    
    delta_pos_1 = [velocity_1 * math.cos(angle_1),
                   velocity_2 * math.sin(angle_1), 0]
                                
    delta_pos_2 = [velocity_2 * math.cos(angle_2), 
                   velocity_2 * math.sin(angle_2), 0]

    if dist_betn(target_1, position_1) > 0.05:
        position_1 = [position_1[i] + delta_pos_1[i] 
                        for i in range(3)]
        tran_1.setSFVec3f(position_1)
    else:
        tran_1.setSFVec3f(target_1)
    
    if dist_betn(target_2, position_2) > 0.05:
        position_2 = [position_2[i] + delta_pos_2[i] 
                        for i in range(3)]
        tran_2.setSFVec3f(position_2)
    else:
        tran_2.setSFVec3f(target_2)

############### Controller End ################
