from controller import Robot
import math
from enum import Enum
import numpy as np
import matplotlib.pyplot as plt
states = Enum("state", 'default wall_following_right wall_following_left make_parallel turn_around_out heading_toward_goal end')
thresh_wall_side_rear = 0.062
thresh_danger_side_rear = 0.06
thresh_wall_side_front = 0.0657
thresh_wall_forward = 0.062
max_velocity = 1
global xt, yt
xt, yt = (0, -0.5)
starting_pos = None
curr_pos = None
min_dist_pos = (math.inf, math.inf)
position_array = []

robot = Robot()

timestep = int(robot.getBasicTimeStep())

gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

left_wheel = robot.getDevice('left wheel motor')
right_wheel = robot.getDevice('right wheel motor')

left_wheel.setPosition(float('inf'))
right_wheel.setPosition(float('inf'))

right_front = robot.getDevice('right_front')
right_front.enable(timestep)

left_front = robot.getDevice('left_front')
left_front.enable(timestep)

right_rear = robot.getDevice('right_rear')
right_rear.enable(timestep)

left_rear = robot.getDevice('left_rear')
left_rear.enable(timestep)

front = robot.getDevice('front')
front.enable(timestep)

f_detect_r = robot.getDevice('f_detect_r')
f_detect_r.enable(timestep)

f_detect_l = robot.getDevice('f_detect_l')
f_detect_l.enable(timestep)

front_r = robot.getDevice('front_r')
front_r.enable(timestep)

front_l = robot.getDevice('front_l')
front_l.enable(timestep)





def get_robot_heading(compass_value):
    rad = math.atan2(compass_value[0], compass_value[1])
    bearing = (rad - 1.5708) / math.pi * 180.0
    if bearing < 0.0:
        bearing = bearing + 360.0

    heading = 360 - bearing
    if heading > 360.0:
        heading -= 360.0
    return heading

def find_bias(value):
    return round(math.atan2(value[0], value[1]) * 180 / math.pi) % 360
    
    
def not_on_the_line():
    x, y, _ = gps.getValues()    
    cx, cy, _ = compass.getValues()
    heading = get_robot_heading((cx, cy))
    target_bias = find_bias((xt - x, yt - y))
    
    if abs(heading - target_bias) > 2:
        return True
    return False
    
def turn_completly_right():
      right_wheel.setVelocity(-max_velocity)
      left_wheel.setVelocity(+max_velocity)


def turn_completly_left():
      right_wheel.setVelocity(+max_velocity)
      left_wheel.setVelocity(-max_velocity) 
       
def move_forward():
      right_wheel.setVelocity(+max_velocity)
      left_wheel.setVelocity(+max_velocity)
      
def parallel():
    
    if right_rear.getValue() < 0.07 and right_front.getValue() > thresh_wall_side_front:
        return True
    elif left_rear.getValue() < 0.07 and left_front.getValue() > thresh_wall_side_front:
        return True
    return False
    
def calculate_euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)    
        
robot.step(timestep) 

prev_dir = None   
while not_on_the_line():      
      turn_completly_right()
      robot.step(timestep)

counter = 0
steps = 0  
curr_s = states.default 
move_forward() 
prev_s = None
full_round = False  
distancesx = []
distancesy = []  
routex = []
routey = []
while robot.step(timestep) != -1:
        steps += 1
        x, y, _ = gps.getValues()
        cx, cy, _ = compass.getValues()
        heading = math.atan2(cx, cy)
        distance = right_rear.getValue()
        if distance < 0.09:
            distancesx.append(distance*math.cos(heading - 1.57) + x)
            distancesy.append(distance*math.sin(heading - 1.57) + y)
        routex.append(x)
        routey.append(y)
        curr_pos = (x, y)
        if calculate_euclidean_distance(curr_pos[0], curr_pos[1], 0, -0.5) < calculate_euclidean_distance(min_dist_pos[0], min_dist_pos[1], 0, -0.5) and not full_round:
            
            min_dist_pos = curr_pos
        if starting_pos is not None and abs(curr_pos[0] - starting_pos[0]) < 0.01 and abs(curr_pos[1] - starting_pos[1]) < 0.01 and not full_round and steps > 1000:
            print(min_dist_pos)
            print(curr_pos)
            print("reached to the first point")
            plt.scatter(distancesx, distancesy)
            plt.xlabel("X")
            plt.ylabel("Y")
            plt.title("cooordinates")
            plt.show()
            full_round = True
        if curr_s == states.default:
            if abs(calculate_euclidean_distance(curr_pos[0], curr_pos[1], 0, -0.5)) < 0.02:
                print("object found!!!!")
                curr_s = states.end
                right_wheel.setVelocity(max_velocity)
                left_wheel.setVelocity(-max_velocity)
                
                plt.scatter(routex, routey, s=1)
                plt.xlabel("X")
                plt.ylabel("Y")
                plt.title("cooordinates")
                plt.show()
                
            if f_detect_r.getValue() < thresh_wall_forward and right_rear.getValue() < thresh_wall_side_rear:
                
                turn_completly_left()
                curr_s = states.make_parallel
                if starting_pos is None:
                    x, y, _ = gps.getValues()
                    starting_pos = (x, y)
                    
            elif f_detect_l.getValue() < thresh_wall_forward and left_rear.getValue() < thresh_wall_side_rear:
                
                turn_completly_right()
                curr_s = states.make_parallel
                if starting_pos is None:
                    x, y, _ = gps.getValues()
                    
                    starting_pos = (x, y)
                    
            elif f_detect_l.getValue() < thresh_wall_forward or f_detect_r.getValue() < thresh_wall_forward:
                
                turn_completly_left()
                curr_s = states.make_parallel
                if starting_pos is None:
                    x, y, _ = gps.getValues()
                    starting_pos = (x, y)
                    
                
            
                
        elif curr_s == states.make_parallel:
           
            if parallel():  
                
                if right_rear.getValue() < thresh_wall_side_rear:
                    move_forward()
                    curr_s = states.wall_following_right
                elif left_rear.getValue() < thresh_wall_side_rear: 
                    move_forward()
                    curr_s = states.wall_following_left
        elif curr_s == states.wall_following_right:
            
            if f_detect_r.getValue() < thresh_wall_forward and right_rear.getValue() < thresh_wall_side_rear:
                
                turn_completly_left()
                curr_s = states.make_parallel
            elif f_detect_l.getValue() < thresh_wall_forward and left_rear.getValue() < thresh_wall_side_rear:
                
                turn_completly_right()
                curr_s = states.make_parallel
            elif f_detect_l.getValue() < thresh_wall_forward or f_detect_r.getValue() < thresh_wall_forward:
                
                turn_completly_left()
                curr_s = states.make_parallel
            elif right_rear.getValue() < thresh_danger_side_rear:
                
                right_wheel.setVelocity(max_velocity * 1)
                left_wheel.setVelocity(max_velocity * 0.9)
            elif right_rear.getValue() > thresh_danger_side_rear and right_rear.getValue() < 0.08:
                
                right_wheel.setVelocity(max_velocity * 0.9)
                left_wheel.setVelocity(max_velocity * 1)
            elif right_rear.getValue() > 0.2:
                 if counter >= 5:
                     counter = 0
                     
                     right_wheel.setVelocity(max_velocity * 0.3)
                     left_wheel.setVelocity(max_velocity * 0.7)
                     prev_s = curr_s
                     curr_s = states.turn_around_out
                 counter += 1
                 
            if full_round:
                
                if abs(calculate_euclidean_distance(curr_pos[0], curr_pos[1], 0, -0.5) - calculate_euclidean_distance(min_dist_pos[0], min_dist_pos[1], 0, -0.5)) < 0.01:
                    
                    curr_s = states.heading_toward_goal
            
            
                       
        elif curr_s == states.wall_following_left:
             
             if f_detect_r.getValue() < thresh_wall_forward and right_rear.getValue() < thresh_wall_side_rear:
                
                turn_completly_left()
                curr_s = states.make_parallel
             elif f_detect_l.getValue() < thresh_wall_forward and left_rear.getValue() < thresh_wall_side_rear:
                
                turn_completly_right()
                curr_s = states.make_parallel
             elif f_detect_l.getValue() < thresh_wall_forward or f_detect_l.getValue() < thresh_wall_forward:
                
                turn_completly_left()
                curr_s = states.make_parallel
             elif left_rear.getValue() < thresh_danger_side_rear:
                
                right_wheel.setVelocity(max_velocity * 0.9)
                left_wheel.setVelocity(max_velocity * 1)
             elif left_rear.getValue() > thresh_danger_side_rear and left_rear.getValue() < 0.08:
                
                right_wheel.setVelocity(max_velocity * 1)
                left_wheel.setVelocity(max_velocity * 0.9)
             elif left_rear.getValue() > 0.2:
                 if counter >= 5:
                     counter = 0
                     
                     right_wheel.setVelocity(max_velocity * 0.7)
                     left_wheel.setVelocity(max_velocity * 0.3)
                     prev_s = curr_s
                     curr_s = states.turn_around_out
                 counter += 1
                 
             if full_round:
                
                if abs(calculate_euclidean_distance(curr_pos[0], curr_pos[1], 0, -0.5) - calculate_euclidean_distance(min_dist_pos[0], min_dist_pos[1], 0, -0.5)) < 0.01:
                   
                    curr_s = states.heading_toward_goal
             
            
                     
                 
        elif curr_s == states.turn_around_out:
             if prev_s == states.wall_following_right:
                 if front_r.getValue() < thresh_wall_side_front:
                     
                     
                     right_wheel.setVelocity(max_velocity * 0.1)
                     left_wheel.setVelocity(max_velocity * 0.04)
                     curr_s = states.make_parallel
                 
             elif prev_s == states.wall_following_left:
                 
                 if front_l.getValue() < thresh_wall_side_front:
                     
                     right_wheel.setVelocity(max_velocity * 0.04)
                     left_wheel.setVelocity(max_velocity * 0.1)
                     curr_s = states.make_parallel
                     
        elif curr_s == states.heading_toward_goal:
            turn_completly_right() 
                      
            if not not_on_the_line():
                x, y, _ = gps.getValues()
                curr_s = states.default
                move_forward()
