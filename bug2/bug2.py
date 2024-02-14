from controller import Robot
import math
from enum import Enum
import matplotlib.pyplot as plt

states = Enum("state", 'default wall_following_right wall_following_left make_parallel turn_around_out heading_toward_goal end')
thresh_wall_side_rear = 0.062
thresh_danger_side_rear = 0.06
thresh_wall_side_front = 0.0657
thresh_wall_forward = 0.062 
max_velocity = 1
global xt, yt
xt, yt = (0, -0.5)
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

front_r = robot.getDevice('front_r')
front_r.enable(timestep)

front_l = robot.getDevice('front_l')
front_l.enable(timestep)

def calculate_euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    
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
    

def calc_route_line():
    x, y, _ = gps.getValues()
    xt, yt = (0, -0.5)
    slope = (yt - y) / (xt - x)
    b = yt - xt * slope
    return b, slope

def is_on_route_line(slope, b):
    x, y, _ = gps.getValues()
    
    return abs(x) < 0.02
    
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
    
leave_point = None
hit_point = None         
robot.step(timestep) 
B, SLOPE = calc_route_line() 
prev_dir = None   
while not_on_the_line():      
      turn_completly_right()
      robot.step(timestep)

counter = 0     
curr_s = states.default 
move_forward() 
prev_s = None 

routex = []
routey = []   
while robot.step(timestep) != -1:
        x, y, _ = gps.getValues()
      
        routex.append(x)
        routey.append(y)
        if abs(calculate_euclidean_distance(x, y, 0, -0.5)) < 0.03 and curr_s != states.end:
            print("end!!!")
            right_wheel.setVelocity(+max_velocity)
            left_wheel.setVelocity(-max_velocity)
            plt.scatter(routex, routey, s=1)
            plt.xlabel("X")
            plt.ylabel("Y")
            plt.title("cooordinates")
            plt.show()
            curr_s = states.end
        if curr_s == states.default:
            
            if front.getValue() < thresh_wall_forward and right_rear.getValue() < thresh_wall_side_rear:
                
                turn_completly_left()
                curr_s = states.make_parallel
            elif front.getValue() < thresh_wall_forward and left_rear.getValue() < thresh_wall_side_rear:
                
                turn_completly_right()
                curr_s = states.make_parallel
            elif front.getValue() < thresh_wall_forward:
                
                turn_completly_left()
                curr_s = states.make_parallel
                
        elif curr_s == states.make_parallel:
            
            if parallel():  
                
                if right_rear.getValue() < thresh_wall_side_rear:
                    move_forward()
                    curr_s = states.wall_following_right
                elif left_rear.getValue() < thresh_wall_side_rear: 
                    move_forward()
                    curr_s = states.wall_following_left
        elif curr_s == states.wall_following_right:
            
            if front.getValue() < thresh_wall_forward and right_rear.getValue() < thresh_wall_side_rear:
                
                turn_completly_left()
                curr_s = states.make_parallel
            elif front.getValue() < thresh_wall_forward and left_rear.getValue() < thresh_wall_side_rear:
                
                turn_completly_right()
                curr_s = states.make_parallel
            elif front.getValue() < thresh_wall_forward:
               
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
            
            if is_on_route_line(SLOPE, B):
                  
                 x, y, _ = gps.getValues()
                 leave_point = (x, y)
                 if hit_point is None:
                     hit_point = (x, y)
                 
                 if hit_point[1] - leave_point[1] > 0.01 :
                     right_wheel.setVelocity(0)
                     left_wheel.setVelocity(0)
                     curr_s = states.heading_toward_goal 
                       
        elif curr_s == states.wall_following_left:
             
             if front.getValue() < thresh_wall_forward and right_rear.getValue() < thresh_wall_side_rear:
                
                turn_completly_left()
                curr_s = states.make_parallel
             elif front.getValue() < thresh_wall_forward and left_rear.getValue() < thresh_wall_side_rear:

                turn_completly_right()
                curr_s = states.make_parallel
             elif front.getValue() < thresh_wall_forward:

                
                turn_completly_left()
                curr_s = states.make_parallel
             elif left_rear.getValue() < thresh_danger_side_rear:
                
                right_wheel.setVelocity(max_velocity * 0.95)
                left_wheel.setVelocity(max_velocity * 1)
             elif left_rear.getValue() > thresh_danger_side_rear and left_rear.getValue() < 0.08:
                
                right_wheel.setVelocity(max_velocity * 1)
                left_wheel.setVelocity(max_velocity * 0.95)
             elif left_rear.getValue() > 0.2:
                 if counter >= 5:
                     counter = 0
                     
                     right_wheel.setVelocity(max_velocity * 0.7)
                     left_wheel.setVelocity(max_velocity * 0.3)
                     prev_s = curr_s
                     curr_s = states.turn_around_out
                 counter += 1
             
             if is_on_route_line(SLOPE, B) :
                 x, y, _ = gps.getValues()
                 leave_point = (x, y)
                 if hit_point is None:
                     hit_point = (x, y)
                 
                 
                 if hit_point[1] - leave_point[1] > 0.01 :
                     right_wheel.setVelocity(0)
                     left_wheel.setVelocity(0)
                     
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
                hit_point = (x, y)
                curr_s = states.default
                move_forward()
