"""Bog0 controller."""


from controller import Robot, Supervisor
from helper import Helper
import math
import csv
import pandas as pd
import matplotlib.pyplot as plt

def enable_proximity_sensors(robot, time_step):
    # enable ps sensors
    ps = []
    psNames = [
        'ps0', 'ps1', 'ps2', 'ps3',
        'ps4', 'ps5', 'ps6', 'ps7'
         ]
    for i in range(8):
        ps.append(robot.getDevice(psNames[i]))
        ps[i].enable(time_step)
        
    return ps

# pos = -0.609522  , -0.330213,  0.044
# rot = -3.0107, 001

def in_circle(center_x, center_y, radius, x, y):
    square_dist = (center_x - x) ** 2 + (center_y - y) ** 2
    return square_dist <= radius ** 2
    
def checkPoint(radius, x, y, percent, startAngle):
 
    # calculate endAngle
    endAngle = 360 / percent + startAngle
 
    # Calculate polar co-ordinates
    polarradius = math.sqrt(x * x + y * y)
    Angle = math.atan(y / x)
 
    # Check whether polarradius is less
    # then radius of circle or not and
    # Angle is between startAngle and
    # endAngle or not
    if (Angle >= startAngle and Angle <= endAngle
                        and polarradius < radius):
        # print("Point (", x, ",", y, ") "
              # "exist in the circle sector")
       return True
    else:
        # print("Point (", x, ",", y, ") "
              # "does not exist in the circle sector")
        return False
    
def area(x1, y1, x2, y2, x3, y3):
 
    return abs((x1 * (y2 - y3) + x2 * (y3 - y1)
                + x3 * (y1 - y2)) / 2.0)
            
def isInside(x1, y1, x2, y2, x3, y3, x, y):
 
    # Calculate area of triangle ABC
    A = area (x1, y1, x2, y2, x3, y3)
 
    # Calculate area of triangle PBC
    A1 = area (x, y, x2, y2, x3, y3)
     
    # Calculate area of triangle PAC
    A2 = area (x1, y1, x, y, x3, y3)
     
    # Calculate area of triangle PAB
    A3 = area (x1, y1, x2, y2, x, y)
     
    # Check if sum of A1, A2 and A3
    # is same as A
    if(A == A1 + A2 + A3):
        return True
    else:
        return False               
                
def run_robot(robot, help, goal):
    time_step = 32
    max_speed =  100
    is_obstacle = False
    epuck = robot.getFromDef('fab')
    create1 = robot.getFromDef('create1')
    create2 = robot.getFromDef('create2')
    start = epuck.getPosition()[:2]
    m_point = start
    
    p_rad = 0.5
    
    percent = 50
    delta = 60
    d = 0.5
    # lets create motor instances
    bleft_motor = robot.getMotor('back_left_wheel_joint')
    bright_motor = robot.getMotor('back_right_wheel_joint')
    
    fleft_motor = robot.getMotor('front_left_wheel_joint')
    fright_motor = robot.getMotor('front_right_wheel_joint')
    
    # set motor position to inf and velocity to 0
    bleft_motor.setPosition(float('inf'))
    bright_motor.setPosition(float('inf'))
    fleft_motor.setPosition(float('inf'))
    fright_motor.setPosition(float('inf'))
    
    bleft_motor.setVelocity(0.0)
    bright_motor.setVelocity(0.0)
    fleft_motor.setVelocity(0.0)
    fright_motor.setVelocity(0.0)
    
    # ps = enable_proximity_sensors(robot, time_step)
    
    # get robot position
    
    # print(epuck)
    # pos = epuck.getField('translation')
    region1 = True
    transition = False
    region2 = False
  
    # with open('path.csv', 'w') as fileobj:
        # writerObj = csv.writer(fileobj)
        # writerObj.writerow(['x', 'y'])
    # rows_list = []
    while robot.step(time_step) != -1:
        
        pos = epuck.getPosition()[:2]
        # writerObj.writerow([pos[0], pos[1]])
        # rows_list.append([pos[0], pos[1]])
        c1_pos = create1.getPosition()[:2]
        c2_pos = create2.getPosition()[:2]
        
        # print(pos, c1_pos, c2_pos)
        
        heading_angle = help.get_heading_angle(epuck.getOrientation())
        heading_angle1 = help.get_heading_angle(create1.getOrientation())
        heading_angle2 = help.get_heading_angle(create2.getOrientation())
        
        # print(heading_angle, heading_angle1, heading_angle2)
        # print(
        
        # if 
        
        
        # print(eps)
        # for i in range(8):
            # print("ind: {}, Val: {}".format(i, ps[i].getValue()))
            
        # front_obs = ps[0].getValue()
        # right_obs = ps[2].getValue()
        # right_corner = ps[1].getValue()
        
        # if  front_obs > 80:
            # is_obstacle = True 
            
        if region1:
            desire_angle = help.angel_line_horizontal((pos[0],pos[1]),goal)
        
            # print(heading_angle, desire_angle)
    
            if heading_angle > desire_angle:
               eps = heading_angle - desire_angle
            else:
               eps = desire_angle - heading_angle
           
           
            # print(help.distance_bw_points(pos, goal))
            if not in_circle(pos[0], pos[1], p_rad, c1_pos[0], c1_pos[1]):
                # print( "false", in_circle(pos[0], pos[1], p_rad, c1_pos[0], c1_pos[1]))
                if eps > 1:
                    if heading_angle < desire_angle:
                        left_speed = -max_speed *0.1
                        right_speed = max_speed *0.1
                        bleft_motor.setVelocity(left_speed)
                        bright_motor.setVelocity(right_speed)
                        fleft_motor.setVelocity(left_speed)
                        fright_motor.setVelocity(right_speed)
                    else:
                        left_speed = max_speed *0.1
                        right_speed = -max_speed *0.1
                        bleft_motor.setVelocity(left_speed)
                        bright_motor.setVelocity(right_speed)
                        fleft_motor.setVelocity(left_speed)
                        fright_motor.setVelocity(right_speed)
                        
                elif eps < 1:
                    left_speed = max_speed * 0.3
                    right_speed = max_speed * 0.3
                    bleft_motor.setVelocity(left_speed)
                    bright_motor.setVelocity(right_speed)
                    fleft_motor.setVelocity(left_speed)
                    fright_motor.setVelocity(right_speed)
                    
            else:
                # print( "True", in_circle(pos[0], pos[1], p_rad, c1_pos[0], c1_pos[1]))
                print("in range")
                if in_circle(pos[0], pos[1], p_rad, c1_pos[0], c1_pos[1]):
                    # if checkPoint(p_rad, pos[0], pos[1], percent, heading_angle-90):
                    if isInside(pos[0], pos[1], pos[0]+(d * math.cos((heading_angle-delta)*math.pi/180)), pos[1]+(d * math.sin((heading_angle-delta)*math.pi/180)), pos[0]+(d * math.cos((heading_angle+delta)*math.pi/180)), pos[1]+(d * math.sin((heading_angle+delta)*math.pi/180)), c1_pos[0], c1_pos[1]):        
                        print("front obs")
                        left_speed = -max_speed 
                        right_speed = -max_speed 
                        bleft_motor.setVelocity(left_speed)
                        bright_motor.setVelocity(right_speed)
                        fleft_motor.setVelocity(left_speed)
                        fright_motor.setVelocity(right_speed)
                    
                    
                # elif checkPoint(p_rad, pos[0], pos[1], percent, 135):
                    # print("left obs")
                    # left_speed = max_speed 
                    # right_speed = max_speed 
                    # bleft_motor.setVelocity(left_speed)
                    # bright_motor.setVelocity(right_speed)
                    # fleft_motor.setVelocity(-left_speed)
                    # fright_motor.setVelocity(-right_speed)
                    
                # elif checkPoint(p_rad, pos[0], pos[1], percent, 225):
                    # print("back obs")
                    # left_speed = max_speed 
                    # right_speed = max_speed 
                    # bleft_motor.setVelocity(left_speed)
                    # bright_motor.setVelocity(right_speed)
                    # fleft_motor.setVelocity(left_speed)
                    # fright_motor.setVelocity(right_speed)
                    
                # elif checkPoint(p_rad, pos[0], pos[1], percent, 315):
                    # print("right obs")
                    # left_speed = max_speed 
                    # right_speed = max_speed 
                    # bleft_motor.setVelocity(-left_speed)
                    # bright_motor.setVelocity(-right_speed)
                    # fleft_motor.setVelocity(left_speed)
                    # fright_motor.setVelocity(right_speed)
                    
                    
                    
            if help.distance_bw_points(pos, goal) < 0.15:
                left_speed = max_speed *0
                right_speed = max_speed *0
                bleft_motor.setVelocity(left_speed)
                bright_motor.setVelocity(right_speed)
                fleft_motor.setVelocity(left_speed)
                fright_motor.setVelocity(right_speed)
                region1 = False
                transition = True
                
        
        elif transition:
            print("transition")
            goal = (0.3, 1.35)
            desire_angle = help.angel_line_horizontal((pos[0],pos[1]),goal)
        
            print(heading_angle, desire_angle)
    
            if heading_angle > desire_angle:
               eps = heading_angle - desire_angle
            else:
               eps = desire_angle - heading_angle
               
               
            # print(help.distance_bw_points(pos, goal))
            
            if not in_circle(pos[0], pos[1], p_rad, c1_pos[0], c1_pos[1]):
                # print( "false", in_circle(pos[0], pos[1], p_rad, c1_pos[0], c1_pos[1]))
                if eps > 1:
                    if heading_angle < desire_angle:
                        left_speed = -max_speed * 0.1
                        right_speed = max_speed * 0.1
                        bleft_motor.setVelocity(left_speed)
                        bright_motor.setVelocity(right_speed)
                        fleft_motor.setVelocity(left_speed)
                        fright_motor.setVelocity(right_speed)
                    else:
                        left_speed = max_speed * 0.1
                        right_speed = -max_speed * 0.1
                        bleft_motor.setVelocity(left_speed)
                        bright_motor.setVelocity(right_speed)
                        fleft_motor.setVelocity(left_speed)
                        fright_motor.setVelocity(right_speed)
                        
                elif eps < 1:
                    left_speed = max_speed * 0.04
                    right_speed = max_speed * 0.04
                    bleft_motor.setVelocity(left_speed)
                    bright_motor.setVelocity(right_speed)
                    fleft_motor.setVelocity(left_speed)
                    fright_motor.setVelocity(right_speed)
                    
            else:
                # print( "True", in_circle(pos[0], pos[1], p_rad, c1_pos[0], c1_pos[1]))
                print("in range")
                if in_circle(pos[0], pos[1], p_rad, c2_pos[0], c2_pos[1]):
                    # if checkPoint(p_rad, pos[0], pos[1], percent, heading_angle-90):
                    if isInside(pos[0], pos[1], pos[0]+(d * math.cos((heading_angle-delta)*math.pi/180)), pos[1]+(d * math.sin((heading_angle-delta)*math.pi/180)), pos[0]+(d * math.cos((heading_angle+delta)*math.pi/180)), pos[1]+(d * math.sin((heading_angle+delta)*math.pi/180)), c2_pos[0], c2_pos[1]): 
                        print("front obs")
                        left_speed = -max_speed 
                        right_speed = -max_speed 
                        bleft_motor.setVelocity(left_speed)
                        bright_motor.setVelocity(right_speed)
                        fleft_motor.setVelocity(left_speed)
                        fright_motor.setVelocity(right_speed)
                    
                    
            if help.distance_bw_points(pos, goal) < 0.15:
                left_speed = max_speed *0
                right_speed = max_speed *0
                bleft_motor.setVelocity(left_speed)
                bright_motor.setVelocity(right_speed)
                fleft_motor.setVelocity(left_speed)
                fright_motor.setVelocity(right_speed)
                region2 = True
                transition = False
                
                
        
        elif region2:
            print("region2")
            
            goal = (0.7, -1.23)
            
            desire_angle = help.angel_line_horizontal((pos[0],pos[1]),goal)
        
            print(heading_angle, desire_angle)
    
            if heading_angle > desire_angle:
               eps = heading_angle - desire_angle
            else:
               eps = desire_angle - heading_angle
               
               
            print(eps, help.distance_bw_points(pos, goal))
            
            if not in_circle(pos[0], pos[1], p_rad, c2_pos[0], c2_pos[1]):
                # print( "false", in_circle(pos[0], pos[1], p_rad, c1_pos[0], c1_pos[1]))
                if eps > 0.9:
                    # if heading_angle < desire_angle:
                        # print("left turn")
                        # left_speed = max_speed * 0.1
                        # right_speed = max_speed * 0.1
                        # bleft_motor.setVelocity(-left_speed)
                        # bright_motor.setVelocity(right_speed)
                        # fleft_motor.setVelocity(-left_speed)
                        # fright_motor.setVelocity(right_speed)
                    # else:
                    
                        print("right turn")
                        left_speed = max_speed * 0.1
                        right_speed = max_speed * 0.1
                        bleft_motor.setVelocity(left_speed)
                        bright_motor.setVelocity(-right_speed)
                        fleft_motor.setVelocity(left_speed)
                        fright_motor.setVelocity(-right_speed)
                        
                elif eps < 0.9:
                    print("go forward")
                    left_speed = max_speed * 0.3
                    right_speed = max_speed * 0.3
                    bleft_motor.setVelocity(left_speed)
                    bright_motor.setVelocity(right_speed)
                    fleft_motor.setVelocity(left_speed)
                    fright_motor.setVelocity(right_speed)
                    
            else:
                print("in range")
                # print( "True", in_circle(pos[0], pos[1], p_rad, c1_pos[0], c1_pos[1]))
                if in_circle(pos[0], pos[1], 0.6, c2_pos[0], c2_pos[1]):
                    # if checkPoint(p_rad, pos[0], pos[1], percent, heading_angle-90):
                    if isInside(pos[0], pos[1], pos[0]+(d * math.cos((heading_angle-delta)*math.pi/180)), pos[1]+(d * math.sin((heading_angle-delta)*math.pi/180)), pos[0]+(d * math.cos((heading_angle+delta)*math.pi/180)), pos[1]+(d * math.sin((heading_angle+delta)*math.pi/180)), c2_pos[0], c2_pos[1]): 
                        print("front obs")
                        left_speed = -max_speed * 0.1
                        right_speed = -max_speed * 0.1
                        bleft_motor.setVelocity(left_speed)
                        bright_motor.setVelocity(right_speed)
                        fleft_motor.setVelocity(left_speed)
                        fright_motor.setVelocity(right_speed)
                    
                    
            if help.distance_bw_points(pos, goal) < 0.15:
                left_speed = max_speed *0
                right_speed = max_speed *0
                bleft_motor.setVelocity(left_speed)
                bright_motor.setVelocity(right_speed)
                fleft_motor.setVelocity(left_speed)
                fright_motor.setVelocity(right_speed)
                region2 = False
                transition = False
                
            
        else:
            print("reached goal")
            # df = pd.DataFrame(rows_list) 
            # df.to_csv('path.csv')
            
                
    
        
            
                    
               # elif checkPoint(p_rad, pos[0], pos[1], percent, 45):
                    # left_speed = max_speed 
                    # right_speed = max_speed 
                    # bleft_motor.setVelocity(left_speed)
                    # bright_motor.setVelocity(right_speed)
                    # fleft_motor.setVelocity(left_speed)
                    # fright_motor.setVelocity(right_speed)
                
        
        
           
        """
 
        # print(epuck.getOrientation())
        if not is_obstacle:
            if eps < 0.01:
                left_speed = -max_speed *0.4
                right_speed = max_speed *0.4
                left_motor.setVelocity(left_speed)
                right_motor.setVelocity(right_speed)
                
            else:
                left_speed = max_speed *0.5
                right_speed = max_speed *0.5
                left_motor.setVelocity(left_speed)
                right_motor.setVelocity(right_speed)
                
        elif is_obstacle:
            # turn left
            if front_obs > 80:
                left_speed = -max_speed 
                right_speed = max_speed            
                # print('turn left')
            else:    
                # move forward
                if right_obs > 80 :
                    left_speed = max_speed *0.5
                    right_speed = max_speed *0.5
                    # print('move forward')
                
                # turn right
                else:
                    left_speed = max_speed 
                    right_speed = max_speed *0.125
                    # print('turn right')
                if right_corner > 80:
                    left_speed = max_speed *0.125
                    right_speed = max_speed                     
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)
            # print(start, pos, help.line_bw_points(start,goal))
            # print(help.perpendicular_dis(pos, help.line_bw_points(start,goal)))
            # print(help.distance_bw_points(pos, m_point) > 0.0001)
            if help.perpendicular_dis(pos, help.line_bw_points(start, pos)) < 0.01 and (start[1] < pos[1] < goal[1]): #abs(desire_angle - heading_angle)<90:                       
                is_obstacle = False
                m_point = pos
        
        # reached goal
        if help.distance_bw_points(pos, goal) < 0.01:
            left_speed = max_speed *0
            right_speed = max_speed *0
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)
            print("agent reached goal")
                # print('true')
        # print(is_obstacle)
        # print(pos, start, goal)
        # print((start[1] < pos[1] < goal[1]))
        # print(desire_angle,heading_angle, abs(desire_angle - heading_angle))
        # print(help.angel_line_horizontal((pos[0],pos[1]),goal))
        # print(help.get_heading_angle(epuck.getOrientation()))
        """
                   

if __name__ == "__main__":

    # my_robot = Robot()
    goal = (-0.25,1.4)
    robot = Supervisor()
    help = Helper()
    run_robot(robot, help, goal)