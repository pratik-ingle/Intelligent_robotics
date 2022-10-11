"""Bog0 controller."""


from controller import Robot, Supervisor
from helper import Helper
import numpy as np
import scipy.spatial.distance as distance
#############


class pidcontroller:
    def __init__ (self, kp, ki, kd):
        self.kp = np.array(kp)
        self.ki = np.array(ki)
        self.kd = np.array(kd)
        self.cumm_error = np.array([0, 0])
        self.prev_error = np.array([0,0])
        
        
    def control(self, error, *agrs):
        error = np.array(error, dtype = float)
        self.cumm_error = np.add (self.cumm_error, error)
        dE = np.subtract(self.prev_error , error)
        self.prev_error = error
        return np.multiply(self.kp, error) + np.multiply(self.ki, self.cumm_error) + np.multiply(self.kd, dE)
        

###########

class lawn_mover():      
    def __init__(self,robot, help, rebb_graph):
        self.time_step = 32
        self.max_speed = 6.24
        self.is_obstacle = False
        self.epuck = robot.getFromDef('e_puck')
        self.start = self.epuck.getPosition()[:2]
        self.rot = self.epuck.getField ("rotation")
        self.trans = self.epuck.getField("translation")
        self.m_point = self.start
        self.sensing_radius = 0.2
        self.reached_goal = False
        
        
        
        self.end = False
        self.count_lane = 1
        self.reg = 0
        # lets create motor instances
        self.left_motor = robot.getMotor('left wheel motor')
        self.right_motor = robot.getMotor('right wheel motor')
        
        # set motor position to inf and velocity to 0
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        self.ps = self.enable_proximity_sensors(robot, self.time_step)
        
        # get robot position
        
        # print(epuck)
        # pos = epuck.getField('translation')
        heading_correction = -1.5708 + 3.14
        pid2d = pidcontroller([2, 100], [0, 10], [0, 0])
    
        heading_correction = -1.5708 + 3.14
        
        robot_pos = np.array(self.trans.getSFVec3f()[0:3:2])
        orientation = np.array(self.rot.getSFRotation()[3] + heading_correction)
        robot_heading = self.boundingangle(orientation)
                
        while robot.step(self.time_step) != -1:
            
            self.region =  rebb_graph[self.reg]
            self.goal = (self.region[0][0]+0.09, self.region[0][1]+0.09)  
            self.pos = self.epuck.getPosition()[:2]
            
            # for i in range(8):
                # print("ind: {}, Val: {}".format(i, ps[i].getValue()))
            # for region in rebb_graph:
            if not self.reached_goal: 
                
                heading_angle = help.get_heading_angle(self.epuck.getOrientation())
                desire_angle = help.angel_line_horizontal((self.pos[0],self.pos[1]),self.goal) 
                
                front_obs = self.ps[0].getValue()
                right_obs = self.ps[2].getValue()
                right_corner = self.ps[1].getValue()
                
                
                if  front_obs > 80:
                    is_obstacle = True 
                    
         
                # print(epuck.getOrientation())
                if not self.is_obstacle:
                    if heading_angle - desire_angle < 0.01:
                        left_speed = -self.max_speed *0.4
                        right_speed = self.max_speed *0.4
                        self.left_motor.setVelocity(left_speed)
                        self.right_motor.setVelocity(right_speed)
                        
                    else:
                        left_speed = self.max_speed *0.5
                        right_speed = self.max_speed *0.5
                        self.left_motor.setVelocity(left_speed)
                        self.right_motor.setVelocity(right_speed)
                        
                elif self.is_obstacle:
                    # turn left
                    if front_obs > 80:
                        left_speed = -self.max_speed 
                        right_speed = self.max_speed            
                        # print('turn left')
                    else:    
                        # move forward
                        if right_obs > 80 :
                            left_speed = self.max_speed *0.5
                            right_speed = self.max_speed *0.5
                            # print('move forward')
                        
                        # turn right
                        else:
                            left_speed = self.max_speed 
                            right_speed = self.max_speed *0.125
                            # print('turn right')
                        if right_corner > 80:
                            left_speed = self.max_speed *0.125
                            right_speed = self.max_speed                     
                    self.left_motor.setVelocity(left_speed)
                    self.right_motor.setVelocity(right_speed)
                    # print(start, pos, help.line_bw_points(start,goal))
                    # print(help.perpendicular_dis(pos, help.line_bw_points(start,goal)))
                    # print(help.distance_bw_points(pos, m_point) > 0.0001)
                    if help.perpendicular_dis(pos, help.line_bw_points(start, pos)) < 0.01 and (start[1] < pos[1] < goal[1]): #abs(desire_angle - heading_angle)<90:                       
                        is_obstacle = False
                        m_point = pos
                
                # reached goal
                if help.distance_bw_points(self.pos, self.goal) < 0.01:
                    left_speed = self.max_speed *0
                    right_speed = self.max_speed *0
                    self.left_motor.setVelocity(left_speed)
                    self.right_motor.setVelocity(right_speed)
                    self.reached_goal = True
                    print("agent reached goal")
                    
            else:
                multi_goals = self.multi_gtg()
                # point =  help.perpendicular_point(self.region[1], self.region[2], self.pos)
                # print(point)
                # if not self.end:
                heading_angle = help.get_heading_angle_180(self.epuck.getOrientation())
                desire_angle = help.angel_line_horizontal_180((self.pos[0],self.pos[1]),multi_goals[self.count_lane]) 
                # self.epuck 
                # print(help.distance_bw_points(self.pos, multi_goals[self.count_lane]))
                # if heading_angle > desire_angle:
                    # angle_diff = (heading_angle - desire_angle)
                # else:
                    # angle_diff = (desire_angle - heading_angle)
                    
                print("angle:", desire_angle,heading_angle,(heading_angle - desire_angle), (desire_angle - heading_angle))
   #########################
                # D_heading = np.arctan2(multi_goals[self.count_lane][1] - self.pos[1], multi_goals[self.count_lane][0] - self.pos[0])
                # desired_heading = self.boundingangle(D_heading)
                
                # orientation = np.array(self.rot.getSFRotation()[3] + heading_correction)
                # robot_heading = orientation
                
                # pos_error = distance.euclidean(multi_goals[self.count_lane], self.pos)
                # angular_error = self.boundingangle( np.array (desired_heading - robot_heading))
                
                # if (pos_error < 0.02):
                    # leftmotor.setVelocity (0)
                    # rightmotor.setVelocity (0)
                    # continue;       
    
                # u = pid2d.control ([pos_error, angular_error]) 
                # [vr, vl] = self.burgermotordriver (u[0], u[1]) 
                
                # self.left_motor.setVelocity (vl)
                # self.right_motor.setVelocity (vr)
                # if help.distance_bw_points(self.pos, multi_goals[self.count_lane]) < 0.05:
                    # print('reached')
                    # self.count_lane += 1
                # if help.distance_bw_points(self.pos, multi_goals[-1]) < 0.01:
                    # self.reg += 1
                    # print('end')
                # print("angle:", desired_heading,robot_heading,(desired_heading - robot_heading))
   #######################3
                if (heading_angle - desire_angle) < 0.1:
                    left_speed = -self.max_speed *0.1
                    right_speed = self.max_speed *0.1
                    self.left_motor.setVelocity(left_speed)
                    self.right_motor.setVelocity(right_speed)
                    print('turnung left')
                    
                # elif (desire_angle - heading_angle) > 0.2:
                    # left_speed = self.max_speed *0.1
                    # right_speed = -self.max_speed *0.1
                    # self.left_motor.setVelocity(left_speed)
                    # self.right_motor.setVelocity(right_speed)
                    # print('turnung right')
                        
                else :
                    left_speed = self.max_speed *0.5
                    right_speed = self.max_speed *0.5
                    self.left_motor.setVelocity(left_speed)
                    self.right_motor.setVelocity(right_speed)
                    if help.distance_bw_points(self.pos, multi_goals[self.count_lane]) < 0.05:
                        print('reached')
                        self.count_lane += 1
                    if help.distance_bw_points(self.pos, multi_goals[-1]) < 0.01:
                        self.reg += 1
                        print('end')
                        self.end = True
                    print('moveing')
                    print("pos:", self.pos,"goal:", multi_goals[self.count_lane], "distance:", help.distance_bw_points(self.pos,multi_goals[self.count_lane]))
                    print(help.distance_bw_points(self.pos,multi_goals[self.count_lane]))
                  
                print("pos:", self.pos,"goal:", multi_goals[self.count_lane], "lane:", (self.count_lane))
                        # print('true')
                # print(is_obstacle)
                # print(pos, start, goal)
                # print((start[1] < pos[1] < goal[1]))
                # print(desire_angle,heading_angle, abs(desire_angle - heading_angle))
                # print(help.angel_line_horizontal((pos[0],pos[1]),goal))
                # print(help.get_heading_angle(epuck.getOrientation()))
            # print(region)
            # print(goal,pos, reached_goal)
            # print(help.distance_bw_points(pos,goal))
            # print(desire_angle,heading_angle, abs(desire_angle - heading_angle))
            # point = help.perpendicular_point(region[1], region[2], pos)
            # print(point)   
    def enable_proximity_sensors(self,robot, time_step):
        # enable ps sensors
        self.ps = []
        self.psNames = [
            'ps0', 'ps1', 'ps2', 'ps3',
            'ps4', 'ps5', 'ps6', 'ps7'
             ]
        for i in range(8):
            self.ps.append(robot.getDevice(self.psNames[i]))
            self.ps[i].enable(time_step)
            
        return self.ps 
    
    def multi_gtg(self):
        region = self.region
        # print(region)
        multi_goals = []
        # point =  help.perpendicular_point(region[1], region[2], self.pos)
        # multi_goals.append(point)
        # multi_goals.append((point[0], point[1]))
        
        line1 =  list(help.getEquidistantPoints((region[0][0] + 0.09, region[0][1] + 0.09) , (region[3][0] + 0.09, region[3][1] - 0.09), int(help.distance_bw_points(region[0], region[3])/self.sensing_radius)))                              
        line2 =  list(help.getEquidistantPoints((region[1][0] - 0.09, region[1][1] + 0.09) , (region[2][0] - 0.09, region[2][1] - 0.09), int(help.distance_bw_points(region[1], region[2])/self.sensing_radius)))
        
        # print(line1)
        # print(line2)
        multi_goals.append(line1[0])
        # multi_goals.append(line2[0])
        # multi_goals.append(line2[1])
        for i in range(1, len(line1)):
            if i%2 == 0:
                multi_goals.append(line1[i-1])
                multi_goals.append(line1[i])
            else:
                multi_goals.append(line2[i-1])
                multi_goals.append(line2[i])
        # print(multi_goals)
        return multi_goals
    
    def boundingangle(self,x):
        while (x > np.pi):
            x -= 2 * np.pi
            
        while (x < -np.pi):
            x += 2 * np.pi
        return x
    
    def burgermotordriver(self,v, w):
        L = 0.16
        R = 0.066
        def velocity(vel):
            return max(-1*self.max_speed, min(vel, self.max_speed))
        
        vr = (2.*v + w*L) / (2.*R)
        vl = (2.*v - w*L) / (2.*R)
        vr = velocity(vr)
        vl = velocity(vl)
        return [vr, vl]
           
if __name__ == "__main__":

    # my_robot = Robot()
    # goal = (0,0)
    A = [(-1,-1), (1,-1), (1,-0.5), (-1,-0.5)]
    B = [(1,-0.5), (0.5,-0.5), (-1,-0.25), (0.5,-0.25)]
    C = [(0.5,-0.5), (1,-0.5), (1,0.5), (0.5,0.5)]
    D = [(-1,-0.25), (-0.5,-0.25), (-0.5,0.5), (-1,0.5)]   
    E = [(-0.5,-0.25), (0.5,-0.25), (0.5,0.5), (-0.5,0.5)]
    F = [(-1,0.5), (1,0.5), (1,1), (-1,1)]
    
    
    # def lawn_mover(region,sensing_radius):
    rebb_graph = [A, B, C, D, E, F]   
    robot = Supervisor()
    help = Helper()
    lawn_mover(robot, help, rebb_graph)