"""Bog2 controller."""


from controller import Robot, Supervisor
from helper import Helper
import sys
sys.path.append('/Users/pratik/Docs/sem2022-23/IR/q9/controllers/p_v')
import p_v

# position = (0,0)

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

# def get_pos():
    # global position
    # return position
    
    
    
    
def run_robot(robot, help, goal):
    
    global position
    
    time_step = 32
    max_speed = 6.24
    is_obstacle = False
    epuck = robot.getFromDef('e_puck')
    epuck_ev = robot.getFromDef('e_puck_ev')  
    start = epuck.getPosition()[:2]
    m_point = start
    
    # lets create motor instances
    left_motor = robot.getMotor('left wheel motor')
    right_motor = robot.getMotor('right wheel motor')
    
    # set motor position to inf and velocity to 0
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    ps = enable_proximity_sensors(robot, time_step)
    
    # get robot position
    
    # print(epuck)
    # pos = epuck.getField('translation')
  
    obstacle_seg = [((0, -0.5),(1, -0.5)), ((-1, 0),(0.5, 0)), ((-0.2, 0.5),(1, 0.5))]
    
    while robot.step(time_step) != -1:
        pos_evader = epuck_ev.getPosition()[:2]
        pos = epuck.getPosition()[:2]
        # p_v.run_robot.position = pos
        position = pos
        heading_angle = help.get_heading_angle(epuck.getOrientation())
        desire_angle = help.angel_line_horizontal((pos[0],pos[1]),goal)

        
                
        # for i in range(8):
            # print("ind: {}, Val: {}".format(i, ps[i].getValue()))
            
        front_obs = ps[0].getValue()
        right_obs = ps[2].getValue()
        right_corner = ps[1].getValue()
        
        if  front_obs > 80:
            is_obstacle = True 
            
 
        # print(epuck.getOrientation())
        if not is_obstacle:
            if heading_angle - desire_angle < 0.01:
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
            
        # found or not! 
        not_found = [True for _ in range(len(obstacle_seg))]
        for idx, obs_seg in enumerate(obstacle_seg):
            not_found[idx] = bool(help.check_intersect((pos, pos_evader), obs_seg))
            
        if not any(not_found):
            print("found the evader!! *_*")
            break
            
      
            break
    left_speed = -max_speed
    right_speed = max_speed
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
                # print('true')
        # print(is_obstacle)
        # print(pos, start, goal)
        # print((start[1] < pos[1] < goal[1]))
        # print(desire_angle,heading_angle, abs(desire_angle - heading_angle))
        # print(help.angel_line_horizontal((pos[0],pos[1]),goal))
        # print(help.get_heading_angle(epuck.getOrientation()))
        
        

                   

if __name__ == "__main__":

    # my_robot = Robot()
    goal = (-0.96,-0.96)
    robot = Supervisor()
    help = Helper()
    run_robot(robot, help, goal)