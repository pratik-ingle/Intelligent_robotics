"""Bog0 controller."""


from controller import Robot, Supervisor
from helper import Helper

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

    
    
def run_robot(robot, help, goal):
    time_step = 32
    max_speed =  0 #16.129
    is_obstacle = False
    epuck = robot.getFromDef('create')
    create1 = robot.getFromDef('create1')
    create2 = robot.getFromDef('create2')
    start = epuck.getPosition()[:2]
    m_point = start
    
    p_rad = 0.5 
    # lets create motor instances
    left_motor = robot.getMotor('left wheel motor')
    right_motor = robot.getMotor('right wheel motor')
    
    # set motor position to inf and velocity to 0
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    # ps = enable_proximity_sensors(robot, time_step)
    
    # get robot position
    
    # print(epuck)
    # pos = epuck.getField('translation')
  
    
    while robot.step(time_step) != -1:
        
        pos = epuck.getPosition()[:2]
        c1_pos = create1.getPosition()[:2]
        c2_pos = create2.getPosition()[:2]
        
        # print(pos, c1_pos, c2_pos)
        
        heading_angle = help.get_heading_angle(epuck.getOrientation())
        heading_angle1 = help.get_heading_angle(create1.getOrientation())
        heading_angle2 = help.get_heading_angle(create2.getOrientation())
        
        # print(heading_angle, heading_angle1, heading_angle2)
        # print(
        
        # if 
        desire_angle = help.angel_line_horizontal((pos[0],pos[1]),goal)
        
        # print(heading_angle, desire_angle)

        if heading_angle > desire_angle:
           eps = heading_angle - desire_angle
        else:
           eps = desire_angle - heading_angle
        
        # print(eps)
        # for i in range(8):
            # print("ind: {}, Val: {}".format(i, ps[i].getValue()))
            
        # front_obs = ps[0].getValue()
        # right_obs = ps[2].getValue()
        # right_corner = ps[1].getValue()
        
        # if  front_obs > 80:
            # is_obstacle = True 
            
        
            
        if eps > 1:
            if heading_angle < desire_angle:
                left_speed = -max_speed *0.1
                right_speed = max_speed *0.1
                left_motor.setVelocity(left_speed)
                right_motor.setVelocity(right_speed)
            else:
                left_speed = max_speed *0.1
                right_speed = -max_speed *0.1
                left_motor.setVelocity(left_speed)
                right_motor.setVelocity(right_speed)
                
        elif eps < 1:
            left_speed = max_speed 
            right_speed = max_speed 
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)
            
        
        if help.distance_bw_points(pos, goal) < 0.15:
                left_speed = max_speed *0
                right_speed = max_speed *0
                left_motor.setVelocity(left_speed)
                right_motor.setVelocity(right_speed)
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
    goal = (-0.2,1.45)
    robot = Supervisor()
    help = Helper()
    run_robot(robot, help, goal)