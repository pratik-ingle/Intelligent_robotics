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
A = [(-1,-1), (1,-1), (1,-0.5), (-1,-0.5)]
B = [(1,-0.5), (0.5,-0.5), (-1,-0.25), (0.5,-0.25)]
C = [(0.5,-0.5), (1,-0.5), (1,0.5), (0.5,0.5)]
D = [(-1,-0.25), (-0.5,-0.25), (-0.5,0.5), (-1,0.5)]   
E = [(-0.5,-0.25), (1,-1), (1,-0.5), (-1,-0.5)]
F = [(-1,-1), (1,-1), (1,-0.5), (-1,-0.5)] 
    
def run_robot(robot, help, goal):
    time_step = 32
    max_speed = 6.24
    is_obstacle = False
    epuck = robot.getFromDef('e_puck')
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
  
    
    while robot.step(time_step) != -1:
        
        pos = epuck.getPosition()[:2]
        
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
                # print('true')
        # print(is_obstacle)
        # print(pos, start, goal)
        # print((start[1] < pos[1] < goal[1]))
        # print(desire_angle,heading_angle, abs(desire_angle - heading_angle))
        # print(help.angel_line_horizontal((pos[0],pos[1]),goal))
        # print(help.get_heading_angle(epuck.getOrientation()))

                   

if __name__ == "__main__":

    # my_robot = Robot()
    goal = (0,0)
    robot = Supervisor()
    help = Helper()
    run_robot(robot, help, goal)