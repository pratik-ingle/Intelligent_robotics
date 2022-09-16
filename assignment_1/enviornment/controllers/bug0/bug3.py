
"""bug0 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot


def run_robot(robot):
    
    time_step = 32
    max_speed = 6.24
    
    counter = 0
    max_counter = 100
    
    state = ["follow_line", "obstacle_avo"]
    current_state = state[0]
    #motors
    left_motor = robot.getMotor('left wheel motor')
    right_motor = robot.getMotor('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    left_ir = robot.getDistanceSensor('ir1')   # get the distance sensors from the robot
    left_ir.enable(time_step)   # enable them with the time stamp
    right_ir = robot.getDistanceSensor('ir0')
    right_ir.enable(time_step)
    
    # enable ps sensors
    ps = []
    psNames = [
        'ps0', 'ps1', 'ps2', 'ps3',
        'ps4', 'ps5', 'ps6', 'ps7'
    ]
    for i in range(8):
        ps.append(robot.getDevice(psNames[i]))
        ps[i].enable(time_step)
        
    while robot.step(time_step) != -1:
        
        left_ir_value = left_ir.getValue()
        right_ir_value = right_ir.getValue()
        
        print("left: {} rigt: {}".format(left_ir_value, right_ir_value))
        
        for i in range(8):
            print("ind: {}, Val: {}".format(i, ps[i].getValue()))
            
            
            
        # detect obstacle 
        right_obs = ps[2].getValue() > 80.0 #ps[1].getValue() > 80.0 or ps[2].getValue() > 80.0 or ps[3].getValue() > 80.0
        left_obs = ps[5].getValue() > 80.0 #ps[4].getValue() > 80.0 or ps[5].getValue() > 80.0 or ps[6].getValue() > 80.0
        front_obs = ps[0].getValue() > 80.0  or ps[7].getValue() > 80.0

        #change direction
        # if left_obs:
            # left_speed  = max_speed
            # right_speed = -max_speed
        # elif right_obs:
            # left_speed  = -max_speed
            # right_speed = max_speed
            
        if front_obs:
            current_state = state[1]
            
        if current_state == "obstacle_avo": 
            
            if front_obs:
                left_speed = max_speed  
                right_speed = -max_speed  
            
            if left_obs and ps[6].getValue() > 80.0 and not front_obs:
                left_speed = max_speed  *0.15
                right_speed = max_speed  *0.25
            
            elif left_obs:
                left_speed = max_speed  *0.15
                right_speed = max_speed  *0.25
            
            elif not left_obs and not front_obs:
                left_speed = max_speed  *0.18
                right_speed = max_speed  *0.25    
            
            elif 6 < left_ir_value < 15 or 6 < right_ir_value < 15 and not ps[0].getValue() > 80.0 and counter > max_counter:
               current_state = state[0]
               counter = 0
               left_speed = max_speed  *0.25
            
            # else:
                 # left_speed = max_speed *0.25 /4
                 # right_speed = max_speed *0.25
               
            counter += 1
               
        elif current_state == "follow_line":
            left_speed = max_speed *0.25
            right_speed = max_speed *0.25
            
            if (left_ir_value > right_ir_value) and (6 < left_ir_value < 15):
                print("go left")
                left_speed = -max_speed *0.25
            elif (right_ir_value > left_ir_value) and (6 < right_ir_value < 15):
                print("go right")
                right_speed = -max_speed *0.25
               
 
        print('counter:'+ str(counter) +' Current state: ' + current_state)
        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
    
 
if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)



    
    