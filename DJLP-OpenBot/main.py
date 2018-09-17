''' -- IMPORTS -- '''
import time
from DJLP_OpenBot import TimeElapsedTracker,Robot,Hardware,DebugTools

print("~~~~~~#################~~~~~~")
hardware = Hardware() # create hardware object
hardware.load_servos()
ROBOT = Robot( # create a new robot with the following properties
    "ROBOT", # robot's name
    TimeElapsedTracker(), # time tracker instance
    joints={ # define robots joints
        "A": Robot.Joint("Axis A",hardware.servos['A']), # define joint A
        "B": Robot.Joint("Axis B",hardware.servos['B']), # define joint B
        "C": Robot.Joint("Axis C",hardware.servos['C']), # define joint C
        "D": Robot.Joint("Axis D",hardware.servos['D'])  # define joint D
    }
)

DebugTools.Printing.object(ROBOT) # print information about the robot object

#ROBOT_joints = {}
#for key in ['A','B','C','D']:
#    ROBOT_joints[key] = Robot.Joint(key,hardware.servos[key])
#DebugTools.Printing.object(hardware)
#print("hardware.servos:",hardware.servos)
#print("joints:",joints)

'''
print("THIS SHOULD ALWAYS BE THE FIRST THING TO PRINT!!!")
loop_led.value(1) # enable loop led
ROBOT.tt.begin() # start tracking time
end_loop = False # make loop ending condition False
while end_loop is not True: main() # loop main() until end_loop is True
toggle_led.value(0) # disable toggle led
loop_led.value(0) # disable lood led
ROBOT.tt.stop # stop tracking time

print("FINAL ROBOT JOINT POSITIONS:")
print_servo_pos_data(all_servos,ROBOT)

ROBOT.update_json(servos_json_filename) # update json file with current data
print("goodbye!")


input("waiting...")
'''