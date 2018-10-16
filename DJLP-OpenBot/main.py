''' -- IMPORTS -- '''
import time
import json
from DJLP_OpenBot import TimeElapsedTracker,Robot,Hardware,DebugTools,ThreadManager
from jmove import jmove
from checks import robot_checks

print("~~~~~~#################~~~~~~")
''' -- SETUP ROBOT -- '''
hardware = Hardware() # create hardware object
hardware.buttons = {"b1": Hardware().Button(12)}
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
robot_list = {'ROBOT': ROBOT}

DebugTools.Printing().object(ROBOT) # print information about the robot object

''' -- SETUP TIMELINE -- '''
timeline_name = 'timeline.json'
timeline = json.load(open(timeline_name))
for key in range(len(timeline)):
    print(timeline[key])

''' -- PRIMARY EXECUTIONS -- '''
#ROBOT.start_sync(delay=1,interval=0.0001) # start a new movement sync thread

#timeline.start()

sync_man = ThreadManager(ROBOT, # parent object
                         ROBOT.Sync) # function in parent object
sync_man.start()

result = robot_checks(robot_list,[ROBOT])
print("result of robot checks:",result)
jmove(ROBOT,'A',50,'B',70)

#time.sleep(3)
#ROBOT.set_joints({"B": 90, "C": 90, "D": 90})
#ROBOT.set_joints({"B": 90})
#time.sleep(3)
#ROBOT.home()
#time.sleep(3)

#sync_man.end()

print("script ending")

'''
time.sleep(2)
ROBOT.set_joints({"A": 70,
                  "B": 70,
                  "C": 70,
                  "D": 70})
time.sleep(2)
ROBOT.set_joints({"A": 120,
                  "B": 120,
                  "C": 120,
                  "D": 120})
time.sleep(2)
ROBOT.home(ROBOT.joints)
time.sleep(2)
ROBOT.end_sync()
'''


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