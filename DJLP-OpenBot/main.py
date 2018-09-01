# setup hardware-related variables
importing = "warnings"
try: import warnings; print("...imported",importing)
except: print("Failed to import!:",importing)

importing = "machine"
try: import machine; is_physical = True; print("...imported",importing) # used for hardware communication, sets a flag if using physical ESP32 with machine library
except: is_physical = False; print("Failed to import!:",importing) # sets flag false if failed to import machine library

importing = "colorama"
try: import colorama; colorama_present = True; print("...imported",importing)
except: colorama_present = False; print("Failed to import!:",importing)

importing = "dbi"
try: from dbi.dbi import dbi; dbi_present = True; print("...imported",importing)
except: dbi_present = False; print("Failed to import!:",importing)

importing = "main_functions.py"
try: from main_functions import *; print("...imported",importing)
except: print("Failed to import!:",importing)

importing = "main_classes.py"
try: from main_classes import *; print("...imported",importing)
except: print("Failed to import!:",importing)

importing = "json"
if dbi_present:
    try: import json; json_present = True; print("...imported",importing)
    except: json_present = False; print("Failed to import!:",importing)
    db = json.load(open('data/db.json'))

def state_message(state,true_message,false_message):
    if state:
        if colorama_present:
            print(colorama.Fore.LIGHTGREEN_EX +
                  colorama.Back.BLACK +
                  true_message +
                  colorama.Style.RESET_ALL)
        else:
            print(true_message)
    else:
        if colorama_present:
            print(colorama.Fore.LIGHTRED_EX +
                  colorama.Back.BLACK +
                  false_message +
                  colorama.Style.RESET_ALL);
        else:
            print(false_message)

state_message(is_physical,"...running on hardware","running as simulation")

# import required libraries/scripts
import time
        
# -- FUNCTION DEFINITIONS --
def valmap(value, in_min, in_max, out_min, out_max):
    return ((value - in_min) * (out_max - out_min) // (in_max - in_min) + out_min)

# -- CLASS DEFINITIONS --
class Led:
    def __init__(self,led_id,led_pin_num,led_is_physical):
        self.id = led_id
        self.get_pin_num = led_pin_num
        self.is_physical = led_is_physical
        if (self.is_physical): self.pin = machine.Pin(self.get_pin_num, machine.Pin.OUT)
    def state(self,is_active):
        if (self.is_physical):
            if (is_active):
                if (self.is_physical): self.pin.high()
                else: print("LED IS NOW:","HIGH")
            else:
                if (self.is_physical): self.pin.low()
                else: print("LED IS NOW:","LOW")
    def flash(self,duration):
        self.state(True)
        time.sleep(duration)
        self.state(False)
class Robot:
    def __init__(self,name):
        self.name = name
        self.position = None # home position
        self.joints = {}
        self.POSITIONS = {}
class Servo:
    def __init__(self,name,servo_pin_num,servo_freq,servo_step_time,servo_pos_home,servo_is_physical):
        self.name = name
        self.get_pin_num = servo_pin_num
        self.get_freq = servo_freq
        self.get_step_time = servo_step_time
        self.get_pos_home = servo_pos_home
        self.get_pos = self.get_pos_home
        self.get_duty = valmap(self.get_pos_home,0,180,40,115)
        self.is_physical = servo_is_physical
        if (self.is_physical): self.pin = machine.Pin(self.get_pin_num)
        if (self.is_physical): self.pwm = machine.PWM(self.pin,freq=self.get_freq)
    def set_pos(self,pos):
        duty = valmap(pos,0,180,40,115)
        if (self.is_physical): self.pwm.duty(int(duty))
        else: print("POS={:<3} (DUTY={:<3})".format(pos,duty))
        self.get_pos = pos
        self.get_duty = duty
class JointTarget:
    def __init__(self,joint,val_target):
        self.joint = joint
        self.val_target = val_target
class Joint:
    def __init__(self,name):
        self.name = name
        self.axes = {}
class Axis:
    def __init__(self,name,val_home,val_min,val_max,motor): # TODO: continue this
        self.name = name # name of axis (e.g 'x')
        self.home = val_home
        self.min = val_min
        self.max = val_max
        self.motor = motor
        self.val = val_home # make the initial value the home value
class TimeElapsedTracker:
    def __init__(self):
        self.script_started = time.time()
    def begin(self):
        self.start = time.time()
        self.elapsed = 0
    def stop(self):
        self.end = time.time()
        self.elapsed = self.end - self.start

# -- SERVOS -- define the physical motors so we can attach them to axes later
A_R_S = Servo("A_R_S",4,50,0.02,0,is_physical)  # always double these are right
B_R_S = Servo("B_R_S",5,50,0.02,0,is_physical)  # always double these are right
C_R_S = Servo("C_R_S",15,50,0.02,0,is_physical) # always double these are right

# -- BASE CLASSES -- create the primary robot class everything is built off of
ROBOT = Robot("ROBOT") # create a robot class
TARGET = Robot("TARGET") # create a class for targets

# create joint main_classes for the robot
for item in [ROBOT,TARGET]: # for the items robot and target
    item.joints['A'] = Joint('A') # create joint A (for all base main_classes)
    item.joints['B'] = Joint('B') # create joint B (for all base main_classes)
    item.joints['C'] = Joint('B') # create joint B (for all base main_classes)
    item.joints['A'].axes['rot'] = Axis('rot',90,0,180,A_R_S) # create axis for joint
    item.joints['B'].axes['rot'] = Axis('rot',90,0,180,B_R_S) # create axis for joint
    item.joints['C'].axes['rot'] = Axis('rot',90,0,180,C_R_S) # create axis for joint
# SABOTAGE EVERYTHING FOR TESTING PURPOSES
#TARGET.joints['A'].axes['rot'].motor.set_pos(40)

# test moving axes
# joint_a = ROBOT.joints['A'].axes['rot'].val
# joint_a = 0 # TODO: move into main loop as an event
# if dbi_present: dbi(db,2,"joint['a']",str(joint_a))
# joint_a = 180 # TODO: move into main loop as an event
# if dbi_present: dbi(db,2,"joint['a']",str(joint_a))

# move the robot to this position
#ROBOT.change_position(ROBOT.POSITIONS['home'])


# # DEV TESTING -------
if dbi_present: dbi(db,2,"name:",str(ROBOT.joints['A'].axes['rot'].name)) # get the name of the axis in joint
if dbi_present: dbi(db,2,"home:",str(ROBOT.joints['A'].axes['rot'].home)) # get the name of the axis in joint
if dbi_present: dbi(db,2,"min:",str(ROBOT.joints['A'].axes['rot'].min)) # get the name of the axis in joint
if dbi_present: dbi(db,2,"max:",str(ROBOT.joints['A'].axes['rot'].max)) # get the name of the axis in joint
if dbi_present: dbi(db,2,"val:",str(ROBOT.joints['A'].axes['rot'].val)) # get the name of the axis in joint
#  
# print("joints:",ROBOT.joints)
#  
# print("exiting prematurely..."); exit()
# # DEV TESTING -------

# essential variables
pause_between_targets = 1 # pause between moving to another target (in seconds)

# for target in targets: # print information for all targets
#     print("ID:[{:>3}] POS:[{:>3}] NAME:['{:<32}']".format(targets[target].id,targets[target].pos,targets[target].name))

# BEFORE LOOP MESSAGE/S #
#print("entering primary loop in 3 seconds...")
#time.sleep(3)

def calc_increment(move_time,joint,axis):
    step_count = move_time//ROBOT.joints[joint].axes[axis].motor.get_step_time # total number of steps taken from target A to target B
    target_dist = abs(ROBOT.joints[joint].axes[axis].motor.get_pos - TARGET.joints[joint].axes[axis].motor.get_pos) # distance from the target relative to current position
    print(colorama.Fore.LIGHTRED_EX + target_dist + colorama.Style.RESET_ALL)
    step_dist = target_dist / step_count # distance per step
    return({'step_count': step_count, 'target_dist': target_dist, 'step_dist': step_dist})

def sync_motors():
    for item in [ROBOT,TARGET]:
        for joint in item.joints:
            for axis in item.joints[joint].axes:
                robot_motor = ROBOT.joints[joint].axes[axis].motor
                target_motor = TARGET.joints[joint].axes[axis].motor
                calc = calc_increment(2,joint,axis)
                move_increment = calc['step_dist']
                if robot_motor.get_pos is target_motor.get_pos: print("motor is aligned!")
                elif robot_motor.get_pos < target_motor.get_pos:
                    robot_motor.set_pos(robot_motor.get_pos + move_increment) #increase by increment amount
                elif robot_motor.get_pos > target_motor.get_pos:
                    robot_motor.set_pos(robot_motor.get_pos - move_increment) #decrease by increment amount
                else: print("[!] something went wrong while setting motors new positions [!]")
                
                print("ACTUAL:[{}] TARGET:[{}]".format(ROBOT.joints[joint].axes[axis].motor.get_pos,TARGET.joints[joint].axes[axis].motor.get_pos))
                
def sublist_gen(li, cols=2):
    start = 0
    for i in range(cols):
        stop = start + len(li[i::cols])
        yield li[start:stop]
        start = stop

# function to update values
def update(updatetrack):
    updatetrack.begin()
    comparison_list = []
    for joint in ROBOT.joints and TARGET.joints:
        for item in [ROBOT,TARGET]:
            for key in ['rot']:
                comparison_list.append(item.joints[joint].axes[key].val)
                
            
    comparison_list = sublist_gen(comparison_list,2)
    nonmatches = 0
    for item in comparison_list:
        if item[0] == item[1]: pass #print("all is well :)")
        else: nonmatches += 1; print("SOMETHING DOESN'T MATCH >:c")
    if nonmatches > 0:
        print("ROBOT DOESN'T MATCH TARGET!!! D:")
        for joint in ROBOT.joints:
            for axis in ROBOT.joints[joint].axes:
                print("AT:[{}] TO:[{}]".format(ROBOT.joints[joint].axes[axis].motor.get_pos,
                                               TARGET.joints[joint].axes[axis].motor.get_pos))
        sync_motors() # adjust position of motors
        # move servos to correct positions (use a function)
    #print("repeating main loop")
    if dbi_present and db['verbosity_level'] == 3:
        print("[VERB=3]: sleeping for 1 second"); time.sleep(1)
    updatetrack.stop()
#    print("update finished in {} seconds".format(updatetrack.elapsed))
    
#---primary loop---#
def main_loop(maintracker,exit_condition):
    while exit_condition == False:
        maintracker.begin()
        time_passed = int(time.time() - maintracker.script_started)
        print("time passed:",time_passed)
        if time_passed > 12: exit_condition = True
        elif time_passed >= 6 and time_passed < 9: TARGET.joints['A'].axes['rot'].motor.set_pos(20); TARGET.joints['B'].axes['rot'].motor.set_pos(30); TARGET.joints['C'].axes['rot'].motor.set_pos(50)
        elif time_passed > 3 and time_passed < 6: TARGET.joints['A'].axes['rot'].motor.set_pos(120); TARGET.joints['B'].axes['rot'].motor.set_pos(160); TARGET.joints['C'].axes['rot'].motor.set_pos(150)
        else: TARGET.joints['A'].axes['rot'].motor.set_pos(90); TARGET.joints['B'].axes['rot'].motor.set_pos(90); TARGET.joints['C'].axes['rot'].motor.set_pos(90)
        timetrack = TimeElapsedTracker() # initialise time tracker
        if type(exit_condition) == bool:
#            print("exit_condition is bool!")
            update(timetrack) # update servos (inherit time tracker)
        elif type(exit_condition) == int:
#            print("exit_condition is int!")
            i = 1
            print("loop:",i)
            update(timetrack)
            i += 1
        else: print("exit_condition is neither bool or int!!!")
        maintracker.stop()
#         print("maintracker:",maintracker.elapsed)
        time.sleep(0.01)
    
###### START 'ER UP!!! ######
maintracker = TimeElapsedTracker()
main_loop(maintracker,False) # use a bool value or an int value, used as a termination condition
