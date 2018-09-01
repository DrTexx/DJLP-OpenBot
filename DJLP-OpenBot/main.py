# import required libraries/scripts
import time
import _thread
try: import machine; is_physical = True
except: is_physical = False
try: import dbi; dbi_present = True
except: dbi_present = False
if dbi_present: import json
if dbi_present: db = json.load(open('data/db.json'))

# -- FUNCTION DEFINITIONS --
def valmap(value, in_min, in_max, out_min, out_max):
    return ((value - in_min) * (out_max - out_min) // (in_max - in_min) + out_min)
def state_message(state,true_message,false_message):
    if state: print(true_message)
    else: print(false_message)
def sublist_gen(li, cols=2):
    start = 0
    for i in range(cols):
        stop = start + len(li[i::cols])
        yield li[start:stop]
        start = stop

# -- CLASS DEFINITIONS --
class Robot:
    def __init__(self,name):
        self.get_name = name
        self.joints = {}
        self.positions = {}
    class NewJoint:
        def __init__(self,name,home,min,max,hardware):
            self.get_name = name
            self.get_home = home
            self.get_pos = home
            self.get_new_pos = home
            self.get_hardware = hardware
    class NewPosition:
        def __init__(self,joints):
            for joint in joints:
                pass # TODO: make this work
    def sync(self,time_tracker,move_duration):
        # TODO: make this work
        print("syncing...",end=" ")
        for joint in ROBOT.joints:
            joint_pos = ROBOT.joints[joint].get_pos
            move_info = self.movement_calc(ROBOT.joints[joint],move_duration)
            print("{}:DIST_FROM:[{:>4}] ".format(ROBOT.joints[joint].get_name,move_info['target_dist']),end="")
            if move_info['target_dist'] is not 0:
                if move_info['target_dist'] < 0:
                    step_pos = jointpos - move_info['step_dist']
                elif move_info['target_dist'] > 0:
                    step_pos = jointpos + move_info['step_dist']
                ROBOT.joints[joint].get_hardware.set_pos(int(step_pos)) # add an amount calculated
            
            time.sleep(ROBOT.joints[joint].get_hardware.get_step_time)
        print()
    def movement_calc(self,joint_obj,reach_by):
        move_duration = reach_by - time.time()
        step_count = move_duration//joint_obj.get_hardware.get_step_time # total number of steps taken from target A to target B
        target_dist = abs(joint_obj.get_pos - joint_obj.get_new_pos) # distance from the target relative to current position
        step_dist = target_dist / step_count # distance per step
        return({'step_count': step_count, 'target_dist': target_dist, 'step_dist': step_dist})
class Hardware:
    def __init__(self,is_physical):
        state_message(is_physical,"...running on hardware","running as simulation")
        self.servos = {}
        self.leds = {}
    class NewServo:
        def __init__(self,name,pin_num,freq,step_time,pos_home_min_max,is_physical):
            self.get_name = name
            self.get_pin_num = pin_num
            self.get_freq = freq
            self.get_step_time = step_time
            if type(pos_home_min_max) is not list:
                raise TypeError
            self.get_home_pos = pos_home_min_max[0]
            self.get_new_pos = pos_home_min_max[0]
            self.get_pos = pos_home_min_max[0]
            self.get_pos_min = pos_home_min_max[1]
            self.get_pos_max = pos_home_min_max[2]
            self.get_duty = valmap(self.get_pos,0,180,40,115)
            self.is_physical = is_physical
            if (self.is_physical):
                self.pin = machine.Pin(self.get_pin_num)
                self.pwm = machine.PWM(self.pin,freq=self.get_freq)
        def set_pos(self,pos):
            self.get_pos = pos
            self.set_duty(valmap(self.get_pos,0,180,40,115))
        def set_duty(self,duty):
            print("SERVO:[{:<5}],POS[{:<3}],DUTY[{:<3}]".format(self.get_name,self.get_pos,self.get_duty),end="")
            if (self.is_physical): self.pwm.duty(int(self.get_duty))
        def set_new_pos(self,new_pos):
            if new_pos < self.get_pos_max and new_pos > self.get_pos_min:
                self.get_new_pos = new_pos
    class NewLed:
        def __init__(self,name,pin_num,initial_state,is_physical):
            self.get_name = name
            self.get_pin_num = pin_num
            self.get_initial_state = initial_state
            self.get_state = initial_state
            self.is_physical = is_physical
            if (self.is_physical):
                self.get_pin_object = machine.Pin(self.get_pin_num, machine.Pin.OUT)
        def set_state(self,state):
            self.get_state = state
            if self.is_physical:
                if state: self.pin_object.high()
                else: self.pin_object.low()
            print("{}.state = {}".format(self.name,self.get_state))
class TimeElapsedTracker:
    def __init__(self):
        self.script_started = time.time()
    def begin(self):
        self.start = time.time()
        self.elapsed = 0
    def check(self):
        self.elapsed = time.time() - self.start
        return(self.elapsed)
    def stop(self):
        self.end = time.time()
        self.elapsed = self.end - self.start
        return(self.elapsed)

# ---- MAIN SETUP ----

# define physical hardware
hardware = Hardware(is_physical)
hardware.servos['A'] = hardware.NewServo("A",4,50,0.02,[0,0,180],is_physical) # always double these are right
hardware.servos['B'] = hardware.NewServo("B",5,50,0.02,[0,0,180],is_physical) # always double these are right
hardware.servos['C'] = hardware.NewServo("C",15,50,0.02,[0,0,180],is_physical) # always double these are right

# -- BASE CLASSES -- create the primary robot class everything is built off of
ROBOT = Robot("ROBOT") # create a robot class

ROBOT.joints['A'] = ROBOT.NewJoint('A',90,0,180,hardware.servos['A']) # create joint A (for all base main_classes)
ROBOT.joints['B'] = ROBOT.NewJoint('B',90,0,180,hardware.servos['B']) # create joint B (for all base main_classes)
ROBOT.joints['C'] = ROBOT.NewJoint('C',90,0,180,hardware.servos['C']) # create joint C (for all base main_classes)

ROBOT.joints['A'].get_hardware.set_new_pos(40)

# SCRAPS
pause_between_targets = 1 # pause between moving to another target (in seconds)
# SCRAPS

# function to update values
# if item[0] == item[1]: pass #print("all is well :)")
# else: nonmatches += 1; print("SOMETHING DOESN'T MATCH >:c")
# if nonmatches > 0:
#     print("ROBOT DOESN'T MATCH IDOL!!! D:")
#     sync_motors() # adjust position of motors
#     print("update finished")

#---primary loop---#
def main(time_tracker):
    global exit_loop
    print("[{:<4}] ".format(int(time_tracker.elapsed)),end="")
    time_tracker.check()
    if time_tracker.elapsed > 10: exit_loop = True
    ROBOT.sync(time_tracker,time.time() + 2) # update all hardwares such as servos (pass on time_tracker)
    
###### START 'ER UP!!! ######
time_tracker = TimeElapsedTracker()
time_tracker.begin()
exit_loop = False
while not exit_loop:
    main(time_tracker) # use a bool value or an int value, used as a termination condition
time_tracker.stop
print("goodbye!")