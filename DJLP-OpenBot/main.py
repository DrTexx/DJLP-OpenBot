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
def println(string):
    print(string,end=" ")

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
            ROBOT.joints[joint].movement.calc()
            print("{}:DIST_FROM:[{:>4}]:STEP_TIME:[{:>4}]:STEP_COUNT:[{:>4}] ".format(ROBOT.joints[joint].movement.get_get_name,move_info['target_dist'],move_info['step_time'],move_info['step_count']),end="")
            if move_info['target_dist'] is not 0:
                print("target dist is not 0")
                if move_info['target_dist'] < 0:
                    step_pos = joint_pos - move_info['step_dist']
                elif move_info['target_dist'] > 0:
                    step_pos = joint_pos + move_info['step_dist']
                print(step_pos)
                ROBOT.joints[joint].get_hardware.set_pos(int(step_pos)) # add an amount calculated
            else:
                print("target dist = 0")
            
            time.sleep(ROBOT.joints[joint].get_hardware.get_step_time)
        print()
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
        class NewMovement:
            def __init__(self,time_tracker,move_duration,before_pos,after_pos,hardware):
                #TODO: complete this
                self.get_before_pos = before_pos
                self.get_during_pos = before_pos
                self.get_after_pos = after_pos
                self.get_dist_from_pos = after_pos-before_pos #TODO: check this is correct
                self.get_hardware = hardware
                self.get_before_time = time.time()
                self.get_after_time = time.time() + move_duration
                # CALCULATIONS (run once per movement created)
                println("calculate duration...")
                self.get_duration_time = self.get_after_time - self.get_before_time # during movement: no change
                println(self.get_duration_time)
                println("calculate total step count...")
                # total number of ticks required to reach target by x seconds (dependent on servo's step time)
                self.get_total_steps = self.get_duration_time//self.get_hardware.get_step_time # total number of steps taken from target A to target B
                println(self.get_total_steps)
                # set how many step are left to take
                println("setting steps remaining...")
                self.get_steps_remaining = self.get_total_steps # decrease this every tick! (CHANGES!)
                println(self.get_steps_remaining)
                # used for outputting to terminal, shouldn't be used in loops!!!
                println("calculate target distance...")
                self.get_target_dist = abs(self.get_hardware.get_pos - self.get_after_pos) # distance from the target relative to current position
                println(self.get_target_dist)
                println("calculate distance per step...")
                self.get_dist_per_step = self.get_target_dist / self.get_total_steps # distance per step
                println(self.get_dist_per_step)
                # time between each step should be servos step time, so motion as smooth as possible
                self.time_per_step = self.get_hardware.get_step_time
                return({'move_duration': move_duration,'step_count': step_count, 'target_dist': target_dist, 'step_dist': step_dist, 'step_time': step_time})
            def calc(self):
                pass # for recalculation during movement, perhaps better to set a reducing attribute of self in main loop or in run?
            def run(self):
                if self.get_steps_taken == self.get_total_steps:
                    pass # don't do stuff
                    pass # delete the movement attribute
                # more stuff
                # delete the movement attribute
                self.get_step_ticks += 1 # add a step to the counter (for determining when to stop)
                return("step taken")
        def set_pos(self,pos):
            self.get_pos = pos
            self.set_duty(valmap(self.get_pos,0,180,40,115))
        def set_duty(self,duty):
            print("SERVO:[{:<5}],POS[{:<3}],DUTY[{:<3}]".format(self.get_name,self.get_pos,self.get_duty),end="")
            if (self.is_physical): self.pwm.duty(int(self.get_duty))
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
    
    first = ROBOT.joints['A'].get_hardware.get_pos
    last = 10
    ROBOT.joints['A'].movement = ROBOT.joints['A'].get_hardware.NewMovement(time_tracker,2,first,last,ROBOT.joints['A'].get_hardware)
    ROBOT.joints['A'].movement.run()
    #REQUIRES A CALC (add within movement object)
    ROBOT.joints['A'].movement.calc()

    
    ROBOT.sync(time_tracker,time.time() + 2) # update all hardwares such as servos (pass on time_tracker)
    for joint in ROBOT.joints:
        print("SERVO:NOW:[{:>3}],SERVO:NEW:[{:>3}]".format(ROBOT.joints[joint].get_hardware.get_pos,
                                               ROBOT.joints[joint].get_hardware.get_new_pos))
    
###### START 'ER UP!!! ######
time_tracker = TimeElapsedTracker()
time_tracker.begin()
exit_loop = False
while not exit_loop:
    main(time_tracker) # use a bool value or an int value, used as a termination condition
time_tracker.stop
print("goodbye!")