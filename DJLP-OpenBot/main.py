''' -- IMPORTS -- '''
import time
import _thread
try: import machine; is_physical = True
except: is_physical = False
try: import dbi; dbi_present = True
except: dbi_present = False
if dbi_present: import json
if dbi_present: db = json.load(open('data/db.json'))

''' -- FUNCTION DEFINITIONS -- '''
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
def println(*args,brak=False):
    output_message = []
    if brak: output_message.append("[")
    for message in args:
        message = str(message)
        output_message.append(message)
    if brak: output_message.append("]")
    
    total_message = ''.join(output_message)
    print(total_message,end=" ")
    
''' -- ROBOT CLASS DEFINITIONS -- '''
class Robot:
    def __init__(self,name):
        self.name = name
        self.joints = {}
        self.positions = {}
    class Joint:
        def __init__(self,name,hardware):
            self.get_name = name
            self.get_hardware = hardware
    class NewPosition:
        def __init__(self,joints):
            for joint in joints:
                pass # TODO: make this work
    def sync(self,time_tracker,move_duration):
        # TODO: make this work
        print("syncing...",end=" ")
        for joint in ROBOT.joints:
            if hasattr(ROBOT.joints[joint], 'movement'): # if this joint has a movement attribute
                ROBOT.joints[joint].movement.run() # this tick, increment servos a bit to target
        print()

''' -- HARDWARE CLASS DEFINITIONS -- '''
class Hardware:
    def __init__(self,is_physical):
        state_message(is_physical,"...running on hardware","running as simulation")
        self.is_physical = is_physical
        self.servos = {}
        self.leds = {}
    class Servo:
        def __init__(self,name,pin_num,freq,step_time,pos_home_min_max):
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
            if (hardware.is_physical):
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
                self.get_time_remaining = self.get_after_time - self.get_before_time
                # CALCULATIONS (run once per movement created)
                self.get_duration_time = self.get_after_time - self.get_before_time # during movement: no change
                println("duration: ",self.get_duration_time,brak=True)
                # total number of ticks required to reach target by x seconds (dependent on servo's step time)
                self.get_total_steps = self.get_duration_time//self.get_hardware.get_step_time # total number of steps taken from target A to target B
                println("total step count: ",self.get_total_steps,brak=True)
                # set how many step are left to take
                self.get_steps_remaining = self.get_total_steps # decrease this every tick! (CHANGES!)
                println("set steps remaining: ",self.get_steps_remaining,brak=True)
                # used for outputting to terminal, shouldn't be used in loops!!!
                self.get_target_dist = abs(self.get_hardware.get_pos - self.get_after_pos) # distance from the target relative to current position
                println("target_dist: ",self.get_target_dist,brak=True)
                self.get_dist_per_step = self.get_target_dist / self.get_total_steps # distance per step
                println("dist_per_step: ",self.get_dist_per_step,brak=True)
                # time between each step should be servos step time, so motion as smooth as possible
                self.get_time_per_step = self.get_hardware.get_step_time
                print()
            def calc(self):
                pass # for recalculation during movement, perhaps better to set a reducing attribute of self in main loop or in run?
            def run(self):
                if self.get_steps_remaining <= 0:
                    self.get_time_remaining = 0
                    pass # don't do stuff
                    pass # delete the movement attribute
                    del self
                else:
                    # do stuff to move servos
                    if self.get_before_pos < self.get_after_pos:
                        self.get_hardware.set_pos(self.get_hardware.get_pos + self.get_dist_per_step)
                    if self.get_before_pos > self.get_after_pos:
                        self.get_hardware.set_pos(self.get_hardware.get_pos - self.get_dist_per_step)
                    # increment values needed for tracking
                    self.get_steps_remaining -= 1 # remove 1 from steps remaining (for determining when to stop)
                    self.get_time_remaining -= self.get_time_per_step
                    println("steps remaining: " + str(self.get_steps_remaining), brak=True)
                    println("time remaining: " + str(self.get_time_remaining), brak=True)                
        def set_pos(self,pos):
            self.get_pos = pos
            self.set_duty(valmap(self.get_pos,0,180,32,130))
        def set_duty(self,duty):
            self.get_duty = duty
            if hardware.is_physical: self.pwm.duty(int(self.get_duty))
            println("SERVO:[{:<5}],POS[{:<5}],DUTY[{:<5}]".format(self.get_name,round(self.get_pos,3),self.get_duty))
    class NewLed:
        def __init__(self,name,pin_num,initial_state):
            self.get_name = name
            self.get_pin_num = pin_num
            self.get_initial_state = initial_state
            self.get_state = initial_state
            if (hardware.is_physical):
                self.get_pin_object = machine.Pin(self.get_pin_num, machine.Pin.OUT)
        def set_state(self,state):
            self.get_state = state
            if hardware.is_physical:
                if state: self.pin_object.high()
                else: self.pin_object.low()
            print("{}.state = {}".format(self.name,self.get_state))
            
''' -- MISC CLASS DEFINITIONS -- '''
class TimeElapsedTracker:
    def __init__(self):
        self.script_started = time.time()
    def begin(self):
        self.start = time.time()
        self.elapsed = 0
    def check(self,print_elapsed=False):
        self.elapsed = time.time() - self.start
        if print_elapsed: println("[{:<4}]".format(self.elapsed))
        return(self.elapsed)
    def stop(self):
        self.end = time.time()
        self.elapsed = self.end - self.start
        return(self.elapsed)

# ---- MAIN SETUP ----
ROBOT = Robot("ROBOT") # create a new robot class
# define physical hardware
hardware = Hardware(is_physical) # set hardware as physical or code simulated
hardware.servos['A'] = Hardware.Servo("A",4,50,0.02,[90,0,180])
hardware.servos['B'] = Hardware.Servo("B",5,50,0.02,[90,0,180])
hardware.servos['C'] = Hardware.Servo("C",15,50,0.02,[90,0,180])
ROBOT.joints['A'] = Robot.Joint('A',hardware.servos['A'])
ROBOT.joints['B'] = Robot.Joint('B',hardware.servos['B'])
ROBOT.joints['C'] = Robot.Joint('C',hardware.servos['C'])

#hardware.add_servo("B",5,50,0.02,[0,0,180],hardware.is_physical)
#ROBOT.add_joint('A',90,0,180)
#ROBOT = Robot("ROBOT") # create a robot class
#ROBOT.joints['A'] = ROBOT.Joint() # create joint A (for all base main_classes)
    
#---primary loop---#
def main():
    global time_tracker
    time_tracker.check(print_elapsed=True) # set and print time elapsed
    if time_tracker.elapsed > 5: global end_loop; end_loop = True # exit if time elapsed > 10
    if time_tracker.elapsed > 3: joints['C'].movement = servos['C'].NewMovement(time_tracker,1,servos['C'].get_pos,servos['C'].get_home_pos,servos['C'])
    ROBOT.sync(time_tracker,time.time() + 2) # update all hardwares such as servos (pass on time_tracker)
    #input()
    #for joint in ROBOT.joints:
    #    print("SERVO:NOW:[{:>3}],SERVO:NEW:[{:>3}]".format(ROBOT.joints[joint].get_hardware.get_pos,ROBOT.joints[joint].get_hardware.get_new_pos))
        
###### START 'ER UP!!! ######
time_tracker = TimeElapsedTracker()
print("THIS SHOULD ALWAYS BE THE FIRST THING TO PRINT!!!")
joints = {'A': ROBOT.joints['A'],
          'B': ROBOT.joints['B'],
          'C': ROBOT.joints['C']}
servos = {'A': joints['A'].get_hardware,
          'B': joints['B'].get_hardware,
          'C': joints['C'].get_hardware}
joints['A'].movement = servos['A'].NewMovement(time_tracker,1,servos['A'].get_home_pos,0,ROBOT.joints['A'].get_hardware)
joints['B'].movement = servos['B'].NewMovement(time_tracker,1,servos['B'].get_home_pos,180,ROBOT.joints['B'].get_hardware)
joints['C'].movement = servos['C'].NewMovement(time_tracker,1,servos['C'].get_home_pos,20,ROBOT.joints['C'].get_hardware)

time_tracker.begin()

end_loop = False
while not end_loop: main()

time_tracker.stop

print("goodbye!")
