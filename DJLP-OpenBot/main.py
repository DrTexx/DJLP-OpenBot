''' -- IMPORTS -- '''
import time
import _thread
try: import machine; is_physical = True
except: is_physical = False
try: import dbi; dbi_present = True
except: dbi_present = False
try: import json; json_present = True
except: json_present = False
if dbi_present and json_present: db = json.load(open('data/db.json'))

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
def print_obj_list(obj_list):
    for obj in obj_list:
        print("name:",obj)
        print("object:",obj_list[obj])
        print("attributes:",obj_list[obj].__dict__)
def load_servos_from_json(json_filename,robot_obj):
    try: json_servos_dict = json.load(open(json_filename)) # load the json file into the robot's dict
    except: raise Exception("cannot open servo file '{}' for some reason (is it there?)".format(json_filename))
    robot_obj.servos_dict = json_servos_dict
    # TODO: for servo specs: check if this is dict or string, if string, find preset
    for s in robot_obj.servos_dict: # for each servo in the robot's dict
        _s = robot_obj.servos_dict[s]
        hardware.servos.update(
            {_s['name']: Hardware.Servo(_s)}
        )
def print_servo_pos_data(all_servos,ROBOT_OBJ):
    for s in all_servos:
        print("{:<25}|SERVO|{:<3} --- POS|{}".format("all_servos",s.get_name,s.get_pos))
    for j in ROBOT_OBJ.joints:
        s = ROBOT_OBJ.joints[j].get_hw
        print("{:<25}|SERVO|{:<3} --- POS|{}".format("ROBOT.joints[j].get_hw",s.get_name,s.get_pos))
    for s in ROBOT_OBJ.servos_dict:
        s_dict = ROBOT_OBJ.servos_dict[s]
        print("{:<25}|SERVO|{:<3} --- POS|{}".format("ROBOT.servos_dict",s_dict['name'],s_dict['pos']))

        
''' -- VARIABLE DEFINITONS -- '''
servos_json_filename = 'servos.json'
speed_scale = 4 # normal duration is 0.25
pause_scale = 0.5

''' -- ROBOT CLASS DEFINITIONS -- '''
class Robot:
    def __init__(self,name,time_tracker,default_move_duration=1):
        self.name = name
        self.servos_dict = {}
        self.joints = {}
        self.leds = {}
        self.positions = {}
        self.tt = time_tracker
        self.move_duration = default_move_duration
    class Joint:
        def __init__(self,name,hardware):
            self.get_name = name
            self.get_hw = hardware
    class NewPosition:
        def __init__(self,joints):
            for joint in joints:
                pass # TODO: make this work
    def sync(self,speed_scale_override=1,print_break=True):
        # TODO: make move_duration_override have an effect on all movements
        #print("syncing...",end=" ")
        for joint in ROBOT.joints:
            if hasattr(ROBOT.joints[joint], 'movement'): # if this joint has a movement attribute
                ROBOT.joints[joint].movement.run(run_speed_scale=speed_scale_override) # this tick, increment servos a bit to target
        if print_break: print()
    def syncloop(self,sync_scale_override):
        at_pos = False
        while not at_pos:
            self.sync(sync_scale_override=sync_scale_override)
    def home(self,joints,hardhome=False):
        for joint in joints:
            joint = joints[joint]
            if hardhome:
                new_pos = joint.get_hw.get_hard_home
            else:
                new_pos = joint.get_hw.get_home_pos
            joint.movement = joint.get_hw.NewMovement(self,new_pos,joint.get_hw)
    def update_json(self,json_filename):
        for servo in hardware.servos:
            _servo = hardware.servos[servo]
            _servo.force_update_dict() # force the dictionary to update in all servos found
            servo_name = _servo.servo_dict['name'] # store the servos name
            self.servos_dict[servo_name] = _servo.servo_dict # update our values in robots dictionary
        with open(json_filename,'w') as f:
            if hardware.is_physical: json.dump(self.servos_dict, f)
            else: json.dump(self.servos_dict, f, indent = 4,ensure_ascii = False)
    def set_joints(self,positions):
        for item in positions:
            ROBOT.joints[item].movement = ROBOT.joints[item].get_hw.NewMovement(ROBOT,positions[item],ROBOT.joints[item].get_hw)
            
''' -- HARDWARE CLASS DEFINITIONS -- '''
class Hardware:
    def __init__(self,is_physical):
        state_message(is_physical,"...running on hardware","running as simulation")
        self.is_physical = is_physical
        self.servos = {}
        self.leds = {}
    class Servo:
        def __init__(self,og_servo_dict):
            self.servo_dict = og_servo_dict
            self.get_name = self.servo_dict['name']
            self.get_pin_num = self.servo_dict['pin_num']
            self.get_pos = self.servo_dict['pos'] # current position of servo (NEEDS TO BE UPDATED ALWAYS)
            self.get_home_pos = self.servo_dict['pos_home']
            self.get_freq = self.servo_dict['specs']['freq']
            self.get_step_time = self.servo_dict['specs']['step_time']
            self.get_pos_min = self.servo_dict['specs']['hard_min'] # make this have an effect
            self.get_pos_max = self.servo_dict['specs']['hard_max'] # make this have an effect
            self.get_hard_home = self.servo_dict['specs']['hard_max'] / 2
            self.get_duty_min = self.servo_dict['specs']['duty_min'] # make this have an effect
            self.get_duty_max = self.servo_dict['specs']['duty_max'] # make this have an effect
            self.get_duty = valmap(self.get_pos,
                                   self.get_pos_min,
                                   self.get_pos_max,
                                   self.get_duty_min,
                                   self.get_duty_max)
            if (hardware.is_physical):
                self.pin = machine.Pin(self.get_pin_num)
                self.pwm = machine.PWM(self.pin,freq=self.get_freq)
        class NewMovement:
            def __init__(self,robot_obj,after_pos,hardware,print_break=False):
                #TODO: complete this
                self.move_duration = robot_obj.move_duration
                self.get_before_pos = hardware.get_pos
                self.get_during_pos = hardware.get_pos
                self.get_after_pos = after_pos
                self.get_dist_from_pos = after_pos - self.get_before_pos #TODO: check this is correct
                self.get_hw = hardware
                self.get_before_time = time.time()
                self.get_after_time = time.time() + self.move_duration
                self.get_time_remaining = self.get_after_time - self.get_before_time
                # CALCULATIONS (run once per movement created)
                self.get_duration_time = self.get_after_time - self.get_before_time # during movement: no change
                println("duration: ",self.get_duration_time,brak=True)
                # total number of ticks required to reach target by x seconds (dependent on servo's step time)
                self.get_total_steps = self.get_duration_time//self.get_hw.get_step_time # total number of steps taken from target A to target B
                println("total step count: ",self.get_total_steps,brak=True)
                # set how many step are left to take
                self.get_steps_remaining = self.get_total_steps # decrease this every tick! (CHANGES!)
                println("set steps remaining: ",self.get_steps_remaining,brak=True)
                # set how much time is left
                self.get_time_remaining = self.get_duration_time
                println("set time remaining: ",self.get_time_remaining,brak=True)
                # used for outputting to terminal, shouldn't be used in loops!!!
                self.get_target_dist = abs(self.get_hw.get_pos - self.get_after_pos) # distance from the target relative to current position
                println("target_dist: ",self.get_target_dist,brak=True)
                self.get_dist_per_step = self.get_target_dist / self.get_total_steps # distance per step
                println("dist_per_step: ",self.get_dist_per_step,brak=True)
                # time between each step should be servos step time, so motion as smooth as possible
                self.get_time_per_step = self.get_hw.get_step_time
                if print_break: print()
                else: print("already there.",end="")
            def calc(self):
                pass # for recalculation during movement, perhaps better to set a reducing attribute of self in main loop or in run?
            def run(self,run_speed_scale=1,print_status=False): # ss is step scale
                rss = run_speed_scale
                if self.get_steps_remaining <= 0:
                    # set time remaining to 0
                    self.get_time_remaining = 0
                    self.get_hw.set_pos(self.get_after_pos) # TODO: check this doesn't cause issues
                    # don't do stuff
                    # delete the movement attribute
                    del self
                else:
                    # do stuff to move servos
                    if self.get_before_pos < self.get_after_pos:
                        self.get_hw.set_pos(self.get_hw.get_pos + (self.get_dist_per_step*rss))
                    if self.get_before_pos > self.get_after_pos:
                        self.get_hw.set_pos(self.get_hw.get_pos - (self.get_dist_per_step*rss))
                    # increment values needed for tracking
                    self.get_steps_remaining -= (1*rss) # remove 1 from steps remaining (for determining when to stop)
                    self.get_time_remaining -= (self.get_time_per_step*rss)
                    if print_status is True: println("step-left:{:<5} | time-left:{:<5}".format(round(self.get_steps_remaining,5),round(self.get_time_remaining,5)),brak=True)
                    #time.sleep(self.get_time_per_step)
        def set_pos(self,pos,print_move=True):
            # TODO: consider min/max values before settings
            self.get_pos = pos
            self.servo_dict['pos'] = pos # needs to be updated when changed
            self.set_duty(valmap(self.get_pos,0,180,32,130),print_move=print_move)
        def set_duty(self,duty,print_move=False):
            self.get_duty = duty
            if hardware.is_physical: self.pwm.duty(int(self.get_duty))
            if print_move: println("['{:<1}'|P:{:<3}|D:{:<3}]".format(self.get_name,round(self.get_pos),self.get_duty))
        def force_update_dict(self):
            self.servo_dict['name'] = self.get_name
            self.servo_dict['pin_num'] = self.get_pin_num
            self.servo_dict['pos'] = self.get_pos #TODO: should just be set when position set
            self.servo_dict['pos_home'] = self.get_home_pos
            self.servo_dict['specs']['freq'] = self.get_freq
            self.servo_dict['specs']['step_time'] = self.get_step_time
            self.servo_dict['specs']['hard_min'] = self.get_pos_min
            self.servo_dict['specs']['hard_max'] = self.get_pos_max 
            self.servo_dict['specs']['duty_min'] = self.get_duty_min
            self.servo_dict['specs']['duty_max'] = self.get_duty_max
    class Led:
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
        if print_elapsed: println("[{:<7}]".format(round(self.elapsed,3)))
        return(self.elapsed)
    def stop(self):
        self.end = time.time()
        self.elapsed = self.end - self.start
        return(self.elapsed)

# ---- MAIN SETUP ----
tt = TimeElapsedTracker()
ROBOT = Robot("ROBOT",tt,default_move_duration=1*speed_scale) # create the robot object
# define physical hardware
hardware = Hardware(is_physical) # set hardware as physical or code simulated
# load servos stored in json into hardware object as well as a robot's servo_dict
load_servos_from_json(servos_json_filename,ROBOT)
# print data for each servo object found in hardware
print_obj_list(hardware.servos)
# print ROBOT.servos_dict (local copy of json file)
print("ROBOT.servos_dict")
for servo in ROBOT.servos_dict:
    print(ROBOT.servos_dict[servo])
# define the robots joints and what hardware is attached at each
ROBOT.joints = {'A': Robot.Joint('A',hardware.servos['A']),
                'B': Robot.Joint('B',hardware.servos['B']),
                'C': Robot.Joint('C',hardware.servos['C']),
                'D': Robot.Joint('D',hardware.servos['D'])}
# print data for each joint found on the robot
print_obj_list(ROBOT.joints)

# ---- put ROBOT's attributes in a more accessible variables to shorten code
# joints
j_a = ROBOT.joints['A']
j_b = ROBOT.joints['B']
j_c = ROBOT.joints['C']
j_d = ROBOT.joints['D']
all_joints = [j_a,j_b,j_c,j_d]
# servos
s_a = ROBOT.joints['A'].get_hw
s_b = ROBOT.joints['B'].get_hw
s_c = ROBOT.joints['C'].get_hw
s_d = ROBOT.joints['D'].get_hw
all_servos = [s_a,s_b,s_c,s_d]

#---primary loop---#
def main():
    global move0_sent
    global m1_sent
    global m2_sent
    global m3_sent
    global m4_sent
    global end_loop
    ROBOT.tt.check(print_elapsed=True) # set and print time elapsed
    #timeline = ['time elapsed', 'condition for completion', 'robot joints and positions']
    #for keyframe in timeline:
    if ROBOT.tt.elapsed > 5*speed_scale:
        end_loop = True # exit if time elapsed > 10
    elif ROBOT.tt.elapsed > 4*speed_scale and m4_sent is False:
        ROBOT.home(ROBOT.joints)
        m4_sent = True
    elif ROBOT.tt.elapsed > 3*speed_scale and m3_sent is False:
        ROBOT.set_joints({'A': 90,'B': 90,'C': 90,'D': 90})
        m3_sent = True
    elif ROBOT.tt.elapsed > 2*speed_scale and m2_sent is False:
        ROBOT.set_joints({'A': 150,'B': 70,'C': 20,'D': 170})
        m2_sent = True
    elif ROBOT.tt.elapsed > 1*speed_scale and m1_sent is False:
        ROBOT.set_joints({'A': 0,'B': 120,'C': 0,'D': 180})
        m1_sent = True
    elif ROBOT.tt.elapsed > 0 and move0_sent is False:
        ROBOT.home(ROBOT.joints)
        move0_sent = True
    ROBOT.sync(speed_scale_override=speed_scale) # update all hardwares such as servos (pass on time_tracker)
    
###### START 'ER UP!!! ######
print("THIS SHOULD ALWAYS BE THE FIRST THING TO PRINT!!!")

ROBOT.tt.begin()

end_loop = False
move0_sent = False
m1_sent = False
m2_sent = False
m3_sent = False
m4_sent = False
while not end_loop: main()

ROBOT.tt.stop

print("FINAL ROBOT JOINT POSITIONS:")
print_servo_pos_data(all_servos,ROBOT)

ROBOT.update_json(servos_json_filename) # update json file with current data
print("goodbye!")
