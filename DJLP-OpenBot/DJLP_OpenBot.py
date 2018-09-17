''' -- IMPORTS -- '''
import time
import _thread
try: import machine; is_physical = True
except: is_physical = False
try: import dbi; dbi_present = True
except: dbi_present = False
try: import json; json_present = True
except: json_present = False
#if dbi_present and json_present: db = json.load(open('data/db.json'))
''' -- LOAD VARIABLES -- '''
robot_move_duration = 1 # normal duration is 1s
settings = json.load(open('settings.json'))
servos_json_filename = settings['servo_json_filename']
timeline_name = settings['timeline_filename']
dur_scale = settings['dur_scale'] 
pause_scale = settings['pause_scale']
print_moves = settings['print_moves']
print_interval_info = settings['print_interval_info']
s_update_servos_json = settings['update_servos_json']
''' -- VARIABLE WARNINGS -- '''
if print_moves is True:
    print("[ WARNING!!! ] print_moves is enabled in settings!")
    print("[ ---------- ] THIS CAN RESULT IN INCORRECT SYNC TIMING!!!")
    time.sleep(3)
if print_interval_info is True:
    print("[ WARNING!!! ] print_interval_info is enabled in settings!")
    print("[ ---------- ] THIS CAN RESULT IN INCORRECT SYNC TIMING!!!")
    time.sleep(3)

''' -- FUNCTION DEFINITIONS -- '''
class DebugTools:
    def __init__(self):
        pass
    class Printing:
        def __init__(self):
            pass
        def object(*args):
            for obj in args:
                print("class:",obj)
                try:
                    print("name:",obj.name)
                except:
                    print("name:","no '.name' attribute found")
                print("attributes:",obj.__dict__)
        def objects(obj_list):
            for obj in obj_list:
                print("name:",obj)
                print("object:",obj_list[obj])
                print("attributes:",obj_list[obj].__dict__)
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

def print_servo_pos_data(all_servos,ROBOT_OBJ):
    for s in all_servos:
        print("{:<25}|SERVO|{:<3} --- POS|{}".format("all_servos",s.get_name,s.get_pos))
    for j in ROBOT_OBJ.joints:
        s = ROBOT_OBJ.joints[j].get_hw
        print("{:<25}|SERVO|{:<3} --- POS|{}".format("ROBOT.joints[j].get_hw",s.get_name,s.get_pos))
    for s in ROBOT_OBJ.servos_dict:
        s_dict = ROBOT_OBJ.servos_dict[s]
        print("{:<25}|SERVO|{:<3} --- POS|{}".format("ROBOT.servos_dict",s_dict['name'],s_dict['pos']))
def timeline_tick(timeline,ROBOT_OBJ,t_dur_scale):
    global end_loop
    for keyframe in timeline:
        if keyframe['done'] is False:
            if keyframe['wait_until']*t_dur_scale < ROBOT_OBJ.tt.elapsed:
                if keyframe['action'] == 'move':
                    ROBOT_OBJ.set_joints(ROBOT_OBJ,keyframe['positions'],automove=False)
                elif keyframe['action'] == "exit":
                    end_loop = True # exit loop if prior conditions true
                elif keyframe['action'] == "home":
                    ROBOT_OBJ.home(ROBOT_OBJ.joints)
                else:
                    raise TypeError("KEYFRAME ACTION UNRECOGNISED!")
                keyframe['done'] = True
def print_servo_status(servo):
    println("| < {:<1} > [P{:<3}] [D{:<3}] |".format(servo.get_name,round(servo.get_pos),round(servo.get_duty)))
def interval_info(print_interval):
    if ROBOT.tt.has_passed(print_interval) is True:
        #print("{} seconds passed on this tick!".format(print_interval))
        println(ROBOT.tt.s_elapsed)
        lag = float()
        lag = round(ROBOT.tt.t_pass_dur - print_interval,6)
        println("LAG: {:<9}".format(lag))
        for joint in ROBOT.joints:
            _joint = ROBOT.joints[joint]
            print_servo_status(_joint.get_hw)
        print()
''' -- ROBOT CLASS DEFINITIONS -- '''
class Robot:
    def __init__(self,name,time_tracker,move_duration=1,servos_dict={},joints={},leds={},positions={}):
        self.name = name
        self.tt = time_tracker
        self.move_duration = move_duration
        self.joints = joints
        self.leds = leds
        self.positions = positions
        self.threads = {}
        # actions
        self.tt.begin() # start time tracker
        # imports
        try: import _thread
        except: raise Exception("can't import _thread!")
        import time
    def start_sync(self,delay=1,interval=0.00001,debug=True):
        # DEFINITIONS
        def sync(delay,interval,debug=True):
            time.sleep(delay)
            #any_moves = False
            while True: # loop until broken
                if ('sync' in self.threads) is True:
                    if self.threads['sync']['close'] is True:
                        break
                    else:
                        time.sleep(interval)
                        for j in self.joints: # for each of the robot's joints
                            if hasattr(self.joints[j], 'movement'): # if the joint has a movement attribute
                                j_move = self.joints[j].movement # (shorter code)
                                if j_move.done is True: # if the movement is finished
                                    del j_move # delete the movement attribute
                                else: # if the movement isn't finished
                                    #if debug is True: println(ROBOT.tt.s_elapsed)
                                    j_move.step() # run one tick step of movement
                                    #any_moves = True
                        #if any_moves is True and debug is True: print()
                else:
                    if debug is True: print("Robot|{}: sync thread missing from robot".format(self.name))
                    break
        # CODE
        if debug is True: print("Robot|{}: sync | initialising...".format(self.name))
        if 'sync' in self.threads: # if thread named "sync" already exists
            print("Robot|{}: sync | thread already running!".format(self.name)) # say this
        else: # if robot doesn't already have a sync thread
            self.threads['sync'] = {"close": False} # create an entry for sync thread in threads
            _thread.start_new_thread(sync, (delay,interval)) # start the thread itself
            if debug is True: print("Robot|{}: sync | thread started".format(self.name))
    def end_sync(self,debug=True):
        self.threads['sync']['close'] = True # send close signal to sync thread
        if debug is True: print("Robot|{}: sync | close thread signal sent...".format(self.name))
        if debug is True: print("Robot|{}: sync | threads: {}".format(self.name,self.threads))
        del self.threads['sync'] # delete sync thread
        if 'sync' not in self.threads:
            if debug is True: print("Robot|{}: sync | thread closed successfully".format(self.name))
        else:
            if debug is True:
                print("Robot|{}: sync | THREAD WASN'T DELETED!".format(self.name))
                print("Robot|{}: sync | threads: {}".format(self.name,self.threads))
            raise Exception("sync thread couldn't be closed")
    class Joint:
        def __init__(self,name,hardware):
            self.get_name = name
            self.get_hw = hardware
    class NewPosition:
        def __init__(self,joints):
            for joint in joints:
                pass # TODO: make this work
    #def syncloop(self):
    #    for joint in ROBOT.joints:
    #        ROBOT.joints[joint].synced = False
    #    at_pos = False
    #    while at_pos is False:
    #        self.sync()
    #        for joint in ROBOT.joints:
    #            _joint = ROBOT.joints[joint]
    #            if hasattr(_joint,'movement'):
    #                print("['{}'] is still moving...".format(_joint.get_name))
    #            else:
    #                #print("['{}'] has no movement attribute!".format(_joint.get_name))
    #                _joint.synced = True
    #        at_pos = True
    #        for joint in ROBOT.joints:
    #            if ROBOT.joints[joint].synced is False:
    #                at_pos = False
    #    print("sync finished!")
    #    if s_update_servos_json is True:
    #        ROBOT.update_json(servos_json_filename)
    def home(self,joints,hardhome=False):
        for joint in joints:
            joint = joints[joint]
            if hardhome:
                new_pos = joint.get_hw.get_hard_home
            else:
                new_pos = joint.get_hw.get_home_pos
            joint.movement = joint.get_hw.NewMovement(
                ROBOT.move_duration*dur_scale,
                new_pos,
                joint.get_hw
            )
    def update_json(self,json_filename):
        for servo in hardware.servos:
            _servo = hardware.servos[servo]
            _servo.force_update_dict() # force the dictionary to update in all servos found
            servo_name = _servo.servo_dict['name'] # store the servos name
            self.servos_dict[servo_name] = _servo.servo_dict # update our values in robots dictionary
        with open(json_filename,'w') as f:
            if hardware.is_physical: json.dump(self.servos_dict, f)
            else: json.dump(self.servos_dict, f, indent = 4,ensure_ascii = False)
    def set_joints(self,joint_pos_dict):
        for joint_pos in joint_pos_dict: # for each joint
            j = self.joints[joint_pos] # set the joint to the dict key
            pos = joint_pos_dict[joint_pos] # set the pos to the key's value
            j.movement = j.get_hw.NewMovement( # create a new movement for the joint
                self.move_duration, # use the robot's move_duration attribute for move duration
                pos, # use the pos defined in joint_pos_dict
                j.get_hw # use the joint defined by j
            )
        
''' -- HARDWARE CLASS DEFINITIONS -- '''
class Hardware:
    def __init__(self,debug=False,servos={},leds={}):
        try: import machine; is_physical = True
        except: is_physical = False
        self.is_physical = is_physical
        if debug is True:
            if self.is_physical is True:print("...running on hardware")
            else: print("running as simulation")
        self.servos = servos
        self.leds = leds
    def load_servos(self,json_filename='servos.json'):
        servos_dict = json.load(open(json_filename)) # load the json file into the robot's dict
        self.servos_dict = servos_dict # add json data to hardware object
        for s in self.servos_dict: # for each servo in the robot's dict
            _s = servos_dict[s]
            self.servos.update({_s['name']: Hardware.Servo(_s)})
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
            def __init__(self,move_duration,after_pos,hardware,print_status=True,print_break=True):
                #TODO: complete this
                println(ROBOT.tt.s_elapsed)
                self.done = False
                self.move_duration = move_duration
                self.get_before_pos = hardware.get_pos
                self.get_during_pos = hardware.get_pos
                self.get_after_pos = after_pos
                self.get_dist_from_pos = after_pos - self.get_before_pos #TODO: check this is correct
                self.get_hw = hardware
                self.get_before_time = time.time()
                self.get_after_time = time.time() + self.move_duration
                self.get_time_remaining = self.get_after_time - self.get_before_time
                # CALCULATIONS (run once per movement created)
                if print_status is True: print("~CALCULATING NEW MOVEMENT!~",end=" ")
                self.get_duration_time = self.get_after_time - self.get_before_time # during movement: no change
                # total number of ticks required to reach target by x seconds (dependent on servo's step time)
                self.get_total_steps = self.get_duration_time//self.get_hw.get_step_time # total number of steps taken from target A to target B
                # set how many step are left to take
                self.get_steps_remaining = self.get_total_steps # decrease this every tick! (CHANGES!)
                # set how much time is left
                self.get_time_remaining = self.get_duration_time
                # used for outputting to terminal, shouldn't be used in loops!!!
                self.get_target_dist = abs(self.get_hw.get_pos - self.get_after_pos) # distance from the target relative to current position
                self.get_dist_per_step = self.get_target_dist / self.get_total_steps # distance per step
                # time between each step should be servos step time, so motion as smooth as possible
                self.get_time_per_step = self.get_hw.get_step_time
                println("duration: ",self.get_duration_time,brak=True)
                println("total step count: ",self.get_total_steps,brak=True)
                println("set steps remaining: ",self.get_steps_remaining,brak=True)
                println("set time remaining: ",self.get_time_remaining,brak=True)
                println("target_dist: ",self.get_target_dist,brak=True)
                println("dist_per_step: ",self.get_dist_per_step,brak=True)
                if print_break is True: print()
            def calc(self):
                pass # for recalculation during movement, perhaps better to set a reducing attribute of self in main loop or in run?
            def step(self,update_servos_json=True,print_status=False,print_complete=True,print_break=False): # ss is step scale
                if self.get_steps_remaining > 0:
                    # do stuff to move servos
                    if self.get_before_pos == self.get_after_pos:
                        if print_moves is True:
                            println("RESIDUAL STEPS!")
                    else:
                        if self.get_before_pos < self.get_after_pos:
                            new_pos = self.get_hw.get_pos + (self.get_dist_per_step)
                        elif self.get_before_pos > self.get_after_pos:
                            new_pos = self.get_hw.get_pos - (self.get_dist_per_step)
                        else:
                            raise TypeError("position isn't equal, greater or less than???")
                        #println("new_pos:",new_pos) # TODO: remove this, for debugging
                        self.get_hw.set_pos(new_pos)
                    # increment values needed for tracking
                    self.get_steps_remaining -= (1) # remove 1 from steps remaining (for determining when to stop)
                    self.get_time_remaining -= (self.get_time_per_step)
                    if print_status is True: println("step-left:{:<5} | time-left:{:<5}".format(round(self.get_steps_remaining,5),round(self.get_time_remaining,5)),brak=True)
                else:
                    # set time remaining to 0
                    self.get_time_remaining = 0
                    self.get_hw.set_pos(self.get_after_pos,print_move=False) # TODO: check this doesn't cause issues
                    # don't do stuff
                    # delete the movement attribute
                    self.done = True
                    if print_complete is True:
                        print("Movement Done!")
                if print_break: print()
        def set_pos(self,pos,print_move=print_moves):
            # TODO: consider min/max values before settings
            self.get_pos = pos
            self.servo_dict['pos'] = pos # needs to be updated when changed
            self.set_duty(valmap(self.get_pos,0,180,32,130),print_move=print_move)
        def set_duty(self,duty,print_move=False):
            self.get_duty = duty
            if hardware.is_physical:
                self.pwm.duty(int(self.get_duty))
            if print_move:
                print_servo_status(self)
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
        self.t_first = float(0)
        self.elapsed = float(0)
        self.s_elapsed = ''
    def check(self,print_elapsed=False):
        try:
            self.elapsed = time.time() - self.start
            self.s_elapsed = "[{:<7}]".format(round(self.elapsed,4))
        except:
            raise Exception("error checking time. Ensure you ran '.begin()' first")
        if print_elapsed: println(self.s_elapsed)
        return(self.elapsed)
    def stop(self):
        self.end = time.time()
        self.elapsed = self.end - self.start
        return(self.elapsed)
    def has_passed(self,duration):
        if self.elapsed > self.t_first + duration:
            self.t_pass_dur = self.elapsed - self.t_first
            self.t_first = self.elapsed
            return(True)
        else:
            return(False)
            # return true
            # set new t_first
class Button:
    def __init__(self,pin):
        self.pin = machine.Pin(pin, machine.Pin.IN, machine.Pin.PULL_UP)
        self.read_state = self.pin.value
        self.state = self.read_state()
        self.last_state = self.read_state()
    def read(self,loop=True):
        self.state = self.read_state() # store read state
        if self.last_state == False and self.state == True:
            print('Button released!')
        elif self.last_state == True and self.state == False:
            print('Button pressed!')
        else:
            pass
            #print("Something went terribly wrong...")
        self.last_state = self.state # set current state for future comparison
        if self.state == 1: return(False)
        elif self.state == 0: return(True)
        else: raise Exception("button state isn't 0 or 1. Was MicroPython updated?")
        
# ---- MAIN SETUP ----
tt = TimeElapsedTracker()
ROBOT = Robot("ROBOT",tt,move_duration=robot_move_duration) # create the robot object
# define physical hardware
hardware = Hardware(is_physical) # set hardware as physical or code simulated
# load servos stored in json into hardware object as well as a robot's servo_dict
'''
load_servos_from_json(servos_json_filename,ROBOT)
'''
# print data for each servo object found in hardware
'''
print_obj_list(hardware.servos)
'''
# print ROBOT.servos_dict (local copy of json file)
'''
print("ROBOT.servos_dict")
for servo in ROBOT.servos_dict:
    print(ROBOT.servos_dict[servo])
'''
# define the robots joints and what hardware is attached at each
'''
ROBOT.joints = {'A': Robot.Joint('A',hardware.servos['A']),
                'B': Robot.Joint('B',hardware.servos['B']),
                'C': Robot.Joint('C',hardware.servos['C']),
                'D': Robot.Joint('D',hardware.servos['D'])}
'''
# print data for each joint found on the robot
'''
print_obj_list(ROBOT.joints)
'''

# ---- put ROBOT's attributes in a more accessible variables to shorten code
# joints
'''
j_a = ROBOT.joints['A']
j_b = ROBOT.joints['B']
j_c = ROBOT.joints['C']
j_d = ROBOT.joints['D']
all_joints = [j_a,j_b,j_c,j_d]
'''
# servos
'''
s_a = ROBOT.joints['A'].get_hw
s_b = ROBOT.joints['B'].get_hw
s_c = ROBOT.joints['C'].get_hw
s_d = ROBOT.joints['D'].get_hw
all_servos = [s_a,s_b,s_c,s_d]
# timeline
timeline = json.load(open(timeline_name))
# LEDs
if hardware.is_physical is True:
    toggle_led = machine.Pin(14,machine.Pin.OUT)
    loop_led = machine.Pin(12,machine.Pin.OUT)
    button = Button(32)
'''

#---primary loop---#
def main():
    #println("✓")
    ROBOT.tt.check(print_elapsed=False) # set time elapsed    
    global timeline
    global end_loop
    timeline_tick(timeline,ROBOT,dur_scale)
    button_val = button.read()
    if button_val is True:
        toggle_led.value(True)
        pass
    elif button_val is False:
        toggle_led.value(False)
        ROBOT.sync() # update all hardwares such as servos
    if print_interval_info is True:
        interval_info(0.5)
        
###### START 'ER UP!!! ######
