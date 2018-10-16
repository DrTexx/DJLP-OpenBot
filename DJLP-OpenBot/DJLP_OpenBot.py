''' -- IMPORTS -- '''
import time
import _thread
try: import machine; is_physical = True
except: is_physical = False
#try: import dbi; dbi_present = True
#except: dbi_present = False
try: import json; json_present = True
except: json_present = False
#if dbi_present and json_present: db = json.load(open('data/db.json'))
''' -- LOAD VARIABLES -- '''
if is_physical is True:
    def time_now():
        return(time.ticks_us())
else:
    def time_now():
        out = time.time()
        out = (out*1000)*1000
        return(out)
robot_move_duration = 1 # normal duration is 1s
settings = json.load(open('settings.json'))
servos_json_filename = settings['servo_json_filename']
timeline_name = settings['timeline_filename']
default_move_duration = settings['default_move_duration'] 
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
        def object(self,*args):
            for obj in args:
                print("class:",obj)
                try:
                    print("name:",obj.name)
                except:
                    print("name:","no '.name' attribute found")
                print("attributes:",obj.__dict__)
class DataTools:
    def __init__(self,name="DataTools"):
        self.name = name
    def valmap(self,value,in_min,in_max,out_min,out_max):
        return ((value - in_min) * (out_max - out_min) // (in_max - in_min) + out_min)
    def sublist_gen(self,li, cols=2):
        start = 0
        for i in range(cols):
            stop = start + len(li[i::cols])
            yield li[start:stop]
            start = stop
class ThreadManager:
    def __init__(self,parent,func):
        self.parent = parent
        self.func = func(parent)
        # imports
        try: import _thread
        except: raise Exception("can't import _thread!")
    def start(self,debug=True):
        p = self.parent
        f = self.func
        p_cn = self.parent.__class__.__name__
        if debug is True: print("{}|{}: {} | initialising...".format(p_cn,p.name,f.name))
        if f.name in p.threads: # if thread named x already exists in parents threads
            if debug is True: print("{}|{}: {} | thread already running!".format(p_cn,p.name,f.name)) # say this
        else: # if robot doesn't already have a sync thread
            p.threads[f.name] = {"close": False} # create an entry for sync thread in threads
            _thread.start_new_thread(f.run, ()) # start the thread itself
            if debug is True: print("{}|{}: {} | thread started".format(p_cn,p.name,f.name))
    def end(self,debug=True):
        p = self.parent
        f = self.func
        p_cn = self.parent.__class__.__name__
        if (f.name in p.threads) is False: # if thread isn't in parent
            if debug is True: print("{}|{}: {} | thread isn't running".format(p_cn,p.name,f.name))
        else: # if thread is in parent
            p.threads[f.name]['close'] = True # send close signal to sync thread
            if debug is True: print("{}|{}: {} | close thread signal sent...".format(p_cn,p.name,f.name))
            if debug is True: print("{}|{}: {} | threads: {}".format(p_cn,p.name,f.name,p.threads))
            del p.threads[f.name] # delete sync thread
            if f.name not in p.threads:
                if debug is True: print("{}|{}: {} | thread closed successfully".format(p_cn,p.name,f.name))
            else:
                if debug is True:
                    print("{}|{}: {} | THREAD WASN'T DELETED!".format(p_cn,p.name,f.name))
                    print("{}|{}: {} | threads: {}".format(p_cn,p.name,f.name,p.threads))
                raise Exception("thread couldn't be closed")
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
    class Sync:
        def __init__(self,parent,name="sync"):
            self.name = name
            self.parent = parent
        def run(self,delay=1,interval=0.00001,debug=True):
            time.sleep(delay)
            p = self.parent
            #any_moves = False
            while True: # loop until broken
                if (self.name in p.threads) is True:
                    if p.threads['sync']['close'] is True:
                        break
                    else:
                        time.sleep(interval)
                        for j in p.joints: # for each of the robot's joints
                            if hasattr(p.joints[j], 'movement'): # if the joint has a movement attribute
                                j_move = p.joints[j].movement # (shorter code)
                                if j_move.done is True: # if the movement is finished
                                    del j_move # delete the movement attribute
                                else: # if the movement isn't finished
                                    #if debug is True: println(ROBOT.tt.s_elapsed)
                                    # TODO: this needs to occur once once x seconds passed
                                    if j_move.step_manager.allowstepnow() is True:
                                        print("stepping...",end="")
                                        j_move.step() # run one tick step of movement
                                        print()
                                    #any_moves = True
                        #if any_moves is True and debug is True: print()
                else:
                    if debug is True: print("{}|{}: sync | thread missing from robot".format(p_cn,p.name))
                    break
    def __init__(self,name,time_tracker,move_duration=default_move_duration,servos_dict={},joints={},leds={},positions={}):
        self.name = name
        self.tt = time_tracker
        self.move_duration = move_duration
        self.joints = joints
        self.leds = leds
        self.positions = positions
        self.threads = {}
        # actions
        self.tt.begin() # start time tracker
    class Joint:
        def __init__(self,name,specific_hardware):
            self.name = name
            self.get_hw = specific_hardware
            self.isactive = True
    class NewPosition:
        def __init__(self,joints):
            for joint in joints:
                pass # TODO: make this work
    def home(self,hardhome=False):
        for joint in self.joints:
            joint = self.joints[joint]
            if hardhome:
                new_pos = joint.get_hw.get_hard_home
            else:
                new_pos = joint.get_hw.get_home_pos
            joint.movement = joint.get_hw.NewMovement(
                self.tt,
                self.move_duration,
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
                self.tt,
                self.move_duration, # use the robot's move_duration attribute for move duration
                pos, # use the pos defined in joint_pos_dict
                j.get_hw # use the joint defined by j
            )
        
''' -- HARDWARE CLASS DEFINITIONS -- '''
class Hardware:
    class Servo:
        class NewMovement:
            class StepManager:
                def __init__(self,time_tracker,time_between_steps):
                    self.tt = time_tracker
                    self.time_between_steps = time_between_steps
                def allowstepnow(self,debug=True):
                    if hasattr(self, 'last_request') is False: # is this is the first request
                        if debug is True: print("first request. allowing")
                        self.last_request = time_now() # set the last request to now
                        return(True) # allow a step
                    else:
                        if debug is True: print("not first request",end=" ")
                        last = self.last_request
                        now = time_now()
                        passed = now - last
                        if passed > self.time_between_steps: # if a step's worth of time has passed
                            if debug is True: print("enough passed | {} > {}".format(passed,self.time_between_steps))
                            self.last_request = time_now()
                            return(True)
                        else:
                            return(False)
            def __init__(self,time_tracker,move_duration,after_pos,hardware,print_status=True,print_break=True):
                #TODO: complete this
                println("duration: ",move_duration,brak=True)
                if print_status is True: println(time_tracker.s_elapsed)
                self.done = False
                self.get_hw = hardware
                self.pos_start = hardware.get_pos
                self.pos_end = after_pos
                self.move_duration = move_duration
                self.time_remaining = move_duration
                self.time_start = time_now()
                print(self.time_start)
                self.time_end = self.time_start + move_duration
                print(self.time_end - self.time_start)
                # CALCULATIONS (run once per movement created)
                println("time_end:",self.time_end,brak=True)
                if print_status is True: print("~CALCULATING NEW MOVEMENT!~",end=" ")
                self.pos_now = self.pos_start
                self.get_dist_from_pos = abs(self.pos_end - self.pos_start) #TODO: check this is correct
                self.get_time_remaining = move_duration
                println("time remaining:",self.get_time_remaining,brak=True)
                # total number of ticks required to reach target by x seconds (dependent on servo's step time)
                self.get_total_steps = self.move_duration//self.get_hw.get_step_time # total number of steps taken from target A to target B
                println("total step count: ",self.get_total_steps,brak=True)
                # set how many step are left to take
                self.get_steps_remaining = self.get_total_steps # decrease this every tick! (CHANGES!)
                println("set steps remaining: ",self.get_steps_remaining,brak=True)
                # set how much time is left
                self.get_time_remaining = self.move_duration
                println("set time remaining: ",self.get_time_remaining,brak=True)
                # used for outputting to terminal, shouldn't be used in loops!!!
                self.get_target_dist = abs(self.get_hw.get_pos - self.pos_end) # distance from the target relative to current position
                println("target_dist: ",self.get_target_dist,brak=True)
                self.get_dist_per_step = self.get_target_dist / self.get_total_steps # distance per step
                println("dist_per_step: ",self.get_dist_per_step,brak=True)
                # time between each step should be servos step time, so motion as smooth as possible
                self.get_time_per_step = self.get_hw.get_step_time
                print("time per step",self.get_time_per_step)
                self.step_manager = self.StepManager(time_tracker,self.get_time_per_step)
                print("step_manager",self.step_manager)
                if print_break is True: print()
            def calc(self):
                pass # for recalculation during movement, perhaps better to set a reducing attribute of self in main loop or in run?
            def step(self,update_servos_json=True,print_status=True,print_complete=True,print_break=False): # ss is step scale
                step_r = self.get_steps_remaining
                before = self.pos_start
                now = self.get_hw.get_pos
                after = self.pos_end
                dps = self.get_dist_per_step
                if step_r > 0: # if there are steps remaining
                    if before == after: # if the position matches it's target
                        if print_moves is True: println("RESIDUAL STEPS!")
                    else: # if the position doesn't match it's target
                        if before < after: # if the current position is less than target
                            new_pos = now + dps
                        elif before > after: # if the current position is more than target
                            new_pos = now - dps
                        else: # if it's neither
                            raise TypeError("position isn't equal, greater or less than???")
                        self.get_hw.set_pos(new_pos) # set the new position
                    # increment values needed for tracking
                    self.get_steps_remaining -= (1) # remove 1 from steps remaining (for determining when to stop)
                    self.time_remaining = self.time_end - time_now()
                    if print_status is True: println("step-left:{:<5} | time-left:{:<5}".format(self.get_steps_remaining,self.time_remaining),brak=True)
                    if print_status is True: println("pos = {}".format(self.get_hw.get_pos))
                else: # if there are no steps remaining
                    self.get_time_remaining = 0
                    self.get_hw.set_pos(self.pos_end,print_move=False) # ensure servo at position (TODO: check this doesn't cause issues)
                    # delete the movement attribute
                    self.done = True
                    if print_complete is True:
                        print("Movement Done!",end=" ")
                if print_break: print()
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
            self.get_duty = DataTools().valmap(
                self.get_pos,
                self.get_pos_min,
                self.get_pos_max,
                self.get_duty_min,
                self.get_duty_max)
            if hardware.is_physical is True:
                self.pin = machine.Pin(self.get_pin_num)
                self.pwm = machine.PWM(self.pin,freq=self.get_freq)
        def set_pos(self,pos,print_move=print_moves):
            # TODO: consider min/max values before settings
            if pos == 'home':
                self.get_pos = self.get_home_pos
            else:
                self.get_pos = pos
            self.servo_dict['pos'] = self.get_pos # needs to be updated when changed
            self.set_duty(DataTools().valmap(self.get_pos,self.get_pos_min,self.get_pos_max,self.get_duty_min,self.get_duty_max),print_move=print_move)
        def set_duty(self,duty,print_move=False):
            self.get_duty = duty
            if hardware.is_physical is True:
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
    def __init__(self,debug=False,servos={},leds={},buttons={}):
        try: import machine; is_physical = True
        except: is_physical = False
        self.is_physical = is_physical
        if debug is True:
            if self.is_physical is True:print("...running on hardware")
            else: print("running as simulation")
        self.servos = servos
        self.leds = leds
        self.buttons = buttons
    def load_servos(self,json_filename='servos.json'):
        servos_dict = json.load(open(json_filename)) # load the json file into the robot's dict
        self.servos_dict = servos_dict # add json data to hardware object
        for s in self.servos_dict: # for each servo in the robot's dict
            _s = servos_dict[s]
            self.servos.update({_s['name']: Hardware.Servo(_s)})
    class Led:
        def __init__(self,name,pin_num,initial_state):
            self.get_name = name
            self.get_pin_num = pin_num
            self.get_initial_state = initial_state
            self.get_state = initial_state
            if hardware.is_physical is True:
                self.get_pin_object = machine.Pin(self.get_pin_num, machine.Pin.OUT)
        def set_state(self,state):
            self.get_state = state
            if hardware.is_physical is True:
                if state: self.pin_object.high()
                else: self.pin_object.low()
            print("{}.state = {}".format(self.name,self.get_state))
    class Button:
        def __init__(self,pin):
            if is_physical is True:
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
        def start_read(self,var_to_change):
            pass # TODO
            
''' -- MISC CLASS DEFINITIONS -- '''
class TimeElapsedTracker:
    def __init__(self):
        self.script_started = time_now()
    def begin(self):
        self.start = time_now()
        self.t_first = float(0)
        self.elapsed = float(0)
        self.s_elapsed = ''
    def check(self,print_elapsed=False):
        try:
            self.elapsed = time_now() - self.start
            self.s_elapsed = "[{:<7}]".format(round(self.elapsed,4))
        except:
            raise Exception("error checking time. Ensure you ran '.begin()' first")
        if print_elapsed: println(self.s_elapsed)
        return(self.elapsed)
    def stop(self):
        self.end = time_now()
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
        
# ---- MAIN SETUP ----
hardware = Hardware(is_physical) # set hardware as physical or code simulated
#tt = TimeElapsedTracker()
#ROBOT = Robot("ROBOT",tt,move_duration=robot_move_duration) # create the robot object
# define physical hardware
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
