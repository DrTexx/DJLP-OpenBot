import _thread
from platform import machine
class HardwareMan:
    def __init__(self):
        # -- import dependencies
        # -- initial tests
        # is the hardware on x86, x64 or ARM? (PC or Microprocessor essentially)
        self.physical = self.canImportMachine()
        # are the required packages for this manager present?
        # -- set states
        # manager not started until called to
        self.running = False # disables tick loop
    def start(self):
        # do any settings conflict?
        # -- enter the main loop
        if self.running is False: # if not running already
            self.running = True
            _thread.start_new_thread(self.tick, ())
        else:
            print("[HardwareMan] HardwareMan is already running!")
    def stop(self):
        self.running = False
    def status(self):
        print("--[HardwareMan status]-- | running?: {} | physical?: {}".format(self.running,self.physical))
    def tick(self):
        pass
        # TODO: make this the actual loop
        i = 0
        while self.running is True:
            i += 1
            print("{:<3}: hello!".format(i))
            if i >= 5:
                self.running = False
    def canImportMachine(self):
        try: import machine; return(True)
        except: return(False)
#NOTE: don't forget to initialize managers you add with .start()
