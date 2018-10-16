import _thread
class HardwareMan:
    def __init__(self):
        # -- import dependencies
        # -- initial tests
        # is the hardware on x86, x64 or ARM? (PC or Microprocessor essentially)
        # are the required packages for this manager present?
        # -- set states
        # manager not started until called to
        self.running = False # disables tick loop
    def start(self):
        # do any settings conflict?
        # -- enter the main loop
        self.running = True
        _thread.start_new_thread(self.tick, ())
    def stop(self):
        self.running = False
    def status(self):
        print("--[Thread manager status]-- | running?: {}".format(self.running))
    def tick(self):
        pass
        # TODO: make this the actual loop
        i = 0
        while self.running is True:
            i += 1
            print("{:<3}: hello!".format(i))
            if i >= 100:
                self.running = False
        
#NOTE: don't forget to initialize managers you add with .start()
