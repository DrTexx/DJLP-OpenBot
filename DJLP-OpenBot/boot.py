# This file is executed on every boot (including wake-boot from deepsleep)
#import esp
#esp.osdebug(None)
#import webrepl_setup
import network
station = network.WLAN(network.STA_IF)
station.active(True)
station.connect("painintheass","meatkaneira")
#station.connect("cchs","hackmelb")
station.isconnected()
station.ifconfig()

import webrepl
webrepl.start()