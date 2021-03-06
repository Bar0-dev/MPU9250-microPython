# This file is executed on every boot (including wake-boot from deepsleep)
#import esp
#esp.osdebug(None)
import uos, machine
uos.dupterm(machine.UART(0, 115200), 1)
import gc
gc.collect()
machine.Pin(0, machine.Pin.OUT)
machine.Pin(2, machine.Pin.IN, machine.Pin.PULL_UP)