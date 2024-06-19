
"""
    OTA Test
"""

# Imports
import time
import uasyncio as asyncio
import machine
from machine import Pin, I2C, WDT, Timer, RTC
import sys

# Const declarations
FIRMWARE_VERSION = '1.4'
INTERVAL_SEC = 0.25
LOOP_REFRESH_SEC = 1.0
ON = 1
OFF = 0
HIGH = 1
LOW = 0
ENABLED = 1
DISABLED = 0

# Create LED object
status_led = Pin(33, Pin.OUT, value=1)  # LED off
status_led_state = ''  # LED state "ON" or "OFF"

# Flash LED at rate defined
def blink_led(frequency=0.5, num_blinks=3):
    global status_led_state

    for _ in range(num_blinks):
        status_led.off()  # Inverted, LED on
        status_led_state = "ON"
        time.sleep(frequency)
        status_led.on()  # Inverted, LED off
        status_led_state = "OFF"
        time.sleep(frequency)

# Main loop
async def main():
    global status_led_state 
    global wlan_reconnect
    global notConnectedCounter

    # Restart visual indication
    blink_led(0.1, 5)
    
    count = 0

    while True:
        if count < 100:
            time.sleep(2)
            count += 1
        elif count >= 100:
            blink_led(0.1, 20)
            count = 0    
        
    # loop...

try:
    asyncio.run(main())
except KeyboardInterrupt:
    print('Exiting program on keyboard interrupt')
    sys.exit()
finally:
    asyncio.new_event_loop()  # Reset the event loop and return it.
