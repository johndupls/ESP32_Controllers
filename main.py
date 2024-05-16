"""
    Pond Warmer Controller with Wi-Fi V1.3
    Date:2024-01-19
    Static IP Address: 192.168.2.49
    Updates: Test button, 30 second 'ON' test
                     Timer activated - used for decrementing  all counters
                     RTC enabled & used for time, time received from NTP server
                     Controller includes relay to power 500W AC heater
"""

# Imports
import time
import network
import ntptime
import uasyncio as asyncio
from machine import Pin, I2C, WDT, Timer, RTC
import bme280
import sys
from credentials import WIFI_NAME, WIFI_PASS
import esp32

# Const declarations
FIRMWARE_VERSION = '1.3'
INTERVAL_SEC = 0.25
LOOP_REFRESH_SEC = 1.0
ON = 1
OFF = 0
HIGH = 1
LOW = 0
ENABLED = 1
DISABLED = 0
HEATER_TIMER_RUNNING = 1
HEATER_TIMER_STOPPED = 0
HEATER_ON_TEMP = 1.2  # Actual1.2, debug 25.0 Deg C
HEATER_OFF_TEMP = 5.0  # Actual5.0, debug 30.0 Deg C
UTC_OFFSET = 5 * 60 * 60  # Seconds, Ottawa offset = 5
HEATER_TEST_PERIOD = 30  # Seconds
CTRL_LIVE_PERIOD = 15 # 15 Seconds

# Global timer variables
timer_tick = False
first_pass = False
local_time = ''
ctrl_live_counter = 30 #Seconds


# Global sensor data variables
amb_temp = ""
pressure = ""
humidity = ""
sensor_status = 'Unknown'

# Global startup variable
coldstart = False

# Global web page variables
enable_color = 'grey'
disable_color = 'grey'

# Global heater variables                      
heater_onPeriod_secs = 0  # Holds the time in seconds the heater will turn on (seconds)
heater_offPeriod_secs = 0  # Holds the time in seconds the heater will turn off (seconds)

heater_onPeriod_hrs = 2.0  # Period for which the heater turns on (hours)  default=2 (hours)
heater_offPeriod_hrs = 1.0  # Period for which the heater turns on (hours)  default=1 (hours)

heater_clientOnPeriod_hrs = 0.0  # Heater on period input by client (hours)
heater_clientOffPeriod_hrs = 0.0  # Heater off period input by client, (hours)

heater_onPeriodCntr_secs = 0  # Down counter, preset with 'heater_onPeriod_secs' value during heater on (seconds)
heater_offPeriodCntr_secs = 0  # Down counter, preset with 'heater_offPeriod_secs' value during heater off (seconds)

heater_enabled_time = ''  # Time and date the heater is enabled
heater_disabled_time = ''  # Time and date the heater is disabled
heater_swon_time = ''  # Time and date when the heater is turned on
heater_swoff_time = ''  # Time and date when the heater is turned off

heater_state = ''  # Indicates if heater is 'ON' or 'OFF'
heater_on_message = ''  # Indicates if heater has 'STARTED' or 'RESTARTED'
heater_off_message = ''  # Indicates if heater has 'TURNED OFF'
heater_timer_status = ''  # Heater timer flag, indicates HEATER_TIMER_RUNNING or HEATER_TIMER_STOPPED
heater_enabled = 'DISABLED'  # Heater will turn off automatically if this flag is false "DISABLED"
heater_test = 'DISABLED'  # Heater will turn on for set time irrespective of heater_enable or temperature
heater_tempWindow_status = ''  # Shows the state of the temperature window for heater turn on "TEMP GOOD" or off "TEMP TOO HIGH"

# Global WLAN variables
ip_addr = ''
wlan_connect_time = ''  # Time and date connected to network
server_connect_state = False  # False indicates server disconnected

# I2C device addresses
bmp_addr1 = 0x76  # BMP280 address 1
bmp_addr2 = 0x77  # BMP280 alternative address 2
num_I2C_devices = 0  # Number of I2C devices detected

# Create LED object
status_led = Pin(33, Pin.OUT, value=1)  # LED off
status_led_state = ''  # LED state "ON" or "OFF"

# Create heater object
heater = Pin(5, Pin.OUT, value=0)  # GPIO5 actual controller, GPIO32 debug board, heater OFF

# Create WLAN object
wlan_connected = False
WLAN_TIMEOUT = 20  # Number of attempts to reconnect. Period = WLAN_TIMEOUT * LOOP_REFRESH_SEC
wlan = network.WLAN(network.STA_IF)

# Create I2C1 object
I2C1 = I2C(1, scl=Pin(22), sda=Pin(21))
I2C_devices = bytearray()  # Devices storage array

# Create WDT object
wdt = WDT(timeout=30000)  # 30000mSecs timeout

# Create periodic timer - 0 object
tim0 = Timer(0)

# Create RTC object
rtc = RTC()

# Configure WiFi credentials
ssid = WIFI_NAME
password = WIFI_PASS


#Cold start variable setup
def setup_variables():
    global heater_onPeriod_secs
    global heater_offPeriod_secs

    global heater_clientOnPeriod_hrs
    global heater_clientOffPeriod_hrs

    global heater_onPeriodCntr_secs
    global heater_offPeriodCntr_secs

    global heater_enabled_time
    global heater_disabled_time

    global heater_swon_time
    global heater_swoff_time

    global heater_onPeriod_hrs
    global heater_offPeriod_hrs

    global heater_state
    global heater_on_message
    global heater_off_message
    global heater_timer_status
    global heater_enabled
    global heater_test
    global heater_tempWindow_status

    global ip_addr
    global wlan_connect_time
    global server_connect_state

    global enable_color
    global disable_color

    global first_pass

    heater_onPeriod_secs = 7200  # Holds default heater on value of '2hrs' in seconds
    heater_offPeriod_secs = 3600  # Holds default heater off value of '1hrs' in seconds

    heater_clientOnPeriod_hrs = heater_onPeriod_hrs  # Heater on period received from client in hrs, default=4
    heater_clientOffPeriod_hrs = heater_offPeriod_hrs  # Heater off period received from client in hrs, default=4

    heater_onPeriodCntr_secs = heater_onPeriod_secs  # Counts down from a preset heater_onPeriod_hrs value during heater on (seconds)
    heater_offPeriodCntr_secs = heater_offPeriod_secs  # Counts down from a preset heater_offPeriod_hrs value during heater off (seconds)

    heater_enabled_time = '...'  # Date and time heater is activated
    heater_disabled_time = '...'  # Date and time heater is deactivated

    heater_swon_time = '...'  # Time and date when the heater is turned on
    heater_swoff_time = '...'  # Time and date when the heater is turned off

    heater_state = 'OFF'  # Heater on/off state
    heater_on_message = 'TURNED OFF'  # Indicates heater has 'STARTED' or 'RESTARTED'
    heater_off_message = 'TURNED OFF'  # Indicates heater has 'TURNED OFF'
    heater_timer_status = 'HEATER_TIMER_STOPPED'  # Heater timer run flag
    heater_enabled = 'ENABLED'  # Heater enable/disable flag. Set to ENABLE at startup
    heater_test = 'DISABLED'  # Heater test flag. Set to DISABLE at startup.
    heater_tempWindow_status = 'TEMP TOO HIGH'  # Heater temperature window flag, turn off heater at startup

    ip_addr = '0,0,0,0'
    wlan_connect_time = '...'
    server_connect_state = False

    enable_color = 'red'
    disable_color = 'grey'


#Tim 0 callback function
def tim0_callback(tim0):
    global ctrl_live_counter
    global  heater_onPeriodCntr_secs
    global  heater_offPeriodCntr_secs

    if ctrl_live_counter != 0: # Decrement while counters not zero
        ctrl_live_counter -= 1
        
    if heater_onPeriodCntr_secs != 0:
        heater_onPeriodCntr_secs -= 1
        
    if heater_offPeriodCntr_secs != 0:
        heater_offPeriodCntr_secs -= 1


# Create server webpage
def webpage(amb_temp, pressure,
            heater_state, heater_enabled,
            heater_on_message, heater_off_message,
            heater_clientOnPeriod_hrs, heater_clientOffPeriod_hrs,
            heater_enabled_time, heater_disabled_time,
            heater_swon_time, heater_swoff_time,
            enable_color, disable_color,
            FIRMWARE_VERSION, local_time):
    # HTML Template
    html = f"""
            <!DOCTYPE html>
            <html lang="en">
           
            <head>
                <meta charset="UTF-8">
                <meta name="viewport" content="width=device-width, initial-scale=1">
                <link rel="icon" href="data:,">
                <title>Pond Heater Monitor</title>
                <meta http-equiv="refresh" content="15">
                <!--meta http-equiv="refresh" content="15; URL=192.168.2.30"/-->
            </head>
               
            <body>
            <p><center><h2>Pond Water Heater Monitor {FIRMWARE_VERSION}</h2></center></p>
            
            <p><center>Local Date: <em>{local_time[0]}:{local_time[1]}:{local_time[2]}</em> &nbsp Local Time: <em>{local_time[4]}:{local_time[5]}:{local_time[6]}</em></center></p>
            
            <p><center><h3>Sensor Data</center></h3></p>            
            <p><center>Temperature:<em> {amb_temp}DegC</em> &nbsp Pressure:<em> {pressure}</em></center></p>
            
            <center><h3>Heater State</h3></center>
            <p><center>Heater Enabled: <em>{heater_enabled}</em> &nbsp Heater State: <em>{heater_state}</em></center></p>
            <p><center>Heater enabled at: <em>{heater_enabled_time} </em> &nbsp Heater disabled at: <em>{heater_disabled_time}</em></center></p> 
            <p><center>Heater {heater_on_message}: <em>{heater_swon_time}</em> &nbsp Heater {heater_off_message}: <em>{heater_swoff_time}</em></center></p>
            
            <center><h3>Heater Control</h3></center>
            
            <form id="0">            
            <center> <button name="HEATER" value="DISABLED" type="submit" style="
            background-color:{disable_color};
            border:4px solid #000000;
            border-radius: 15px;
            color:white;
            padding:10px 10px;
            text-align:center;
            text-decoration:none;
            display:inline-block;
            font-weight:bold;
            font-size:1.4em;
            cursor:pointer">Heater DISABLED</button></center>
            </form>
            <br>
            
            <form id="1">
            <center><label for="ON PERIOD">Heater on time:</label>
                <select name="ON PERIOD" id="ON PERIOD">
                    <option value="">--Select--</option>
                    <option value="0.02">1 mins</option>
                    <option value="0.08">5 mins</option>
                    <option value="0.25">15 mins</option>
                    <option value="0.5">30 mins</option>
                    <option value="1">1 hr</option>
                    <option value="2">2 hrs</option>
                    <option value="4">4 hrs</option>
                    <option value="8">8 hrs</option>
                    <option value="12">12 hrs</option>
                    <option value="16">16 hrs</option>
                    <option value="20">20 hrs</option>
                    <option value="24">24 hrs</option>
                </select> &nbsp; <button type="submit" form="1" value="Submit">Submit</button></center>
            </form>
            <center><p>Heater on time selected: <em>{heater_clientOnPeriod_hrs} hrs </em></p></center>
            
            <form id="2">
            <center><label for="OFF PERIOD">Heater off time:</label>
                <select name="OFF PERIOD" id="OFF PERIOD">
                    <option value="">--Select--</option>
                    <option value="0.02">1 mins</option>
                    <option value="0.08">5 mins</option>
                    <option value="0.25">15 mins</option>
                    <option value="0.5">30 mins</option>
                    <option value="1">1 hr</option>
                    <option value="2">2 hrs</option>
                    <option value="4">4 hrs</option>
                    <option value="8">8 hrs</option>
                    <option value="12">12 hrs</option>
                    <option value="16">16 hrs</option>
                    <option value="20">20 hrs</option>
                    <option value="24">24 hrs</option>
                </select> &nbsp; <button type="submit" form="2" value="Submit">Submit</button></center>       
            </form>
            <center><p>Heater off time selected: <em>{heater_clientOffPeriod_hrs} hrs</em></p></center>
            
            <form id="3">
            <center> <button name="HEATER" value="ENABLED" type="submit" style="
            background-color:{enable_color};
            border:4px solid #000000;
            border-radius: 15px;
            color:white;
            padding:10px 10px;
            text-align:center;
            text-decoration:none;
            display:inline-block;
            font-weight:bold;
            font-size:1.4em;
            cursor:pointer">Heater ENABLED</button></center> 
            </form>
            
            <br>
            
            <form id="4">
            <center>
            <button name="REFRESH" value="REFRESH" type="submit" style="
            background-color:#008CBA;
            border:4px solid #000000;
            border-radius: 15px;
            color:white;
            padding:10px 10px;
            text-align:center;
            text-decoration:none;
            display:inline-block;
            font-weight:bold;
            font-size:1.4em;
            cursor:pointer">REFRESH</button>
            
             <button name="TEST"   value="TEST" type="submit" style="
            background-color:#008CBA;
            border:4px solid #000000;
            border-radius: 15px;
            color:white;
            padding:10px 10px;
            text-align:center;
            text-decoration:none;
            display:inline-block;
            font-weight:bold;
            font-size:1.4em;
            cursor:pointer">TEST</button>
             
            <center><p>Last command issued: %s</p></center>
            </body>
            </html>
            """
    return str(html)


# Search for any devices on bus
def find_I2C_devices():
    # Scan
    print("Scanning for I2C devices")
    devices = I2C1.scan()
    if len(devices) >= 1:
        print("...{} x I2C devices found".format(len(devices)))
    else:
        print("...{} x I2C device found".format(len(devices)))
    # Return number of devices
    return devices #List of devices


# Create I2C object
def create_I2C_obj(I2C_devices):
    if bmp_addr1 in I2C_devices:
        # print("Creating BME instance for {}...\n".format(hex(bmp_addr1)))
        bmp = bme280.BME280(i2c=I2C1, address=bmp_addr1)  # Create BME280 object
    elif bmp_addr2 in I2C_devices:
        # print("Creating BME instance for {}...\n".format(hex(bmp_addr2)))
        bmp = bme280.BME280(i2c=I2C1, address=bmp_addr2)  # Create BME280 alternate object
    return bmp #bmp objrct


# Confirm I2C adresses
def check_I2C_addr(I2C_devices):
    if I2C_devices == False:
        return 0  # No devices found
    else:
        for d in I2C_devices:  # Iterate through device/s
            if d == bmp_addr1:
                print("...BME280 device found @ address={}\n".format(hex(d)))
                return 1
            elif d == bmp_addr2:
                print("...BME280 device found @ address={}\n".format(hex(d)))
                return 1
            else:
                return 2  # Unknown address found


def test_I2C():
    # Scan for I2C device/s
    I2C_devices = find_I2C_devices()

    # Compare found I2C devices to expected addresses
    result = check_I2C_addr(I2C_devices)
    if result == 1:
        bmp = create_I2C_obj(I2C_devices)
        return bmp
    elif result == 0:
        print("No I2C devices found")
    elif result == 2:
        print("Device address not recognised")
    return False


# Flash LED at rate defined
def blink_led(frequency=0.5, num_blinks=3):
    for _ in range(num_blinks):
        status_led.off()  # Inverted, LED on
        status_led_state = "ON"
        time.sleep(frequency)
        status_led.on()  # Inverted, LED off
        status_led_state = "OFF"
        time.sleep(frequency)


# Get sensor data
def get_sensor_data(bmp):
    global amb_temp
    global pressure
    global humidity
    global sensor_status

    try:
        # Get sensor data
        t = bmp.values[0]
        t = t.split('C')  # Remove 'C'
        t = t[0]  # Get only the number
        amb_temp = t

        pressure = bmp.values[1]
        # humidity = bmp.values[2]
        sensor_status = 'Sensor active'
    except:
        print("Temp sensor error...")
        sensor_status = 'Sensor error'


# Turn heater on/off depending on temperature
def check_temp_window():
    global amb_temp
    global heater_tempWindow_status

    # Check heater temperature window
    if float(amb_temp) <= HEATER_ON_TEMP:
        heater_tempWindow_status = 'TEMP GOOD'
        # print("Temp below min threshold...heater will turn on")
    elif float(amb_temp) >= HEATER_OFF_TEMP:
        heater_tempWindow_status = 'TEMP TOO HIGH'
        # print("Temp above max threshold...heater will turn off")


def setup_RTC():
    try:
        # Update the  RTC with NTP server time
        ntptime.settime()

        # Read RTC time (year, month, day, weekday, hours, minutes, seconds, subseconds)
        # print("Current UTC time retrieved from RTC  is :  ", rtc.datetime())

        # Get time in seconds since epoch
        sec = ntptime.time()

        # Calculate delta between epoch and local time zone time
        sec = int(sec - UTC_OFFSET)  # offset  = local time zone in hours * 60 * 60

        # Adjust local time
        local_time = time.localtime(sec)  # (year, month, day, hours, minutes, seconds, weekday, yearday)
        # print("Local time : ", local_time)

        # Write RTC
        (year, month, day, hours, minutes, seconds, weekday, yearday) = local_time
        # print( (year, month, day, hours, minutes, seconds, weekday, yearday))
        rtc.datetime((year, month, day, 0, hours, minutes, seconds, 0))

        # RTC value is now set to local time zone
        # print("RTC local time {} \n ".format(rtc.datetime()))
    except:
        print("RTC error")
        return False

# Connect to Wi-Fi network
async def connect_to_wifi():
    global WLAN_TIMEOUT
    global wlan_connected
    global wlan_connect_time
    global ip_addr
    global local_time

    wlan.active(True)  # Activate interface
    wlan.connect(ssid, password)

    # Wait for connect or fail
    max_wait = WLAN_TIMEOUT  # 20 secs
    while max_wait > 0 and wlan.status() != 1010:  # 1010 for ESP32
        max_wait -= 1
        print('...Waiting for connection...{}'.format(max_wait))
        time.sleep(LOOP_REFRESH_SEC)
        wdt.feed()  # Keep watch dog from triggering

    # Handle connection error
    if wlan.status() != 1010:
        blink_led(0.1, 5)
        # raise RuntimeError('WiFi connection failed')
        wlan_connected = False
        await asyncio.sleep(2)  # 2sec
    else:
        # Connection successful
        blink_led(0.5, 2)

        # Update RTC
        t = setup_RTC()
        if t != False :
            #print("Retrieving  sync'ed data from RTC")
            local_time = rtc.datetime()
        else:
            print("Using local time")
            local_time = time.localtime() 

        print("...Date: {}:{}:{}".format(local_time[0], local_time[1], local_time[2]))
        print('...WiFi Connected at {}:{}:{}'.format(local_time[4], local_time[5], local_time[6]))
        status = wlan.ifconfig()
        ip_addr = status[0]
        print('...IP Addr = {}\n'.format(ip_addr))
        wlan_connected = True
        wdt.feed()  # Keep watch dog from triggering


# Client handler
async def serve_client(reader, writer):
    global amb_temp
    global pressure
    global heater_state
    global heater_test
    global heater_enabled
    global heater_enabled_time  # Heater on date and time
    global heater_disabled_time  # Heater off date and time
    global FIRMWARE_VERSION
    global heater_onPeriod_secs  # Heater on time received from client in seconds
    global heater_offPeriod_secs  # Heater off time received from client in seconds
    global heater_onPeriodCntr_secs
    global heater_offPeriodCntr_secs
    global heater_on_message
    global heater_off_message
    global heater_swon_time
    global heater_swoff_time
    global heater_clientOnPeriod_hrs
    global heater_clientOffPeriod_hrs
    global sensor_status
    global enable_color
    global disable_color

    # print("Client connected")
    blink_led(0.1, 1)

    request_line = await reader.readline()  # Read a line
    # print("Request:", request_line)

    # We are not interested in HTTP request headers, skip them
    while await reader.readline() != b"\r\n":
        pass

    # find valid heater commands within the request
    request = str(request_line)
    print("Request:\n", request_line)

    cmd_disabled = request.find('HEATER=DISABLED')
    cmd_enabled = request.find('HEATER=ENABLED')
    cmd_onPeriod = request.find('ON+PERIOD')
    cmd_offPeriod = request.find('OFF+PERIOD')
    cmd_refresh = request.find('REFRESH=REFRESH')
    cmd_test = request.find('TEST=TEST')

    # show where the commands were found (-1 means not found)
    # print ('HEATER=DISABLED => ' + str(cmd_disabled))
    # print ('HEATER=ENABLED => ' + str(cmd_enabled))
    # print('ON+PERIOD => ' + str(cmd_onPeriod))
    # print('OFF+PERIOD => ' + str(cmd_offPeriod))
    # print('HEATER=TEST => ' + str(cmd_test))

    stateis = ""  # Keeps track of the last command issued

    # Carry out a command if it is found (found at index: 8)
    if cmd_disabled == 8:
        stateis = "Heater disabled"
        # print(stateis)
        heater_enabled = 'DISABLED'  # Clear flag
        enable_color = 'grey'
        disable_color = 'green'
        blink_led(0.1, 2)

    if cmd_enabled == 8:
        stateis = "Heater enabled"
        # print(stateis)
        heater_enabled = 'ENABLED'  # Set flag
        enable_color = 'red'
        disable_color = 'grey'
        blink_led(0.1, 2)

    if cmd_onPeriod == 8 and heater_enabled == 'DISABLED':
        t = request.split()  # Extract time value
        t = t[1]
        t = t.split('=')
        heater_clientOnPeriod_hrs = t[1]  # Hrs
        # print("On period in hours: ",heater_clientOnPeriod_hrs)
        try:
            heater_onPeriod_secs = int(
                60 * 60 * float(heater_clientOnPeriod_hrs))  # Convert hrs to seconds & update main register
            # print("Heater on period in seconds: ", heater_onPeriod_secs)
            heater_onPeriodCntr_secs = heater_onPeriod_secs  # Update on period timer counter
            stateis = "On period changed to {}hrs".format(heater_clientOnPeriod_hrs)
        except:
            print('Data input error')

    elif cmd_onPeriod == 8 and heater_enabled == 'ENABLED':
        stateis = "Can't modify heater 'ON TIME' when heater enabled"

    if cmd_offPeriod == 8 and heater_enabled == 'DISABLED':
        t = request.split()  # Extract time value
        t = t[1]
        t = t.split('=')
        heater_clientOffPeriod_hrs = t[1]  # Hrs
        # print("Off period in hours:", heater_clientOffPeriod_hrs)
        try:
            heater_offPeriod_secs = int(
                60 * 60 * float(heater_clientOffPeriod_hrs))  # Convert hrs to seconds & update main register
            # print("Heater off period: ",heater_offPeriod_secs)
            heater_offPeriodCntr_secs = heater_offPeriod_secs  # Update off period timer counter
            stateis = "Off period changed to {}hrs".format(float(heater_clientOffPeriod_hrs))
        except:
            print('Data input error')
            stateis = "Can't modify heater OFF TIME when enabled"

    elif cmd_offPeriod == 8 and heater_enabled == 'ENABLED':
        stateis = "Can't modify heater 'OFF TIME' when heater enabled"

    if cmd_refresh == 8:
        stateis = "Page refresh"
        # print(stateis)
        blink_led(0.1, 2)

    if cmd_test == 8:
    # Enter if heater disabled
        if heater_enabled == 'DISABLED':
            stateis = "Heater Test Duration: {} seconds".format(HEATER_TEST_PERIOD)
            heater_test = 'ENABLED'
            blink_led(0.1, 2)
        else:
            stateis = "TEST.....Disable heater before test"

    wdt.feed()  # Keep watch dog from triggering every second

    response = webpage(
        amb_temp, pressure,
        heater_state, heater_enabled,
        heater_on_message, heater_off_message,
        heater_clientOnPeriod_hrs, heater_clientOffPeriod_hrs,
        heater_enabled_time, heater_disabled_time,
        heater_swon_time, heater_swoff_time,
        enable_color, disable_color,
        FIRMWARE_VERSION, local_time) % stateis

    writer.write('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
    writer.write(response)

    await writer.drain()
    await writer.wait_closed()


# Turn heater element off
def heater_off():
    global heater_state
    global heater_timer_status
    global heater_offPeriodCntr_secs
    global heater_onPeriodCntr_secs

    heater_state = 'OFF'
    heater.off() 
    heater_timer_status = 'HEATER_TIMER_STOPPED'


# Turn heater element on
def heater_on():
    global heater_state
    global heater_offPeriodCntr_secs
    global heater_onPeriodCntr_secs
    global heater_offPeriod_secs
    global heater_onPeriod_secs

    heater_state = 'ON'
    heater_offPeriodCntr_secs = heater_offPeriod_secs  # Cntr preset with latest value
    heater_onPeriodCntr_secs = heater_onPeriod_secs  # Cntr preset with latest value
    heater.on()


# Main loop
async def main():
    global LOOP_REFRESH_SEC
    global server_connect_state
    global wlan_connected
    global heater_onPeriod_secs  # Heater on time received from client in seconds
    global heater_offPeriod_secs  # Heater off time received from client in seconds
    global heater_onPeriodCntr_secs
    global heater_offPeriodCntr_secs
    global heater_timer_status
    global heater_enabled
    global heater_state
    global heater_tempWindow_status
    global heater_offPeriod_hrs
    global heater_onPeriod_hrs
    global heater_enabled_time
    global heater_disabled_time
    global heater_swon_time
    global heater_swoff_time
    global heater_clientOnPeriod_hrs
    global heater_clientOffPeriod_hrs
    global heater_on_message
    global heater_off_message
    global sensor_status
    global heater_test
    global first_pass
    global local_time
    global ctrl_live_counter

    # Start timer 0
    tim0.init(period=1000, mode=Timer.PERIODIC, callback=tim0_callback)

    # Restart visual indication
    blink_led(0.1, 5)

    # Look for I2C devices
    bmp = test_I2C()
    if bmp == False:
        sensor_status = 'Sensor error'
        blink_led(1, 10)
        sys.exit()
    else:
        sensor_status = 'Sensor active'

    # Preset states
    status_led_state = 'OFF'
    coldstart = True

    # Set up outputs
    heater_off()
    status_led.off()

    # Get data
    if sensor_status == 'Sensor active':
        get_sensor_data(bmp)
    elif sensor_status == 'Sensor error':
        time.sleep(2)  # 2sec
        get_sensor_data(bmp)  # Try again
        if sensor_status == 'Sensor error':
            blink_led(1, 15)
            sys.exit()  # Reset

    wdt.feed()  # Keep watch dog from triggering

    while True:
        # Tasks on cold start
        if coldstart == True:
            # Preset variables
            setup_variables()
            coldstart = False

        if not wlan_connected:
            print('Connecting to WiFi')
            asyncio.create_task(connect_to_wifi())

        if wlan_connected and (server_connect_state == False):
            '''
                Start a TCP server on the given host and port. The callback will be called with incoming, accepted connections,
                and be passed 2 arguments: reader and writer streams for the connection. Returns a Server object.
            '''
            print('Creating webserver')
            asyncio.create_task(asyncio.start_server(serve_client, "0.0.0.0", 80))
            print('...Web server ready...\n')
            server_connect_state = True

        await asyncio.sleep(LOOP_REFRESH_SEC)  # 1sec

        # Update RTC  global storage variable
        local_time = rtc.datetime()
        
        #Check contoller alive counter
        if ctrl_live_counter == 0 :
            blink_led(frequency=0.05, num_blinks=1) #Flash LED
            ctrl_live_counter = CTRL_LIVE_PERIOD #Preset counter

        
        # Refresh data
        if sensor_status == 'Sensor active':
            get_sensor_data(bmp)
        elif sensor_status == 'Sensor error':
            time.sleep(2)  # 2sec
            get_sensor_data(bmp)  # Try again
            if sensor_status == 'Sensor error':
                blink_led(1, 15)
                sys.exit()  # Reset

        # Check heater temperature window
        check_temp_window()

        # Check heater test flag
        if heater_test == 'ENABLED':
            if first_pass == False:
                #print("Turning on heater for test")
                heater_onPeriod_secs = HEATER_TEST_PERIOD  # Preset on period
                heater_offPeriod_secs = 0  # Preset off period
                heater_on()  # Turn on heater
                result = "{}:{}:{}".format(local_time[4], local_time[5], local_time[6])  # Get RTC time
                if result == '':
                    heater_swon_time = 'Time unavailable...'
                else:
                    heater_swon_time = result
                    heater_swoff_time = '...'
                first_pass = True
            elif heater_test == 'ENABLED' and first_pass == True and heater_onPeriodCntr_secs != 0:
                #print("heater_onPeriodCntr_secs:", heater_onPeriodCntr_secs)
                pass
            else: 
                #print("Heater test complete, turning off heater")
                heater_off()  # Heater turned off
                result = "{}:{}:{}".format(local_time[4], local_time[5], local_time[6])  # Get RTC time
                if result == '':
                    heater_swoff_time = 'Time unavailable...'
                else:
                    heater_swoff_time = result
                heater_test = 'DISABLED'
                first_pass = False
            wdt.feed()  # Keep watch dog from triggering every second

        # Check heater enabled flag
        if heater_enabled == 'ENABLED' and heater_state == 'OFF' and heater_timer_status == 'HEATER_TIMER_STOPPED' and heater_tempWindow_status == 'TEMP GOOD':
            # print("Turning on heater...heater ENABLED, heater OFF, heater timer STOPPED, heater temp below min threshold")
            heater_on()  # Turn on heater
            result = "{}:{}:{}".format(local_time[4], local_time[5], local_time[6])  # Get RTC time
            if result == '':
                heater_swon_time = 'Time unavailable...'
                heater_enabled_time = 'Time unavailable...'
            else:
                heater_swon_time = result
                heater_enabled_time = result
            heater_disabled_time = '...'
            heater_swoff_time = '...'

            # print("Heater started",heater_swon_time)
            if heater_state == 'ON':
                heater_timer_status = 'HEATER_TIMER_RUNNING'  # Start timer
                heater_on_message = 'turned on at'
                heater_off_message = 'turned off at'
                # print("Heater timer running")

        if heater_enabled == 'ENABLED' and (heater_state == 'ON' or heater_state == 'OFF') and heater_tempWindow_status == 'TEMP TOO HIGH':
            heater_off()  # Turn off heater
            result = "{}:{}:{}".format(local_time[4], local_time[5], local_time[6])  # Get RTC time
            if result == '':
                heater_swoff_time = 'Time unavailable...'
            else:
                heater_swoff_time = result
            heater_on_message = 'turned on at'
            heater_off_message = 'Temperature too high...element turned off at!'
            heater_timer_status = 'HEATER_TIMER_STOPPED'
            # print("Heater ENABLED, heater OFF, heater state == ON/OFF, heater temp window above max threshold")

        if heater_enabled == 'DISABLED' and  heater_state == 'ON'  and  heater_test == 'DISABLED':
            heater_off()
            result = "{}:{}:{}".format(local_time[4], local_time[5], local_time[6])  # Get RTC time
            if result == '':
                heater_swoff_time = 'Time unavailable...'
                heater_disabled_time = 'Time unavailable...'
            else:
                heater_swoff_time = result
                heater_disabled_time = result
            heater_on_message = 'turned on at'
            heater_off_message = 'heater disabled...turned off at!'
            heater_timer_status = 'HEATER_TIMER_STOPPED'
            # print("Heater DISABLED, heater OFF, heater state = ON")

        if heater_enabled == 'DISABLED' and heater_state == 'OFF' and  heater_test == 'DISABLED' :
            if heater_timer_status == 'HEATER_TIMER_RUNNING':
                result = "{}:{}:{}".format(local_time[4], local_time[5], local_time[6])  # Get RTC time
                if result == '':
                    heater_disabled_time = 'Time unavailable...'
                else:
                    heater_disabled_time = result
                heater_swon_time = '...'
                heater_swoff_time = '...'
            # print("Heater DISABLED, heater state = OFF")
            heater_on_message = 'turned on at'
            heater_off_message = 'turned off at'
            heater_timer_status = 'HEATER_TIMER_STOPPED'

        # Task when heater enabled and timer active
        if heater_timer_status == 'HEATER_TIMER_RUNNING' and heater_state == 'ON':
            if heater_onPeriodCntr_secs != 0:
                # print("heater_onPeriodCntr_secs", heater_onPeriodCntr_secs)
                #heater_onPeriodCntr_secs = heater_onPeriodCntr_secs - 1  # Decrement on period cntr
                pass
            elif heater_onPeriodCntr_secs == 0:
                heater.off()  # Heater turned off
                heater_state = 'OFF'
                result = "{}:{}:{}".format(local_time[4], local_time[5], local_time[6])  # Get RTC time
                if result == '':
                    heater_swoff_time = 'Time unavailable...'
                else:
                    heater_swoff_time = result
                # print("heater_turned off: {}".format(result))
                heater_offPeriodCntr_secs = heater_offPeriod_secs  # Cntr preset
                # heater_enabled_time = ''

        elif heater_timer_status == 'HEATER_TIMER_RUNNING' and heater_state == 'OFF':
            if heater_offPeriodCntr_secs != 0:
                # print("heater_offPeriodCntr_secs", heater_offPeriodCntr_secs)
                #heater_offPeriodCntr_secs = heater_offPeriodCntr_secs - 1  # Decrement off period cntr
                pass
            elif heater_offPeriodCntr_secs == 0:
                heater.on()  # Heater turned on
                heater_state = 'ON'
                result = "{}:{}:{}".format(local_time[4], local_time[5], local_time[6])  # Get RTC time
                if result == '':
                    heater_swon_time = 'Time unavailable...'
                else:
                    heater_swon_time = result
                # print("heater_re-started: {}".format(result))
                heater_onPeriodCntr_secs = heater_onPeriod_secs  # Cntr preset
                # heater_disabled_time = ''

        if wlan.isconnected() == False:
            print('Network disconnected...')
            await asyncio.sleep(5)  # 5sec
            print('Confirming network disconnected')
            if wlan.isconnected() == False:  # Confirm disconnected
                # Config for retry
                wlan_connected = False  # Re-connect flag cleared
                server_connect_state = False  # Server flag cleared

        wdt.feed()  # Keep watch dog from triggering every second
    # loop...


try:
    asyncio.run(main())
except KeyboardInterrupt:
    print('Exiting program on keyboard interrupt')
    sys.exit()
finally:
    asyncio.new_event_loop()  # Reset the event loop and return it.
