"""
    Pond Warmer Controller with Wi-Fi 
    Version: V1.51
    Date: 2025-05-31
    Static IP Address: 192.168.2.49
    
    Updates V1.4:
                    Test button, 30 second 'ON' test
                    Timer activated - used for decrementing  all counters
                    RTC enabled & used for time (time sourced from NTP server)
                    Controller includes relay to power 500W AC heater
                    ESP resets after 10 attempts to connect to WiFi
                    Client refresh time constant added. Set to 30 secs
                    Add humidity and dew point to client web page
                    Add OTA programming
                    Set up to use static address
                    Added garbage collection to avoid memory overruns
    Updates V1.51:
                    Add buttons to turn on/off spot light
                    Add auto and manual mode for the light control
                    Timer control over how long light stays on in auto mode, manual mode light stays on continiously
                    Check LDR to check if dark enough to turn on light in both modes
                    Heater turns on when temperature drops below the minimum and stays on continiously, no longer
                    controlled from the web page.
"""

# Imports
import time
import network
import ntptime
import uasyncio as asyncio
import machine
from machine import Pin, I2C, WDT, Timer, RTC
import bme280
import sys
from credentials import WIFI_NAME, WIFI_PASS
import gc
import errno

# Const declarations
FIRMWARE_VERSION = '1.51'
WLAN_TIMEOUT = 20  # Number of attempts to connect. Period = WLAN_TIMEOUT * LOOP_REFRESH_SEC
LOOP_REFRESH_SEC = 2.0
REQ_TIMEOUT = 20
ON = 1
OFF = 0
HIGH = 1
LOW = 0
ENABLED = 1
DISABLED = 0
HEATER_ON_TEMP = 0.5  # Actual 0.5, debug 25.0 Deg C
HEATER_OFF_TEMP = 3.0  # Actual 3.0, debug 30.0 Deg C
LED_LIGHT_ON_PERIOD = 7200  # 2*60*60 seconds
UTC_OFFSET = 4 * 60 * 60  # Seconds, Ottawa offset = 4/5
PERIPHERAL_TEST_PERIOD = 60  # Seconds
CLIENT_REFRESH_PERIOD = 15 # Seconds
CTRL_LIVE_PERIOD = 15 # 15 Seconds
GC_TIMEOUT = 1800 # 1800 Seconds (30mins)
STATIC_ADDR = '192.168.2.49'
LIGHT = 1
DARK = 0
BUSY = 1
DONE = 0

# Global timer variables
first_pass = False
local_time = ''
ctrl_live_counter = CTRL_LIVE_PERIOD # 15 Seconds

# Garbage collection timeout
gc_timeout_counter = GC_TIMEOUT

# Global controller variables
unit_id = ''

# Global sensor data variables
amb_temp = ''
pressure = ''
humidity = ''
dew_point = ''
sensor_status = 'Unknown'
interrupt_pin = 0

# Global startup variable
coldstart = False

# Global heater variables
heater_button_color = 'red'
heater_action = 'On'
heater_swon_time = '...'  # Time and date when the heater is turned on
heater_run_time = '...'  # Time and date when the heater is turned off
heater_tempWindow_status = ''  # Shows the state of the temperature window for heater turn on "TEMP GOOD" or off "TEMP TOO HIGH"
heater_sec_count = 0
heater_min_count = 0
heater_hour_count = 0
heater_day_count = 0
heater_time_unit = '' # Web page heater on time unit

# Global test variables
peripheral_onPeriodCntr_secs = 0  # Down counter, preset with 'PERIPHERAL_TEST_PERIOD' value during test on (seconds)
peripheral_test = 'DISABLED'  # Heater will turn on for set time irrespective of heater_enable or temperature

# Global LED light variables
led_light_button_color = 'red'
led_light_button_action = 'On'      
led_light_onPeriodCntr_secs = 0  # Down counter, preset with 'light_onPeriod_secs' value during light on (seconds)
led_light_flag = HIGH
auto_button_action = 'On'
auto_button_color = 'red'
auto_mode = False
manual_mode = False

# Global WLAN variables
ip_addr = ''
wlan_connect_time = ''  # Time and date connected to network
wlan_disconnect_time = '' # Time and date disconnected from network
server_connect_state = False  # False indicates server disconnected
notConnectedCounter = 0 # Number of re-connect attempts
wlan_reconnect = False
wlan_connected = False
rssi = '' # WiFi recieved signal strength

# I2C device addresses
bmp_addr1 = 0x76  # BMP280 address 1
bmp_addr2 = 0x77  # BMP280 alternative address 2
num_i2c_devices = 0  # Number of I2C devices detected

# Create LED object
status_led = Pin(33, Pin.OUT, value=1)  # By default LED off
status_led_state = 'OFF'  # LED state "ON" or "OFF"

# Create 12V DC heater object
heater = Pin(5, Pin.OUT, value=0)  # By default heater off
heater_state = 'OFF' # Indicates if heater is 'ON' or 'OFF'

# Create LED spot light object
led_light = Pin(32, Pin.OUT, value=0)  # By default light off
led_light_state = 'OFF'

# Create LDR light sensor object
ldr_sensor = Pin(35, Pin.IN)  # By default light off
ldr_sensor_state = 'LIGHT' # Spot light off, ambient light, LDR comparator output high
ldr_int_flag = LOW

# Create WLAN object
wlan = network.WLAN(network.STA_IF)

# Create I2C1 object
i2c1 = I2C(1, scl=Pin(22), sda=Pin(21))
i2c_devices = bytearray()  # Devices storage array

# Create WDT object
wdt = WDT(timeout=30000)  # 30000mSecs timeout

# Create periodic timer - 0 object
tim0 = Timer(0)

# Create RTC object
rtc = RTC()

# Configure WiFi credentials
ssid = WIFI_NAME
password = WIFI_PASS

# Enable garbage collection
gc.enable()
    
# Cold start variable setup...
def setup_variables():
    global peripheral_onPeriodCntr_secs
    global heater_swon_time
    global heater_run_time
    global heater_state
    global heater_tempWindow_status
    global heater_button_color
    global heater_action
    global heater_sec_count
    global heater_min_count
    global heater_hour_count
    global heater_day_count
    global heater_time_unit
    
    global peripheral_test 
    global ldr_sensor_state
    global ambient_light

    global ip_addr
    global wlan_connect_time
    global server_connect_state

    global led_light_onPeriodCntr_secs
    global led_light_button_action  
    global led_light_button_color
    global led_light_state
    global led_light_flag
    global auto_button_color
    global auto_button_action
    global auto_mode 
    global manual_mode 

    peripheral_onPeriodCntr_secs = 0  # Counts down from a preset value in seconds during test
    heater_swon_time = '...'  # Time and date when the heater is turned on
    heater_run_time = '...'  # Time and date when the heater is turned off
    heater_tempWindow_status = 'TEMP TOO HIGH'  # Heater temperature window flag, turn off heater at startup
    heater_state = 'OFF'  # Heater on/off state
    heater_time_unit = ''
    
    peripheral_test = 'DISABLED'  # Heater test flag. Set to DISABLE at startup.
    ldr_sensor_state = 'LIGHT'
    
    led_light_onPeriodCntr_secs = 0 # Counts down from a preset value in seconds during auto mode
    led_light_button_color = 'red'
    led_light_button_action = 'On'
    led_light_state = 'OFF'
    led_light_flag = HIGH
    
    auto_button_color = 'red'
    auto_button_action = 'On'
    auto_mode = False
    manual_mode = False
    ambient_light = 'LIGHT'

    ip_addr = '0,0,0,0'
    wlan_connect_time = '...'
    server_connect_state = False
    
# Get unit ID...
def  get_id():
    global unit_id
    
    id = STATIC_ADDR
    id = id.split(".")
    unit_id = id[3] # The last value in the address is used for the ID

# Tim 0 callback function...
def tim0_callback(tim0):
    global ctrl_live_counter
    global  peripheral_onPeriodCntr_secs
    global led_light_onPeriodCntr_secs
    global gc_timeout_counter
    global heater_sec_count
    global heater_min_count
    global heater_hour_count
    global heater_day_count

    # Decrement while counters not zero
    if ctrl_live_counter != 0: # Keep alive counter
        ctrl_live_counter -= 1
        
    if peripheral_onPeriodCntr_secs > 0:
        peripheral_onPeriodCntr_secs -= 1
        
    if led_light_onPeriodCntr_secs > 0:
        led_light_onPeriodCntr_secs -= 1
        
    if gc_timeout_counter > 0: # Garbage collection after 30 seconds
        gc_timeout_counter -= 1
    
    # Increment counters while heater is on
    if heater_state == 'ON':
        heater_sec_count += 1 # Increment every second
        if heater_sec_count == 60:
            heater_min_count += 1 # Increment every min
            heater_sec_count = 0
        if heater_min_count == 60:
            heater_hour_count += 1 # Increment every hour
            heater_min_count = 0
        if heater_hour_count == 24:
            heater_day_count += 1 # Increment every day
            heater_hour_count = 0
        if heater_day_count > 365: # Should never reach this maximum
            heater_day_count = 0
             
# Get recieved signal strength...
def get_rssi():
    global rssi
    
    result = wlan.status('rssi')
    if result <=-50 and result >= -64:
        print('RSSI...strong signal: {}dBm'.format(result))
    elif result <= -65 and result >= -79:
        print('RSSI...moderate signal: {}dBm'.format(result))
    elif result <= -80:
        print('RSSI...exceeding minimum acceptable signal for connection: {}dBm'.format(result))
    print('\n')
    rssi = str(result)

# Create server webpage...
def webpage(
            amb_temp, pressure, humidity, dew_point,
            heater_state, heater_swon_time, heater_run_time, heater_time_unit,
            auto_button_color, auto_button_action,
            ldr_sensor_state, led_light_state, led_light_button_color, led_light_button_action,
            FIRMWARE_VERSION, unit_id, rssi, local_time):
    # HTML Template
    html = f"""
            <!DOCTYPE html>
            <html lang="en">
           
            <head>
                <meta charset="UTF-8">
                <meta name="viewport" content="width=device-width, initial-scale=1">
                <link rel="icon" href="data:,">
                <title>Pond Heater Monitor</title>      
                <meta http-equiv="refresh" content={CLIENT_REFRESH_PERIOD}>
                <!--meta http-equiv="refresh" content="15; URL=192.168.2.30"/-->
            </head>
               
            <body>
                <body style="background-color: #C0C0C0;">
            
                <p><center><h3>POND WATER HEATER CTRL {FIRMWARE_VERSION}</h3></center></p>

                <p><center>Date: <em>{local_time[0]}:{local_time[1]}:{local_time[2]}</em> &nbsp Time: <em>{local_time[4]}:{local_time[5]}:{local_time[6]}</em></center></p>
                <p><center>ID: <em>{unit_id}</em> &nbsp RSSI: <em>{rssi}dBm</em></center></p>
            
                <p><center><h4>SENSOR DATA</center></h4></p>            
                <p><center>Temperature:<em> {amb_temp}DegC</em> &nbsp Pressure:<em> {pressure}</em></center></p>
                <p><center>Humidity:<em> {humidity}&percnt;</em> &nbsp Dew point:<em> {dew_point}DegC</em></center></p>

                <center><h4>HEATER CONTROL</h4></center>
                <p><center>Heater State: <em>{heater_state}</em></center></p>
                <p><center>Heater On Time: <em>{heater_swon_time}</em> &nbsp Heater Run Time: <em>{heater_run_time}{heater_time_unit}</em></center></p>
            
                <center><h4>LIGHT CONTROL</h4></center>
                <p><center>Ambient: <em>{ldr_sensor_state}</em> &nbsp Light State: <em>{led_light_state}</em></center></p>
            
                <form id="1">
                    <center>&nbsp; Auto Mode&nbsp;&nbsp;
                    <button name="AUTO" value="TOGGLE" type="submit" style="
                    background-color:{auto_button_color};
                    border:4px double #000000;
                    border-radius: 10px;
                    color:white;
                    padding:8px 15px;
                    text-align:center;
                    text-decoration:none;
                    display:inline-block;
                    font-weight:bold;
                    font-size:1.4em;
                    cursor:pointer">{auto_button_action}</button>
                </form>
            
                <br>
                <br>
               
                <form id="2">
                    <center>Manual Mode
                    <button name="POND LIGHT" value="TOGGLE" type="submit" style="
                    background-color:{led_light_button_color};
                    border:4px double #000000;
                    border-radius: 10px;
                    color:white;
                    padding:8px 15px;
                    text-align:center;
                    text-decoration:none;
                    display:inline-block;
                    font-weight:bold;
                    font-size:1.4em;
                    cursor:pointer">{led_light_button_action}</button>
                </form>

                <br>
            
                <center><h4>UTILITIES</h4></center>
                <form id="3">
                    <center>Refresh Page
                    <button name="REFRESH" value="REFRESH" type="submit" style="
                    background-color:#008CBA;
                    border:4px double #000000;
                    border-radius: 10px;
                    color:white;
                    padding:8px 4px;
                    text-align:center;
                    text-decoration:none;
                    display:inline-block;
                    font-weight:bold;
                    font-size:1.4em;
                    cursor:pointer">Update</button>
                </form>
            
                <br>
                <br>
            
                <form id="4">
                    <center>Test Peripherals
                    <button name="TEST"   value="TEST" type="submit" style="
                    background-color:#008CBA;
                    border:4px double #000000;
                    border-radius: 10px;
                    color:white;
                    padding:8px 20px;
                    text-align:center;
                    text-decoration:none;
                    display:inline-block;
                    font-weight:bold;
                    font-size:1.4em;
                    cursor:pointer">Test</button>
                    &nbsp;&nbsp;&nbsp;
                </form>
             
                <center><p>Last command issued: %s</p></center>
            </body>
            </html>
            """
            #border:3px solid #000000;
    return str(html)

# Search for any devices on bus...
def find_i2c_devices():
    # Scan
    print('\n' + 'Scanning for I2C devices')
    devices = i2c1.scan()
    if len(devices) >= 1:
        print('...{} x I2C devices found'.format(len(devices)))
    else:
        print('...{} x I2C device found'.format(len(devices)))
    # Return number of devices
    return devices #List of devices

# Create I2C object...
def create_i2c_obj(i2c_devices):
    if bmp_addr1 in i2c_devices:
        # print("Creating BME instance for {}...\n".format(hex(bmp_addr1)))
        bmp = bme280.BME280(i2c=i2c1, address=bmp_addr1)  # Create BME280 object
    elif bmp_addr2 in i2c_devices:
        # print("Creating BME instance for {}...\n".format(hex(bmp_addr2)))
        bmp = bme280.BME280(i2c=i2c1, address=bmp_addr2)  # Create BME280 alternate object
    return bmp #bmp object

# Confirm I2C adresses...
def check_i2c_addr(i2c_devices):
    if i2c_devices == False:
        return 0  # No devices found
    else:
        for d in i2c_devices:  # Iterate through device/s
            if d == bmp_addr1:
                print('...BME280 device found @ address={}\n'.format(hex(d)))
                return 1
            elif d == bmp_addr2:
                print('...BME280 device found @ address={}\n'.format(hex(d)))
                return 1
            else:
                return 2  # Unknown address found

# Validate I2C addresses found...
def test_i2c():
    # Scan for I2C device/s
    i2c_devices = find_i2c_devices()

    # Compare found I2C devices to expected addresses
    result = check_i2c_addr(i2c_devices)
    if result == 1:
        bmp = create_i2c_obj(i2c_devices)
        return bmp
    elif result == 0:
        print('No I2C devices found')
    elif result == 2:
        print('Device address not recognised')
    return False

# Get sensor data...
def get_sensor_data(bmp):
    global amb_temp
    global pressure
    global humidity
    global sensor_status
    
    try:
        # Get temperature sensor data (value only)
        t = bmp.values[0]
        t = t.split('C')  # Remove 'C'
        t = t[0]  # Get only the number
        amb_temp = t
        
        # Get pressure sensor  data
        pressure = bmp.values[1]

        # Get humidity sensor  data (value only)
        h = bmp.values[2]
        h = h.split('%')  # Remove '%'
        h = h[0]  # Get only the number
        humidity = h
        
        sensor_status = 'Sensor active'
        
    except:
        print('Temp sensor error...')
        sensor_status = 'Sensor error'

# Get LDR output value...
def read_ldr():
    # Read comparator output
    state = ldr_sensor.value() # Read comparator output
    if state == DARK: # Ambient dark
        time.sleep_ms(1) # 10ms delay 
        state = ldr_sensor.value() # Check again after delay to confirm valid activation
        if state == DARK:
            return 'DARK'
    else:
        return 'LIGHT'

# Calculate dew point...
def dew_point_calc():
    global amb_temp
    global humidity
    global dew_point
    temp = 0.0
    
    temp = float(amb_temp) - ((100-float(humidity))/5) # Dew point calculation
    temp = round(temp,2) # Round up number
    dew_point = str(temp)
    
# Check heater temperature window...
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

# Setup RTC...
def setup_RTC():
    global local_time
    
    try:
        # Update the  RTC with NTP server time
        ntptime.settime()

        # Read RTC time (year, month, day, weekday, hours, minutes, seconds, subseconds)
        # print("Current UTC time retrieved from RTC  is :  ", rtc.datetime())

        # Get time in seconds since epoch
        sec = ntptime.time()

        # Calculate delta between epoch and local time zone time
        delta = int(sec - UTC_OFFSET)  # offset  = local time zone in hours * 60 * 60

        # Adjust local time
        local_time = time.localtime(delta)  # (year, month, day, hours, minutes, seconds, weekday, yearday)
        # print("Local time : ", local_time)

        # Write RTC
        (year, month, day, hours, minutes, seconds, weekday, yearday) = local_time
        # print( (year, month, day, hours, minutes, seconds, weekday, yearday))
        rtc.datetime((year, month, day, 0, hours, minutes, seconds, 0))

        # RTC value is now set to local time zone
        # print("RTC local time {} \n ".format(rtc.datetime()))
        return 'RTC_good'
    except:
        print('RTC error')
        return False

# Connect to Wi-Fi network...
async def connect_to_wifi():
    global wlan_connected
    global wlan_connect_time
    global ip_addr
    global local_time

    # Keep watch dog from triggering
    wdt.feed()  

    wlan.active(True)  # Activate interface
    wlan.ifconfig( (STATIC_ADDR, '255.255.255.0', '192.168.2.1', '192.168.2.1')) # Set static address, subnet, gateway & dns
    wlan.connect(ssid, password) # Connect

    # Wait for connect or fail
    max_wait = WLAN_TIMEOUT  # 20 secs
    while max_wait > 0 and wlan.status() != 1010:  # 1010 for ESP32
        max_wait -= 1
        print('...Waiting for connection...{}'.format(max_wait))
        await asyncio.sleep(0.5)

    # Handle connection error
    if wlan.status() != 1010:
        blink_led(0.1, 5)
        wlan_connected = False
        await asyncio.sleep(0.5)
    else:
        # Connection successful
        blink_led(0.1, 2)

        # Update RTC
        t = setup_RTC()
        if t != False :
            #print("Retrieving  sync'ed data from RTC")
            local_time = rtc.datetime()
        else:
            print('Using local time')
            local_time = time.localtime()
            
        print('...Date: {}:{}:{}'.format(local_time[0], local_time[1], local_time[2]))
        print('...WiFi Connected at {}:{}:{}'.format(local_time[4], local_time[5], local_time[6]))
        
        #Record time connected
        wlan_connect_time = str(local_time[0]) + ':' + str(local_time[1])  + ':' +  str(local_time[2])  + '...' +  str(local_time[4])  + ':' +  str(local_time[5]) + ':' +  str(local_time[6])
        
        # Get WLAN parameters
        status = wlan.ifconfig()
        ip_addr = status[0]
        print('...WLAN parameters: {}\n'.format(wlan.ifconfig()) )
        
        # Set connected flag
        wlan_connected = True
        
        # Keep watch dog from triggering
        wdt.feed()  
        

# Client handler...
async def serve_client(reader, writer):
    global amb_temp
    global pressure
    global humidity
    global dew_point
    
    global heater_state
    global peripheral_test
    global heater_enabled
    global heater_swon_time
    global heater_run_time
    global heater_time_unit
    
    global sensor_status
    
    global led_light_button_color
    global led_light_button_action
    global led_light_state
    global led_light_onPeriodCntr_secs
    global ldr_sensor_state
    global ldr_int_flag
    global led_light_flag
    
    global auto_button_color
    global auto_button_action
    global auto_mode
    global manual_mode
    
    global unit_id
    
    wdt.feed()  # Keep watch dog from triggering
    
    print("Client connected")
    blink_led(0.1, 1)

    req_timeout = REQ_TIMEOUT # 20 secs
    try:
        #print('Reading request')
        request_line = await reader.readline()  # Read a line
     
        # We are not interested in HTTP request headers, skip them
        while await reader.readline() != b"\r\n":
            if req_timeout != 0:
                req_timeout -= 1
                continue
            else:
                break
    except OSError as exc:
        if exc.errno == errno.ECONNRESET:
            print('Connection reset by peer')
        elif exc.errno == errno.ECONNABORTED:
            print('Software caused connection abort')
        return         

    # find valid heater commands within the request
    request = str(request_line)
    #print('Request recieved:\n', request_line)

    cmd_auto_button_action = request.find('AUTO=TOGGLE')
    cmd_led_light_button_action = request.find('POND+LIGHT=TOGGLE')
    cmd_refresh = request.find('REFRESH=REFRESH')
    cmd_test = request.find('TEST=TEST')

    # show where the commands were found (-1 means not found)
    #print ('AUTO => ' + str(cmd_auto_button_action))
    #print ('LIGHT+TOGGLE => ' + str(cmd_led_light_button_action))
    #print('FRESH =>' +str(cmd_refresh))
    # print('HEATER=TEST => ' + str(cmd_test))

    stateis = ''  # Keeps track of the last command issued

    # LED auto light action
    if cmd_auto_button_action == 8:
        if manual_mode == False:
            if auto_button_action == 'On': # Set to auto mode
                stateis = 'Auto mode on'
                auto_button_color = 'green'
                auto_button_action = 'Off'
                auto_mode = True
                led_light_button_color = 'grey'
                led_light_flag = LOW # Set low to allow the light to come on without the daily LDR transition
            elif auto_button_action == 'Off': # Set to manual mode
                stateis = 'Auto mode off'
                auto_button_color = 'red'
                auto_button_action = 'On'
                auto_mode = False
                led_light_button_color = 'red'
                setup_led_light(OFF) # Turn off light
            blink_led(0.1, 2)
            
    # LED manual light action
    if cmd_led_light_button_action == 8:
        if auto_mode == False:
            if led_light_button_action == 'On' and ldr_sensor_state == 'DARK': # Light on
                stateis = 'Light on'
                led_light_button_color = 'green'
                led_light_button_action = 'Off'
                auto_button_color = 'grey'
                manual_mode = True
            elif led_light_button_action == 'Off': # Light off
                stateis = 'Light off'
                setup_led_light(OFF) # Turn off light
                led_light_button_color = 'red'
                led_light_button_action = 'On'
                auto_button_color = 'red'
                manual_mode = False
            blink_led(0.1, 2)

    # Refresh action
    if cmd_refresh == 8:
        stateis = 'Page refresh'
        blink_led(0.1, 2)

    # Test action
    if cmd_test == 8:
        stateis = 'Peripheral Test Duration: {} seconds'.format(PERIPHERAL_TEST_PERIOD)
        peripheral_test = 'ENABLED'
        blink_led(0.1, 2)
        
    wdt.feed()  # Keep watch dog from triggering every second

    # Free memory
    gc.collect() #Run garbage collection
    free_mem = gc.mem_free()
    #print('Memory freed for response: ', free_mem)
    if free_mem < 10000:
        print('Not enough memory for response: ', free_mem)
        return

    try:
        #print('Sending response')
        response = webpage(
            amb_temp, pressure, humidity, dew_point,
            heater_state, heater_swon_time, heater_run_time, heater_time_unit,
            auto_button_color, auto_button_action,
            ldr_sensor_state, led_light_state, led_light_button_color, led_light_button_action,
            FIRMWARE_VERSION, unit_id, rssi, local_time) % stateis
        writer.write('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
        writer.write(response)
        await writer.drain()
        await writer.wait_closed()
        print("Client disconnected")
    except OSError as exc:
        # Free memory
        if exc.errno == errno.ENOMEM:
            gc.collect()
            print("Error...freeing memory: ", gc.mem_free())

# Turn status LED on or off...
def setup_status_led(action):
    global status_led_state
    
    if action == ON:
        status_led.off() # Inverted, LED on
        status_led_state = 'ON'
    elif action == OFF:
        status_led.on() # Inverted, LED off
        status_led_state = 'OFF'
        
# Flash status LED at rate defined...
def blink_led(frequency=0.5, num_blinks=3):
    for _ in range(num_blinks):
        setup_status_led(ON)  # Turn on
        time.sleep(frequency)
        setup_status_led(OFF)  # Turn off
        time.sleep(frequency)

# Turn heaters (AC and DC) on or off...
def setup_heater(action):
    global heater_state
    
    if action == ON: 
        heater.on() # Turn on
        heater_state = 'ON'
    elif action == OFF:
        heater.off() # Turn off
        heater_state = 'OFF'

# Turn LED spot light on or off...
def setup_led_light(action):
    global led_light_state
    
    if action == ON: 
        led_light.on() # Turn on
        led_light_state = 'ON'
    elif action == OFF:
       led_light.off() # Turn off
       led_light_state = 'OFF'
        
# Get wlan status code...
def wlan_test():
     # Handle connection status
    if  wlan.status() != 1010:
        print('WiFi connection error') 
    elif wlan.status() == 1000:
        print('Link down, no connection and no activity') 
    elif wlan.status() == 1001:
            print('Link join, connecting in progress')
    elif wlan.status() == 200:
            print('Link noip')
    elif wlan.status() == 203:
            print('Link fail, failed due to other problems')
    elif wlan.status() == 201:
            print('Link nonet, failed because no access point replied')
    elif wlan.status() == 202:
            print('Link badauth, failed due to incorrect password')
    elif wlan.status() == 204:
            print('Link handshake timeout')
    else:
        print("WLAN connected, status = ", 1010)
    return wlan.status()

# LDR sensor interrupt handler...
def ldr_sensor_interrupt_handler(pin):
    global ldr_int_flag
    global interrupt_pin
    
    ldr_int_flag = HIGH # Set high every time an interrupt occurs
    interrupt_pin = pin # Pin is the GPIO that issued the interrupt
    
# Main loop
async def main():
    global server_connect_state
    global wlan_connected
    global wlan_reconnect
    global notConnectedCounter
    global wlan_disconnect_time
    global wlan_connect_time
    
    global peripheral_onPeriodCntr_secs
    global heater_state
    global heater_tempWindow_status
    global heater_swon_time
    global heater_run_time
    global heater_sec_count
    global heater_min_count
    global heater_hour_count
    global heater_day_count
    global heater_time_unit
    
    global sensor_status
    global status_led_state
    global peripheral_test
    global first_pass
    global local_time
    global ctrl_live_counter
    global gc_timeout_counter
    global coldstart

    global  ldr_sensor_state
    global ldr_int_flag    
    global led_light_button_action
    global led_light_button_color
    global led_light_state
    global led_light_onPeriodCntr_secs
    global led_light_flag
    global auto_mode
    global auto_button_color
    global manual_mode

    # Preset flags
    coldstart = True
    first_pass = False
    
    # Preset GPIO input output states
    ldr_sensor_state = 'LIGHT'
    
    # Set up GPIO outputs/inputs
    setup_heater(OFF)
    setup_status_led(OFF)
    setup_led_light(OFF)
    
    # Start timer 0
    tim0.init(period=1000, mode=Timer.PERIODIC, callback=tim0_callback) # 1sec
    
    # Cold start visual indication
    blink_led(0.1, 5)
    
    # Search for I2C devices
    bmp = test_i2c()
    if bmp == False:
        sensor_status = 'Sensor error'
        blink_led(1, 10)
        print('Exiting application:', sensor_status)
        sys.exit() # Reset
    else:
        sensor_status = 'Sensor active'
    
    # Get sensor data
    if sensor_status == 'Sensor active':
        get_sensor_data(bmp)
        dew_point_calc()    # Calculate dew point
    elif sensor_status == 'Sensor error':
        time.sleep(2)  # 2 sec
        get_sensor_data(bmp)  # Try again
        if sensor_status == 'Sensor error':
            blink_led(1, 15)
            print('Exiting application:', sensor_status)
            sys.exit()  # Reset
        
    # Update unit ID
    get_id() # Last value in IP addr
    
    # Set up interrupt pin
    ldr_sensor.irq(trigger=Pin.IRQ_FALLING, handler=ldr_sensor_interrupt_handler) # Triggers interrupt when LDR sensor goes dark
    
    wdt.feed()  # Keep watch dog from triggering

    while True:
        # Tasks on cold start
        if coldstart == True:
            # Preset variables
            setup_variables()
            coldstart = False
            first_pass = False

        if not wlan_connected:
            print('Connecting to WiFi')
            if wlan_reconnect == True:
                notConnectedCounter += 1
            asyncio.create_task(connect_to_wifi())
               
        if wlan_connected and (server_connect_state == False):
            '''
                Start a TCP server on the given host and port. The callback will be called with incoming, accepted connections,
                and be passed 2 arguments: reader and writer streams for the connection. Returns a Server object.
            '''
            print('Creating webserver')
            asyncio.create_task(asyncio.start_server(serve_client, '0.0.0.0', 80))
            print('...Web server ready...\n')
            server_connect_state = True
        
        await asyncio.sleep(LOOP_REFRESH_SEC)  # 2 sec 
        
        # Update RTC  global storage variable
        local_time = rtc.datetime()

        # Check garbage collection timeout
        if gc_timeout_counter == 0: # Times out after 30 seconds
            gc.collect() #Free up memory
            gc_timeout_counter = GC_TIMEOUT #Preset counter
        
        #Check controller alive counter
        if ctrl_live_counter == 0: # Times out after 15 seconds
            blink_led(frequency=0.05, num_blinks=1) #Flash LED
            ctrl_live_counter = CTRL_LIVE_PERIOD #Preset counter

        # Refresh sensor data
        if sensor_status == 'Sensor active':
            get_sensor_data(bmp)
            dew_point_calc() # Calculate dew point
        elif sensor_status == 'Sensor error':
            time.sleep(2)  # 2sec
            get_sensor_data(bmp)  # Try again to confirm
            if sensor_status == 'Sensor error':
                blink_led(1, 15)
                print('Resetting ESP32...', sensor_status)
                await asyncio.sleep(2)
                machine.reset() # Reset ESP

        # Update LDR light state
        ldr_sensor_state = read_ldr() # Record LDR comparator output state
        
        # Check LDR int flag for a daylight transition
        if ldr_int_flag == HIGH: # Set if interrupt from LDR cct occured
            ldr_sensor_state = read_ldr() # Read LDR again
            if ldr_sensor_state == 'DARK':
                led_light_flag = LOW # Set flag low if dark
                #print('Dark Light flag: ', led_light_flag)
            elif ldr_sensor_state == 'LIGHT': 
                led_light_flag = HIGH # Set flag high if light
                #print('Light Light flag: ', led_light_flag)
            ldr_int_flag = LOW # Clear interrupt flag
        
        # Task if light auto mode set
        if auto_mode == True and ldr_sensor_state == 'DARK' and led_light_state == 'OFF' and led_light_flag == LOW:
            led_light_onPeriodCntr_secs = LED_LIGHT_ON_PERIOD # Preset counter
            setup_led_light(ON) # Turn on light
            led_light_flag = HIGH # Indicate transition from light to dark occurred
        elif auto_mode == True and ldr_sensor_state == 'LIGHT' and peripheral_test == 'DISABLED': # Turn off if it turns 'light' momentarily
            setup_led_light(OFF) # Turn off light
            led_light_flag = LOW # To enable the light to turn on when it goes dark again before the next true daily transition
        elif auto_mode == True and led_light_onPeriodCntr_secs <= 0: # Check if led light counter expired
            setup_led_light(OFF) # Turn off light
            led_light_flag = HIGH # After count expires the light is not allowed to turn back on until a true daylight transition clears the flag  
        elif auto_mode == True and led_light_onPeriodCntr_secs > 0:
            print('LED light on period count: ', led_light_onPeriodCntr_secs)
            
        # Task if light manual mode set
        if manual_mode == True and ldr_sensor_state == 'DARK' and led_light_state == 'OFF':
            setup_led_light(ON) # Turn on light
        elif manual_mode == True and ldr_sensor_state == 'LIGHT':
            setup_led_light(OFF) # Turn off light
            led_light_button_color = 'red' # Setup button for next action  
            led_light_button_action = 'On'
            auto_button_color = 'red'
            manual_mode = False
 
        # Check heater temperature window
        check_temp_window()

        # Check if TEST button pressed
        if peripheral_test == 'ENABLED':
            if first_pass == False:
                #print("Turning on peripherals for test")
                peripheral_onPeriodCntr_secs = PERIPHERAL_TEST_PERIOD  # Preset on period
                setup_heater(ON)  # Turn on heaters
                setup_led_light(ON) # Turn on LED light
                result = "{}:{}:{}".format(local_time[4], local_time[5], local_time[6])  # Get RTC time
                if result == '':
                    heater_swon_time = 'Time unavailable...'
                else:
                    heater_swon_time = result
                    heater_run_time = '...'
                first_pass = True
                
            elif first_pass == True and peripheral_onPeriodCntr_secs > 0:
                #print("Peripheral on period count: ", peripheral_onPeriodCntr_secs)
                heater_run_time = str(peripheral_onPeriodCntr_secs)
                
            elif first_pass == True and peripheral_onPeriodCntr_secs <= 0: 
                #print("Heater test complete, turning off peripherals")
                setup_heater(OFF)  # Turn off heaters
                setup_led_light(OFF) # Turn off LED Light 
                heater_run_time = '...'
                heater_swon_time = '...'
                await asyncio.sleep(1)
                peripheral_test = 'DISABLED'
                first_pass = False
                if auto_mode == True:
                    led_light_flag = LOW
                
        wdt.feed()  # Keep watch dog from triggering every second
        
        # Turn on heaters if temperature good...
        if heater_tempWindow_status == 'TEMP GOOD' and heater_state == 'OFF':
            setup_heater(ON) # Turn on heaters
            result = '{}:{}:{}'.format(local_time[4], local_time[5], local_time[6])  # Get RTC time
            if result == '':
                heater_swon_time = 'Time unavailable...'
            else:
                heater_swon_time = result # Record RTC time
            heater_run_time = '...'
        
        # Calculate web page heater on run time value
        if heater_tempWindow_status == 'TEMP GOOD' and heater_state == 'ON':
            if heater_day_count > 0:
                heater_run_time = str(heater_day_count)
                heater_time_unit = 'days'
            elif heater_hour_count > 0:
                heater_run_time = str(heater_hour_count)
                heater_time_unit = 'hours'
            elif heater_min_count > 0:
                heater_run_time = str(heater_min_count)
                heater_time_unit = 'mins'
            elif heater_sec_count > 0:
                heater_run_time = str(heater_sec_count)
                heater_time_unit = 'secs'
              
        # Check if temperature rises above threshold...
        if heater_tempWindow_status == 'TEMP TOO HIGH' and heater_state == 'ON' and peripheral_test == 'DISABLED':
            setup_heater(OFF)  # Turn off heaters
            result = '{}:{}:{}'.format(local_time[4], local_time[5], local_time[6])  # Get RTC time
            if result == '':
                heater_run_time = 'Time unavailable...'
            else:
                heater_run_time = result # Record RTC time
        
        #Print wlan flags
        #print('wlan_connected:', wlan_connected)
        #print('server_connect_state', server_connect_state)
        #print('Config:', wlan.ifconfig())
        
        #Get RSSI
        if wlan_connected:
            get_rssi()
        
        # Check wlan connection state
        wifi_state  = wlan_test()
        if wifi_state != 1010:
            wlan_disconnect_time = str(local_time[0]) + ':' + str(local_time[1])  + ':' +  str(local_time[2])  + '...' +  str(local_time[4])  + ':' +  str(local_time[5]) + ':' +  str(local_time[6])
            #print('Network connected at ', wlan_connect_time)
            #print('Network disconnected at ', wlan_disconnect_time)
            # Config for restart
            wlan_connected = False  # Re-connect flag cleared
            server_connect_state = False  # Server flag cleared
            wlan_reconnect = True # Indicate reconnection is required
            if notConnectedCounter > 10: # Check re-connect timeout counter
                print('WiFi error...resetting ESP32...')
                await asyncio.sleep(2)
                machine.reset() # Reset ESP
                
        wdt.feed()  # Keep watch dog from triggering every second
    # loop...


try:
    asyncio.run(main())
except KeyboardInterrupt:
    print('Exiting program on keyboard interrupt')
    sys.exit()
finally:
    asyncio.new_event_loop()  # Reset the event loop and return it.


