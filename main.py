"""
    Water Level Monitor with WiFi V1.4
    Uses one sw input for a mechanical level sensor and  one PWM output to drive a piezo buzzer. 
    The sensor is checked periodically and if an alarm exists the controller responds in alarm mode until the alarm goes away.
    When the keep alive timeout expires the WDT is reset. 
    Date: 2024-07-19
    
Note:
    Sump pump number needs to be adjusted for each new device.
    Sump '1' just off stairs
    Sump '2' opposite end of house

Updates:
    BME280 temp sensor included.
    No longer goes into sleep mode
    ESP resets after 10 attempts to connect to WiFi
    Client refresh time constant added. Set to 30 secs
    Add temperatue, pressure, humidity and dew point to client web page
    Add OTA programming
    Set up to use static address
    Add garbage collection
"""

# Modules
import time
import ntptime
import ubinascii
import network
import urequests as requests
from credentials import WIFI_NAME, WIFI_PASS, TWILIO_ACCOUNT_SID, TWILIO_AUTH_TOKEN, TWILIO_NUMBER, BASE_NUMBER
import uasyncio as asyncio
import machine
from machine import Pin, I2C, WDT, PWM, Timer, RTC
import sys
import utime
import esp, esp32
import bme280
import ota
import errno
import gc

# Constants
WLAN_TIMEOUT = 20 # Number of attempts to reconnect. Period = WLAN_TIMEOUT * LOOP_REFRESH_SEC
FIRMWARE_VERSION = '1.4'
INTERVAL_SEC = 0.25
LOOP_REFRESH_SEC = 2.0
WDT_TIMEOUT = 30000 # 30sec
ON = 1
OFF = 0
HIGH = 1
LOW = 0
CLOSED = 0
OPEN = 1
CONTACT_BOUNCE_DELAY = 10 # ms
BUZZER_ONTIME = 0.25 # mins
BUZZER_OFFTIME = 0.25 # mins
BUZZER_TIMER_RUN = 1
BUZZER_TIMER_STOP = 0
SUMP_PUMP_NUMBER = '1'
UTC_OFFSET = 4 * 60 * 60  # Seconds, Ottawa offset = 4/5
CLIENT_REFRESH_PERIOD = 30 # Seconds
CTRL_LIVE_PERIOD = 15 # 15 Seconds
GC_TIMEOUT = 1800 # 30mins x 60secs = 1800secs
STATIC_ADDR = '192.168.2.51'

# Sensor variables
amb_temp = ''
pressure = ''
humidity = ''
dew_point = ''
sensor_status = 'Unknown'
level_sensor_status = 'closed' #'open' or 'closed'
water_level_state = 'water_low' #'water_high' or 'water_low'
interrupt_pin = 0

# Garbage collection timeout variable
gc_timeout_counter = GC_TIMEOUT

# Buzzer variables
buzzer_offPeriod_secs = BUZZER_OFFTIME * 60
buzzer_onPeriod_secs = BUZZER_ONTIME * 60
buzzer_offPeriodCntr_secs = buzzer_offPeriod_secs
buzzer_onPeriodCntr_secs = buzzer_onPeriod_secs
buzzer_clientOnPeriod_mins = BUZZER_ONTIME # Default = 5 mins
buzzer_clientOffPeriod_mins = BUZZER_OFFTIME # Default = 5 mins
buzzer_state = 'OFF' #'ON' or 'OFF'
buzzer_enabled = 'ENABLED' # Default
buzzer_muted = False # Default
buzzer_timer_status = ''

# Alarm variables
alarm_activated_time = '...'
alarm_deactivated_time = '...'
alarm_state = 'inactive' # 'active' or 'inactive'

# Controller variables
unit_id = ''
coldstart = True # On power up flag is true

# WLAN variables
ip_addr = ''
wlan_connected = False
wlan_connect_time = '' # Time and date connected to network
server_connect_state = False # False indicates server disconnected
wlan_reconnect = ''
notConnectedCounter = 0
wlan_disconnect_time = ''
rssi = '' # Recieved signal strength

# Timer variables
timer_tick = False
first_pass = False
local_time = ''
ctrl_live_counter = 30 #Seconds

# SMS variables
recipient = BASE_NUMBER
sender = TWILIO_NUMBER
auth_token = TWILIO_AUTH_TOKEN
account_sid = TWILIO_ACCOUNT_SID
sms_message = ''
sms_state = '...'

# Web-page variables
mute_button_color = 'green' #Default 
mute_button_txt = 'MUTE'
enable_button_color = 'green' #Default
enable_button_txt = 'DISABLE'

# I2C device addresses
bmp_addr1 = 0x76  # BMP280 address 1
bmp_addr2 = 0x77  # BMP280 alternative address 2
num_i2c_devices = 0  # Number of I2C devices detected

# Create I2C1 object
i2c1 = I2C(1, scl=Pin(22), sda=Pin(21))
i2c_devices = bytearray()  # Devices storage array

# Create LED object
status_led = Pin(33, Pin.OUT, value=1) #LED off at power up
status_led_state = 'OFF' # LED state "ON" or "OFF"

# Create PWM object to drive buzzer
pwm0 = PWM(Pin(32), freq=1200, duty=512) #50%
pwm0.deinit() #Buzzer off

# Create water level sensor object
level_sensor = Pin(14, Pin.IN, Pin.PULL_UP) # GPIO14, internal pull up

# Create WLAN object
wlan = network.WLAN(network.STA_IF)

# Create WDT object
wdt = WDT(timeout=WDT_TIMEOUT) # 30secs timeout

# Create periodic timer object
tim0 = Timer(0)

# Create RTC object
rtc = RTC()

# Twilio class definition
class TwilioSMS:
    base_url = 'https://api.twilio.com/2010-04-01'
    
    def __init__(self, account_sid, auth_token):
        self.twilio_account_sid = account_sid
        self.twilio_auth = ubinascii.b2a_base64('{sid}:{token}'.format(
            sid=account_sid, token=auth_token)).strip()
        
    def create(self, body, from_, to):
        data = 'Body={body}&From={from_}&To={to}'.format(
            body=body, from_=from_.replace('+', '%2B'),
            to=to.replace('+','%2B'))
        
        r = requests.post(
            '{base_url}/Accounts/{sid}/Messages.json'.format(
            base_url=self.base_url, sid=self.twilio_account_sid),
            data = data,
            headers = {'Authorization': b'Basic ' + self.twilio_auth,
                       'Content-Type': 'application/x-www-form-urlencoded'})
        print('SMS sent with status code', r.status_code)
        print('Response: ', r.text)

# Create Twilio SMS class object
sms = TwilioSMS(account_sid, auth_token)

# Cold start variable setup
def setup_variables():
    global ip_addr
    global wlan_connect_time
    global server_connect_state
    global mute_button_color 
    global mute_button_txt
    global enable_button_color
    global enable_button_txt

    #WLAN variables
    ip_addr = '0,0,0,0'
    wlan_connect_time = '...'
    server_connect_state = False

    #Web page variables
    mute_button_color = 'green' #Default 
    mute_button_txt = 'MUTE'
    enable_button_color = 'green' #Default
    enable_button_txt = 'DISABLE'
    

#Tim 0 callback function
def tim0_callback(tim0):
    global ctrl_live_counter
    global gc_timeout_counter
    global buzzer_onPeriodCntr_secs
    global buzzer_offPeriodCntr_secs

    # Decrement while counters > zero
    if ctrl_live_counter > 0: # Keep alive counter
        ctrl_live_counter -= 1

    if gc_timeout_counter > 0: # Garbage collection timeout
        gc_timeout_counter -= 1
        
    if buzzer_timer_status == 'BUZZER_TIMER_RUN' and (buzzer_state == 'ON') and (buzzer_muted == False) and (buzzer_enabled == 'ENABLED'):
        if buzzer_onPeriodCntr_secs > 0:
            buzzer_onPeriodCntr_secs -= 1 #Decrement on period cntr 
    
    if buzzer_timer_status == 'BUZZER_TIMER_RUN' and buzzer_state == 'OFF' and buzzer_muted == False and buzzer_enabled == 'ENABLED':
        if buzzer_offPeriodCntr_secs > 0:
            buzzer_offPeriodCntr_secs -= 1 #Decrement off period cntr

# Get recieved signal strength
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

# Get unit ID
def  get_id():
    global unit_id
    
    id = STATIC_ADDR
    id = id.split('.')
    unit_id = id[3] # Last value in address is used for ID
    
#Create server webpage
def webpage(
            amb_temp, pressure, humidity, dew_point, sms_state,
            alarm_state, alarm_activated_time, alarm_deactivated_time, local_time, buzzer_muted,
            buzzer_state, buzzer_enabled, buzzer_clientOnPeriod_mins, buzzer_clientOffPeriod_mins,
            mute_button_color, mute_button_txt,
            enable_button_color, enable_button_txt,
            water_level_state, FIRMWARE_VERSION, unit_id, rssi, SUMP_PUMP_NUMBER):
    
    # HTML Template
    html = f"""
            <!DOCTYPE html>
            <html lang="en">

            <head>
                <meta charset="UTF-8">
                <meta name="viewport" content="width=device-width, initial-scale=1">
                <link rel="icon" href="data:,">
                <title>Water Level Sensor</title>
                <meta http-equiv="refresh" content={CLIENT_REFRESH_PERIOD}>
                <!--meta http-equiv="refresh" content="15; URL=192.168.2.xx"/-->
            </head>
            <body>
            
            <p><center><h2>Sump {SUMP_PUMP_NUMBER} Water Level Monitor {FIRMWARE_VERSION}</center></h2></p>

            <p><center>Local Date: <em>{local_time[0]}:{local_time[1]}:{local_time[2]}</em> &nbsp Local Time: <em>{local_time[4]}:{local_time[5]}:{local_time[6]}</em></center></p> 
            <p><center>Unit ID: <em>{unit_id}</em> &nbsp Signal Strength: <em>{rssi}dBm</em></center></p> 

            <p><center><h3>Sensor Data</center></h3></p>            
            <p><center>Temperature:<em> {amb_temp}DegC</em> &nbsp Pressure:<em> {pressure}</em></center></p>
            <p><center>Humidity:<em> {humidity}&percnt;</em> &nbsp Dew point:<em> {dew_point}DegC</em></center></p>   
             
            <center><h3>Alarm Status</h3></center>
            <p><center>Alarm state: <em>{alarm_state}</em></center></p> 
            <p><center>Alarm activated: <em>{alarm_activated_time}</em> &nbsp Alarm deactivated: <em>{alarm_deactivated_time}</em></center></p>
            <p><center>SMS message: <em>{sms_state}</em></center></p>
                          
            <p><center><h3>Water Level Status</center></h3></p>
            <p><center>Water level: <em>{water_level_state}</em></center></p>
            
            <center><h3>Buzzer Control</h3></center>
            <p><center>Buzzer state: <em>{buzzer_state}</em></center></p>
            <p><center>Buzzer enabled: <em>{buzzer_enabled}</em> &nbsp Buzzer muted: <em>{buzzer_muted}</em></center></p>
            
            <form id="0">            
            <center> <button name="BUZZER" value="CONTROL" type="submit" style="
            background-color:{enable_button_color};
            border:4px solid #000000;
            border-radius: 15px;
            color:white;
            padding:10px 10px;
            text-align:center;
            text-decoration:none;
            display:inline-block;
            font-weight:bold;
            font-size:1.4em;
            cursor:pointer">{enable_button_txt}</button></center>
            </form>
            
            <br>
            
            <form id="1">
            <center><label for="ON PERIOD">Buzzer on time:</label>
                <select name="ON PERIOD" id="ON PERIOD">
                    <option value="">--Select--</option>
                    <option value="0.13">5 secs</option>
                    <option value="0.25">15 secs</option>
                    <option value="0.5">30 secs</option>
                    <option value="1">1 mins</option>
                    <option value="5">5 mins</option>
                    <option value="15">15 mins</option>
                    <option value="30">30 mins</option>
                    <option value="60">60 mins</option>
                </select> &nbsp; <button type="submit" form="1" value="Submit">Submit</button></center>
            </form>
            <center><p>Buzzer on time selected: {buzzer_clientOnPeriod_mins} mins</p></center>
            
            <form id="2">
            <center><label for="OFF PERIOD">Buzzer off time:</label>
                <select name="OFF PERIOD" id="OFF PERIOD">
                    <option value="">--Select--</option>
                    <option value="0.13">5 secs</option>
                    <option value="0.25">15 secs</option>
                    <option value="0.5">30 secs</option>
                    <option value="1">1 mins</option>
                    <option value="5">5 mins</option>
                    <option value="15">15 mins</option>
                    <option value="30">30 mins</option>
                    <option value="60">60 mins</option>
                </select> &nbsp; <button type="submit" form="2" value="Submit">Submit</button></center>       
            </form>
            <center><p>Buzzer off time selected: {buzzer_clientOffPeriod_mins} mins</p></center>
            
            <form id="3">            
            <center> <button name="MUTE" value="MUTE" type="submit" style="
            background-color:{mute_button_color};
            border:4px solid #000000;
            border-radius: 15px;
            color:white;
            padding:10px 10px;
            text-align:center;
            text-decoration:none;
            display:inline-block;
            font-weight:bold;
            font-size:1.4em;
            cursor:pointer">{mute_button_txt}</button></center>
            </form>
            
            <br>
            
            <form id="4">            
            <center> <button name="REFRESH" value="REFRESH" type="submit" style="
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
            cursor:pointer">REFRESH</button></center>
            </form>
            
            <center><p>Last command issued: %s</p></center>
            
            </body>
            </html>
            """
    return str(html)

# Search for any devices on bus
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


# Create I2C object
def create_i2c_obj(i2c_devices):
    if bmp_addr1 in i2c_devices:
        # print("Creating BME instance for {}...\n".format(hex(bmp_addr1)))
        bmp = bme280.BME280(i2c=i2c1, address=bmp_addr1)  # Create BME280 object
    elif bmp_addr2 in i2c_devices:
        # print("Creating BME instance for {}...\n".format(hex(bmp_addr2)))
        bmp = bme280.BME280(i2c=i2c1, address=bmp_addr2)  # Create BME280 alternate object
    return bmp #bmp object


# Confirm I2C adresses
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

# Scan I2C bus
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

# Flash LED at rate defined
def blink_led(frequency = 0.5, num_blinks = 3):
    global status_led_state

    for _ in range(num_blinks):
        status_led.off() #Inverted, LED on
        status_led_state = "ON"
        time.sleep(frequency)
        status_led.on() #Inverted, LED off
        status_led_state = "OFF"
        time.sleep(frequency)

# Configure RTC
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
        
# Check water level        
def get_water_level():
    level = level_sensor.value() # Read level
    if level == OPEN: # Contacts closed
        time.sleep_ms(CONTACT_BOUNCE_DELAY) # 10ms delay 
        level = level_sensor.value() # Check again after delay to confirm valid activation
        if level == OPEN:
            return 'water_high'
    else:
        return 'water_low'
            
# Turn buzzer element OFF
def buzzer_off():
    global buzzer_state
    global buzzer_timer_status
    
    pwm0.deinit() # PWM0 off, GPIO32
    buzzer_state = 'OFF'
    buzzer_timer_status = 'BUZZER_TIMER_STOP'

# Turn buzzer element on
def buzzer_on():
    global buzzer_state
    global buzzer_timer_status
    global buzzer_offPeriodCntr_secs
    global buzzer_onPeriodCntr_secs
    global buzzer_offPeriod_secs
    global buzzer_onPeriod_secs
    
    buzzer_offPeriodCntr_secs = buzzer_offPeriod_secs # Counter preset with latest value
    buzzer_onPeriodCntr_secs = buzzer_onPeriod_secs # Counter preset with latest value
    pwm0.init(1200, duty=512) #PWM0 ON,GPIO32
    buzzer_state = 'ON'
    buzzer_timer_status = 'BUZZER_TIMER_RUN'

# Get sensor data
def get_sensor_data(bmp):
    global amb_temp
    global pressure
    global humidity
    global sensor_status

    try:
        # Get sensor temperature data (value only)
        t = bmp.values[0]
        t = t.split('C')  # Remove 'C'
        t = t[0]  # Get only the number
        amb_temp = t
        
        # Get sensor pressure data
        pressure = bmp.values[1]

        # Get sensor humidity data (value only)
        h = bmp.values[2]
        h = h.split('%')  # Remove '%'
        h = h[0]  # Get only the number
        humidity = h
        
        sensor_status = 'Sensor active'
    except:
        print('Temp sensor error...')
        sensor_status = 'Sensor error'

# Get dew point
def dew_point_calc():
    global amb_temp
    global humidity
    global dew_point
    temp = 0.0
    
    temp = float(amb_temp) - ((100-float(humidity))/5) # Dew point calculation
    temp = round(temp,2) # Round up number
    dew_point = str(temp)
    
def send_sms():
    global sms_message
    global sender
    global recipient
    
    print('Attempting to send sms')
    sms.create('Hello \r\n' + sms_message, sender, recipient)
    
#Connect to WiFi network
async def connect_to_wifi():
    global WLAN_TIMEOUT
    global wlan_connected
    global wlan_connect_time
    global ip_addr
    global local_time

    wdt.feed()  # Keep watch dog from triggering
    
    wlan.active(True) # Activate interface
    wlan.ifconfig( (STATIC_ADDR, '255.255.255.0', '192.168.2.1', '192.168.2.1')) # Set static address, subnet, gateway & dns
    wlan.connect(WIFI_NAME, WIFI_PASS)

    # Wait for connect or fail
    max_wait = WLAN_TIMEOUT # 20 secs
    while max_wait > 0 and wlan.status() != 1010: # 1010 for ESP32
        max_wait -= 1
        print('Waiting for connection...{}'.format(max_wait))
        time.sleep(LOOP_REFRESH_SEC)

    # Handle connection error
    if wlan.status() != 1010:
        blink_led(0.5, 1)
        wlan_connected = False
        await asyncio.sleep(2) #2sec    
    else:
        # Connection successful
        blink_led(0.5, 2)

        # Update RTC
        t = setup_RTC()
        if t != False :
            # print("Retrieving  sync'ed data from RTC")
            local_time = rtc.datetime()
        else:
            print('Using local time')
            local_time = time.localtime()
            
        print('...Date: {}:{}:{}'.format(local_time[0], local_time[1], local_time[2]))
        print('...WiFi Connected at {}:{}:{}'.format(local_time[4], local_time[5], local_time[6]))
        
        # Record time connected
        wlan_connect_time = str(local_time[0]) + ':' + str(local_time[1])  + ':' +  str(local_time[2])  + '...' +  str(local_time[4])  + ':' +  str(local_time[5]) + ':' +  str(local_time[6])
        status = wlan.ifconfig()
        ip_addr = status[0]
        print('...WLAN parameters: {}\n'.format(wlan.ifconfig()) )
        wlan_connected = True

# Web client handler
async def serve_client(reader, writer):  
    global alarm_state
    global alarm_activated_time
    global alarm_deactivated_time
    global buzzer_state
    global buzzer_enabled
    global buzzer_clientOnPeriod_mins
    global buzzer_clientOffPeriod_mins
    global buzzer_onPeriodCntr_secs
    global buzzer_offPeriodCntr_secs
    global buzzer_onPeriod_secs
    global buzzer_offPeriod_secs
    global enable_button_color
    global enable_button_txt
    global water_level_state
    global buzzer_muted
    global mute_button_txt
    global mute_button_color
    global unit_id
    global sms_state
    
    wdt.feed() # Keep the watch dog from triggering

    print('Client connected')
    blink_led(0.5, 1)
    
    req_timeout = 20
    try:        
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
    
    # Find valid heater commands within the request
    request = str(request_line)
    print('Request:', request_line)
    
    cmd_control = request.find('BUZZER=CONTROL')
    cmd_onPeriod = request.find('ON+PERIOD')
    cmd_offPeriod = request.find('OFF+PERIOD')
    cmd_mute = request.find('MUTE=MUTE')
    cmd_refresh = request.find('REFRESH=REFRESH')
    
    # show where the commands were found (-1 means not found)
    #print ('BUZZER=CONTROL => ' + str(cmd_control))
    #print('ON+PERIOD => ' + str(cmd_onPeriod))
    #print('OFF+PERIOD => ' + str(cmd_offPeriod))
    #print('MUTE=MUTE => ' + str(cmd_mute))
    #print('REFRESH=REFRESH => ' + str(cmd_refresh))
    
    stateis = '' # Keeps track of the last command issued
    
    # Carry out a command if it is found (found at index: 8)
    if cmd_control == 8:
        if enable_button_txt == 'DISABLE':
            stateis = "Buzzer disabled"
            # print(stateis)
            enable_button_color = 'red'
            enable_button_txt = 'ENABLE'
            buzzer_enabled = 'DISABLED'
        elif enable_button_txt == 'ENABLE':
            stateis = 'Buzzer enabled'
            # print(stateis)
            enable_button_color = 'green'
            enable_button_txt = 'DISABLE'
            buzzer_enabled = 'ENABLED'
        blink_led(0.1, 2)

    if cmd_onPeriod == 8 and buzzer_enabled == 'DISABLED':
        t = request.split() # Extract time value
        t = t[1]
        t = t.split('=')
        buzzer_clientOnPeriod_mins = t[1] # Mins
        print('On period in mins: ',buzzer_clientOnPeriod_mins)
        try:
            buzzer_onPeriod_secs = int(60* float(buzzer_clientOnPeriod_mins)) # Convert hrs to seconds & update main register
            print('Buzzer on period in seconds: ', buzzer_onPeriod_secs)
            buzzer_onPeriodCntr_secs = buzzer_onPeriod_secs # Update ON period timer counter
            stateis = 'On period changed to {}mins'.format(buzzer_clientOnPeriod_mins)
        except:
            print('Data input error')
    
    elif cmd_onPeriod == 8 and buzzer_enabled == 'ENABLED':
        stateis = "Not allowed to modify on time when buzzer enabled"
    
    if cmd_offPeriod == 8 and buzzer_enabled == 'DISABLED':
        t = request.split() # Extract time value
        t = t[1]
        t = t.split('=')
        buzzer_clientOffPeriod_mins = t[1] # Mins
        print('Off period in mins:', buzzer_clientOffPeriod_mins) 
        try:
            buzzer_offPeriod_secs = int(60* float(buzzer_clientOffPeriod_mins)) # Convert hrs to seconds & update main register
            print('Buzzer off period in seconds: ',buzzer_offPeriod_secs)
            buzzer_offPeriodCntr_secs = buzzer_offPeriod_secs # Update off period timer counter
            stateis = 'Off period changed to {}mins'.format(float(buzzer_clientOffPeriod_mins))
        except:
            print('Data input error')
            stateis = 'Not allowed to modify buzzer off time when enabled'

    elif cmd_offPeriod == 8 and buzzer_enabled == 'ENABLED':
        stateis = 'Not allowed to modify off time when buzzer enabled'

    if cmd_mute == 8:
        if mute_button_txt == 'MUTE':
            buzzer_muted = True  
            stateis = 'Buzzer muted'
            # print(stateis)
            mute_button_txt = 'UNMUTE'
            mute_button_color = 'red'
        elif mute_button_txt == 'UNMUTE':
            buzzer_muted = False
            stateis = 'Buzzer unmuted'
            # print(stateis)
            mute_button_txt = 'MUTE'
            mute_button_color = 'green'
        blink_led(0.1, 2)

    if cmd_refresh == 8:
        stateis = 'Page refresh'
        # print(stateis)
        blink_led(0.1, 2)

    # Free memory
    gc.collect() # Run garbage collection
    free_mem = gc.mem_free()
    print('Memory freed for response: ', free_mem)
    if free_mem < 10000:
        print('Not enough memory for response message')
        return
    
    try:
        response = webpage(
            amb_temp, pressure, humidity, dew_point, sms_state,
            alarm_state, alarm_activated_time, alarm_deactivated_time, local_time, buzzer_muted,
            buzzer_state, buzzer_enabled, buzzer_clientOnPeriod_mins, buzzer_clientOffPeriod_mins,
            mute_button_color, mute_button_txt,
            enable_button_color, enable_button_txt,
            water_level_state, FIRMWARE_VERSION, unit_id, rssi, SUMP_PUMP_NUMBER) % stateis
        
        writer.write('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n') # Header
        writer.write(response) # Send html web page
        await writer.drain()
        await writer.wait_closed()
        print('Client disconnected')
    except OSError as exc:
        # Free memory
        if exc.errno == errno.ENOMEM:
            gc.collect()
            print('Free memory: ', gc.mem_free())
     
def level_sensor_interrupt_handler(pin):
    global water_level_state
    global interrupt_pin
    
    # Get water level state
    water_level_state = get_water_level() # Returns 'water_high' or 'water_low'
    interrupt_pin = pin   

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
        print('WLAN connected, status = ', 1010)
    return wlan.status() 

#Main loop
async def main():
    global LOOP_REFRESH_SEC
    global server_connect_state
    global wlan_connected
    global buzzer_onPeriodCntr_secs
    global buzzer_offPeriodCntr_secs 
    global buzzer_offPeriod_secs
    global buzzer_onPeriod_secs
    global buzzer_clientOnPeriod_mins
    global buzzer_clientOffPeriod_mins
    global buzzer_timer_status
    global buzzer_state
    global buzzer_swoff_time
    global buzzer_swon_time
    global sms_message
    global buzzer_muted
    global alarm_state
    global coldstart
    global water_level_state
    global alarm_activated_time
    global alarm_deactivated_time
    global mute_button_txt
    global enable_button_txt
    global buzzer_button_txt
    global sensor_status
    global first_pass
    global local_time
    global ctrl_live_counter
    global status_led_state
    global wlan_reconnect
    global notConnectedCounter
    global wlan_disconnect_time
    global wlan_connect_time
    global gc_timeout_counter
    global sms_state

    # Start timer 0
    tim0.init(period=1000, mode=Timer.PERIODIC, callback=tim0_callback)

    # Restart visual indication
    blink_led(0.25, 2)

    # Look for I2C devices
    bmp = test_i2c()
    if bmp == False:
        sensor_status = 'Sensor error'
        blink_led(1, 10)
        print("Exiting application:", sensor_status)
        sys.exit() # Reset
    else:
        sensor_status = 'Sensor active'
    
     # Preset states
    status_led_state = 'OFF'
    coldstart = True
    
    # Set up outputs
    buzzer_off()
    status_led.off()
    pwm0.deinit() # Turn off GPIO32 pwm
    
    # Preset flags
    buzzer_state = 'OFF'
    buzzer_muted = False
    water_level_state = 'water_low'
    buzzer_timer_status = 'BUZZER_TIMER_STOP'
    
    # Preset web template
    mute_button_txt = 'MUTE'
    enable_button_txt = 'DISABLE'

    # Get data
    if sensor_status == 'Sensor active':
        get_sensor_data(bmp)
        dew_point_calc()    # Calculate dew point
    elif sensor_status == 'Sensor error':
        time.sleep(2)  # 2sec
        get_sensor_data(bmp)  # Try again
        if sensor_status == 'Sensor error':
            blink_led(1, 15)
            print("Exiting application:", sensor_status)
            sys.exit()  # Reset

    # Update unit ID
    get_id() # Last value in IP addr

    # Set up interrupt pin
    level_sensor.irq(trigger=Pin.IRQ_RISING, handler=level_sensor_interrupt_handler) # Triggers interrupt when level sensor sw opens

    wdt.feed()  # Keep watch dog from triggering
    
    #Main loop
    while True:
        # Tasks on cold start
        if coldstart == True:
            # Preset variables
            setup_variables()
            coldstart = False

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
            asyncio.create_task(asyncio.start_server(serve_client, "0.0.0.0", 80))
            print('...Web server ready...\n')
            server_connect_state = True
        
        await asyncio.sleep(LOOP_REFRESH_SEC)  # 2 sec

        # Update RTC  global storage variable
        local_time = rtc.datetime()

        # Do garbage collection every 30 secs
        if gc_timeout_counter == 0:
            gc.collect() #Free up memory
            print("Memory free: ", gc.mem_free())
            gc_timeout_counter = GC_TIMEOUT # Preset counter
        
        # Visual controller alive indication
        if ctrl_live_counter == 0:
            blink_led(frequency=0.05, num_blinks=1) # Flash LED
            ctrl_live_counter = CTRL_LIVE_PERIOD # Preset counter

        # Refresh data
        if sensor_status == 'Sensor active':
            get_sensor_data(bmp)
            dew_point_calc()
        elif sensor_status == 'Sensor error':
            time.sleep(2)  # 2sec
            get_sensor_data(bmp)  # Try again
            if sensor_status == 'Sensor error':
                blink_led(1, 15)
                print("Exiting application:", sensor_status)
                sys.exit()  # Reset

        # Process water level result, 'water_high' or 'water_low'
        if water_level_state == 'water_high' and alarm_state == 'inactive' and buzzer_muted == False and buzzer_enabled =='ENABLED': # Water level goes 'high'
            buzzer_on() # Buzzer turned on, start timer
            alarm_state = 'active' # Activate alarm
            print("Sump pump water level high")
                   
            # Get time
            result = str(local_time[4])  + ':' +  str(local_time[5]) + ':' +  str(local_time[6])
            # print('Result ', result)
            if result == '...':            
                alarm_activated_time = 'Time unavailable...'
            else:
                alarm_activated_time = result
                print("Alarm activated at ", alarm_activated_time)
            alarm_deactivated_time = '...'
                    
            # Send SMS
            if wlan_connected == True:
                sms_message = "Alarm at sump pump " + SUMP_PUMP_NUMBER + " at " + alarm_activated_time + " \n\r Water level high, alarm activated!"
                print("Sending alarm activated sms")
                send_sms()
                sms_state = 'Alarm activated message sent'
            else:
                print("WLAN not connected, unable to send SMS")
 
        if water_level_state == 'water_high' and alarm_state == 'active' and buzzer_muted == False and buzzer_enabled =='DISABLED':
            pwm0.deinit() # PWM off
            buzzer_state = 'OFF'

        if water_level_state == 'water_low' and alarm_state == 'active': # Water level goes 'low'
            buzzer_off() # Buzzer turned off
            buzzer_state = 'OFF'
            alarm_state = 'inactive'
            buzzer_button_txt = 'MUTE'
            print("Sump pump water level normal")
                    
            # Get time
            result = str(local_time[4])  + ':' +  str(local_time[5]) + ':' +  str(local_time[6])
            if result == '...':
                alarm_deactivated_time = 'Time unavailable...'
            else:
                alarm_deactivated_time = result
                print("Alarm deactivated at ", alarm_deactivated_time)

            # Send SMS
            if wlan_connected == True:
                sms_message = "Alarm deactivated at sump pump " + SUMP_PUMP_NUMBER + " at " + alarm_deactivated_time + "\n\r Water level normal!"
                print("Sending alarm deactivated sms")
                send_sms()
                sms_state = 'Alarm deactivated message sent'
            else:
                print("WLAN not connected, unable to send SMS")
                
        # Tasks when buzzer on and buzzer timer active
        if buzzer_timer_status == 'BUZZER_TIMER_RUN' and buzzer_state == 'ON' and buzzer_muted == False and buzzer_enabled == 'ENABLED':
            if buzzer_onPeriodCntr_secs == 0:
                pwm0.deinit() # Buzzer turned off
                buzzer_state = 'OFF'

                # Get time
                result = str(local_time[4])  + ':' +  str(local_time[5]) + ':' +  str(local_time[6])
                if result == '...':
                    buzzer_swoff_time = 'Time unavailable...'
                else:
                    buzzer_swoff_time = result
                # print("Buzzer_turned off: {}".format(result))
                buzzer_offPeriodCntr_secs = buzzer_offPeriod_secs #Cntr preset
                                
        if buzzer_timer_status == 'BUZZER_TIMER_RUN' and buzzer_state == 'OFF' and buzzer_muted == False and buzzer_enabled == 'ENABLED':
            if buzzer_offPeriodCntr_secs == 0:
                pwm0.init(1200, duty=512) # Buzzer turned on
                buzzer_state = 'ON'

                # Get time
                result = str(local_time[4])  + ':' +  str(local_time[5]) + ':' +  str(local_time[6])
                if result == '...':
                    buzzer_swon_time = 'Time unavailable...'
                else:
                    buzzer_swon_time = result
                # print("Buzzer_re-started: {}".format(result)) 
                buzzer_onPeriodCntr_secs = buzzer_onPeriod_secs # Cntr preset

        if buzzer_timer_status == 'BUZZER_TIMER_RUN' and buzzer_state == 'ON' and buzzer_muted == True and buzzer_enabled == 'ENABLED':
            pwm0.deinit() # PWM off
            buzzer_state = 'OFF'
                    
        if buzzer_timer_status == 'BUZZER_TIMER_STOP' and (buzzer_state == 'OFF' or buzzer_state == 'ON'): # Default
            buzzer_off()
            water_level_state = 'water_low'
            alarm_state = 'inactive'

        # Print wlan flags
        print('wlan_connected:', wlan_connected)
        print('server_connect_state', server_connect_state)
        print('Config:', wlan.ifconfig())
        
        # Get RSSI
        get_rssi()

        wifi_state  = wlan_test()    
        if wifi_state != 1010:
            wlan_disconnect_time = str(local_time[0]) + ':' + str(local_time[1])  + ':' +  str(local_time[2])  + '...' +  str(local_time[4])  + ':' +  str(local_time[5]) + ':' +  str(local_time[6])
            print('Network connected at ', wlan_connect_time)
            print('Network disconnected at ', wlan_disconnect_time)
            # Config for restart
            wlan_connected = False  # Re-connect flag cleared
            server_connect_state = False  # Server flag cleared
            wlan_reconnect = True # Indicate re connection is required
            if notConnectedCounter > 10: # Check re-connect timeout counter
                print('Resetting ESP32...')
                await asyncio.sleep(2)
                machine.reset() # Reset ESP
            
        wdt.feed() # Keep watch dog from triggering

    # Main loop...
    
try:
    asyncio.run(main())
except KeyboardInterrupt:
    print('Exiting program on keyboard interrupt')
    sys.exit()
finally:
    asyncio.new_event_loop()   # Reset the event loop and return it.
