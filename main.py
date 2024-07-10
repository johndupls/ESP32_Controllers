"""
    Water Level Monitor with WiFi
    Version: V1.4 
    Date: 2024-07-10
    Static IP Address: 192.168.2.51

    Desc:
    Uses one sw input and  one PWM output to drive a buzzer. Spends all its time in light sleep.
    Comes out of sleep when the level sensor is activated or the keep alive timeout expires.
    The sensor is checked and if an alarm does exist the controller stays awake until the alarm goes away.
    When the keep alive timeout expires the WDT is reset. BMP temp sensor excluded.
    
    Note:
    Sump pump IP address needs to be adjusted for new devices.
    Sump '1' just off stairs ID: 52
    Sump '2' opposite end of house ID: 51

    Updates:
    BMP device replaced with BME280
    Email functionality replaces SMS functionality
    Unit no longer remains in sleep
    Unit is assigned a static IP address

"""

#Modules
import time
import ubinascii
import network
import urequests as requests
from credentials import WIFI_NAME, WIFI_PASS, USER_NAME, MAIL_PASS , PORT_NUM , HOST_NAME, SENDER, RECIPIENT
import uasyncio as asyncio
from machine import Pin,I2C,WDT,PWM,deepsleep, lightsleep, wake_reason
import sys
import utime
import esp, esp32
import umail

#Constants
WLAN_TIMEOUT = 20 #Number of attempts to reconnect. Period = WLAN_TIMEOUT * LOOP_REFRESH_SEC
FIRMWARE_VERSION = '1.4'
INTERVAL_SEC = 0.25
LOOP_REFRESH_SEC = 1.0
WDT_TIMEOUT = 50000 #50sec
ON = 1
OFF = 0
HIGH = 1
LOW = 0
CLOSED = 0
OPEN = 1
CONTACT_BOUNCE_DELAY = 10 #ms
BUZZER_ONTIME = 0.25 #mins
BUZZER_OFFTIME = 0.25 #mins
BUZZER_TIMER_RUN = 1
BUZZER_TIMER_STOP = 0
SUMP_PUMP_NUMBER = '2'
STATIC_ADDR = "192.168.2.51"

#Water level sensor variables
level_sensor_status = 'closed' #'open' or 'closed'
water_level_state = 'water_low' #'water_high' or 'water_low'
interrupt_pin = 0

#Global controller variables
unit_id = ""

#Buzzer variables
buzzer_offPeriod_secs = BUZZER_OFFTIME * 60
buzzer_onPeriod_secs = BUZZER_ONTIME * 60
buzzer_offPeriodCntr_secs = buzzer_offPeriod_secs
buzzer_onPeriodCntr_secs = buzzer_onPeriod_secs
buzzer_clientOnPeriod_mins = BUZZER_ONTIME #Default = 5 mins
buzzer_clientOffPeriod_mins = BUZZER_OFFTIME #Default = 5 mins
alarm_activated_time = ''
alarm_deactivated_time = ''
buzzer_state = 'OFF' #'ON' or 'OFF'
buzzer_enabled = 'ENABLED' #Default
buzzer_muted = True
buzzer_timer_status = ''

#Scratch variables
email_message = ''
alarm_state = 'inactive' #'active' or 'inactive'

#WLAN variables
ip_addr = ''
wlan_connected = False
wlan_connect_time = '' #Time and date cONnected to network
server_connect_state = False #False indicates server discONnected

#Configure WiFi credentials
ssid = WIFI_NAME
password = WIFI_PASS

#Email variables
host_name = HOST_NAME
port_num = PORT_NUM
user_name = USER_NAME
mail_pass = MAIL_PASS
sender = SENDER
recipient = RECIPIENT

#Global web page variables
mute_button_color = 'green' #Default
mute_button_txt = ''
enable_button_color = 'green' #Default
enable_button_txt = ''

#Global startup variable
coldstart = True #On power up flag is true

#Create LED object
status_led = Pin(33,Pin.OUT,value=1) #LED off at power up
status_led_state = 'OFF' #LED state "ON" or "OFF"

#Create PWM object to drive buzzer
pwm0 = PWM(Pin(32), freq=1200, duty=512) #50%
pwm0.deinit() #Buzzer off

#Create water level sensor object
level_sensor = Pin(14, Pin.IN, Pin.PULL_UP) #GPIO14, internal pull up

#Create WLAN object
wlan = network.WLAN(network.STA_IF)

#Create WDT object
wdt = WDT(timeout=WDT_TIMEOUT) #30secs timeout

#Create email  object
smtp = umail.SMTP(host_name, port_num, username=user_name, password=mail_pass)

def get_id():
    global unit_id
    
    id = STATIC_ADDR
    id = id.split(".")
    unit_id = id[3]
    
#Create server webpage
def webpage(
            alarm_state, alarm_activated_time, alarm_deactivated_time,
            buzzer_state, buzzer_enabled, buzzer_clientOnPeriod_mins, buzzer_clientOffPeriod_mins,
            mute_button_color, mute_button_txt,
            enable_button_color, enable_button_txt,
            water_level_state, FIRMWARE_VERSION, unit_id, SUMP_PUMP_NUMBER):
    
    # HTML Template
    html = f"""
            <!DOCTYPE html>
            <html lang="en">

            <head>
                <meta charset="UTF-8">  
                <meta name="viewport" content="width=device-width, initial-scale=1">
                <link rel="icon" href="data:,">
                <title>Water Level Monitor</title>
                <!--meta http-equiv="refresh" content="10"/-->
            </head>

            <body>
            <p><center><h2>Sump {SUMP_PUMP_NUMBER} Water Level Monitor {FIRMWARE_VERSION}</center></h2></p>
            <p><center>Unit ID: <em>{unit_id}</em></center></p>          
             <br>
             
            <center><h3>Alarm Status</h3></center>
            <p><center>Alarm state: {alarm_state}</center></p> 
            <p><center>Alarm activated: {alarm_activated_time}</center></p>
                          
            <p><center><h3>Water Level Status</center></h3></p>
            <p><center>Water level: {water_level_state}</center></p>
            
            <center><h3>Buzzer Control</h3></center>
            <p><center>Buzzer state: {buzzer_state}</center></p>
            <p><center>Buzzer enabled: {buzzer_enabled}</center></p>
            
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
            <center> <buttON name="MUTE" value="MUTE" type="submit" style="
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

#Flash LED at rate defined
def blink_led(frequency = 0.5, num_blinks = 3):
    for _ in range(num_blinks):
        status_led.off() #Inverted, LED on
        status_led_state = "ON"
        time.sleep(frequency)
        status_led.on() #Inverted, LED off
        status_led_state = "OFF"
        time.sleep(frequency)
        
#Check water level        
def get_water_level():
    level = level_sensor.value() #Read level
    if level == OPEN: #Contacts closed
        time.sleep_ms(CONTACT_BOUNCE_DELAY) #10ms delay 
        level = level_sensor.value() #Check again to confirm valid activation
        if level == OPEN:
            return 'water_high'
    else:
        return 'water_low'
            
#Get time from web
def get_time():
    try:
        start = utime.ticks_ms() #Start timing request
        response = requests.get('https://www.timeapi.io/api/Time/current/zone?timeZone=America/New_York')
        #print("Request took " + str(utime.ticks_ms() - start) + "ms") #Time to get response
        date_time = response.json()['dateTime'] #Get date and time
        date_time = date_time.split('.')
        date_time = date_time[0]
        return date_time
    except:
        print("Could not access time URL")
    return '...'

#Turn buzzer element OFF
def buzzer_off():
    global buzzer_state
    global buzzer_timer_status
    global buzzer_muted
    
    pwm0.deinit() #PWM0 off, GPIO32
    buzzer_state = 'OFF'
    buzzer_muted = False
    buzzer_timer_status = 'BUZZER_TIMER_STOP'

#Turn buzzer element on
def buzzer_on():
    global buzzer_state
    global buzzer_offPeriodCntr_secs
    global buzzer_onPeriodCntr_secs
    global buzzer_offPeriod_secs
    global buzzer_onPeriod_secs
    global_buzzer_timer_status
    
    buzzer_offPeriodCntr_secs = buzzer_offPeriod_secs #Cntr preset with latest value
    buzzer_onPeriodCntr_secs = buzzer_onPeriod_secs #Cntr preset with latest value
    pwm0.init(1200, duty=512) #PWM0 ON,GPIO32
    buzzer_state = 'ON'
    buzzer_timer_status = 'BUZZER_TIMER_RUN'
    
def send_email():
    global email_message
    global sender
    global recipient
    
    print('Attempting to send email')
    smtp.to(recipient)
    smtp.send(email_message)
    smtp.quit()
    
#Connect to WiFi network
async def connect_to_wifi():
    global wlan_connected
    global wlan_connect_time
    global ip_addr
    
    wlan.active(True) #Activate interface
    wlan.connect(ssid, password)

    # Wait for connect or fail
    max_wait = WLAN_TIMEOUT #20 secs
    while max_wait > 0 and wlan.status() != 1010: #1010 for ESP32
        max_wait -= 1
        print('Waiting for connection...{}'.format(max_wait))
        time.sleep(LOOP_REFRESH_SEC)

    # Handle connection error
    if wlan.status() != 1010:
        blink_led(0.5, 1)
        wlan_connected = False
        await asyncio.sleep(2) #2sec    
    else:
        #Connection successful
        blink_led(0.5, 2)
        wlan_connect_time = get_time() #Retrieve time from web
        if wlan_connect_time == '...':
            print('Could not retrieve time from web')
        else:
            print('WiFi Connected at {}'.format(get_time()))
        status = wlan.ifconfig()
        ip_addr = status[0]
        print('ip = {}\n'.format(ip_addr))
        wlan_connected = True

#Web client handler
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
            
    print("Client connected")
    blink_led(0.5, 1)
    
    request_line = await reader.readline() # Read a line
    #print("Request:", request_line)
    
    # We are not interested in HTTP request headers, skip them
    while await reader.readline() != b"\r\n":
        pass
    
    # find valid heater commands within the request
    request = str(request_line)
    print("Request:", request_line)
    
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
    
    stateis = "" # Keeps track of the last command issued
    
    # Carry out a command if it is found (found at index: 8)
    if cmd_control == 8:
        if enable_button_txt == 'DISABLE':
            stateis = "Buzzer disabled"
            #print(stateis)
            enable_button_color = 'red'
            enable_button_txt = 'ENABLE'
            buzzer_enabled = 'DISABLED'
        elif enable_button_txt == 'ENABLE':
            stateis = "Buzzer enabled"
            #print(stateis)
            enable_button_color = 'green'
            enable_button_txt = 'DISABLE'
            buzzer_enabled = 'ENABLED'
        blink_led(0.1, 2)

    if cmd_onPeriod == 8 and buzzer_enabled == 'DISABLED':
        t = request.split() #Extract time value
        t = t[1]
        t = t.split('=')
        buzzer_clientOnPeriod_mins = t[1] #Mins
        print("On period in mins: ",buzzer_clientOnPeriod_mins)
        try:
            buzzer_onPeriod_secs = int(60* float(buzzer_clientOnPeriod_mins)) #CONvert hrs to secONds & update main register
            print("Buzzer on period in seconds: ", buzzer_onPeriod_secs)
            buzzer_onPeriodCntr_secs = buzzer_onPeriod_secs #Update ON period timer counter
            stateis = "On period changed to {}mins".format(buzzer_clientOnPeriod_mins)
        except:
            print('Data input error')
    
    elif cmd_onPeriod == 8 and buzzer_enabled == 'ENABLED':
        stateis = "Can't modify buzzer 'ON TIME' when buzzer enabled"
    
    if cmd_offPeriod == 8 and buzzer_enabled == 'DISABLED':
        t = request.split() #Extract time value
        t = t[1]
        t = t.split('=')
        buzzer_clientOffPeriod_mins = t[1] #Mins
        print("Off period in mins:", buzzer_clientOffPeriod_mins) 
        try:
            buzzer_offPeriod_secs = int(60* float(buzzer_clientOffPeriod_mins)) #CONvert hrs to secONds & update main register
            print("Buzzer off period in seconds: ",buzzer_offPeriod_secs)
            buzzer_offPeriodCntr_secs = buzzer_offPeriod_secs #Update off period timer counter
            stateis = "Off period changed to {}mins".format(float(buzzer_clientOffPeriod_mins))
        except:
            print('Data input error')
            stateis = "Can't modify buzzer off time when enabled"

    elif cmd_offPeriod == 8 and buzzer_enabled == 'ENABLED':
        stateis = "Can't modify buzzer off time when buzzer enabled"
        
    if cmd_mute == 8:
        if buzzer_muted == False:
            buzzer_muted = True
            stateis = "Buzzer muted"
            #print(stateis)
            mute_button_txt = 'UNMUTE'
            mute_button_color = 'red'
        else:
            buzzer_muted = False
            stateis = "Buzzer unmuted"
            #print(stateis)
            mute_button_txt = 'MUTE'
            mute_button_color = 'green'
        blink_led(0.1, 2)
    
    if cmd_refresh == 8:
        stateis = "Page refresh"
        #print(stateis)
        blink_led(0.1, 2)
        
    response = webpage(
            alarm_state, alarm_activated_time, alarm_deactivated_time,
            buzzer_state, buzzer_enabled, buzzer_clientOnPeriod_mins, buzzer_clientOffPeriod_mins,
            mute_button_color, mute_button_txt,
            enable_button_color, enable_button_txt,
            water_level_state, FIRMWARE_VERSION, unit_id, SUMP_PUMP_NUMBER) % stateis

    writer.write('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
    writer.write(response)

    await writer.drain()
    await writer.wait_closed()
            
def stay_alive():
     wdt.feed() #Keeps the watch dog from triggering
     
def level_sensor_interrupt_handler(pin):
    global water_level_state
    global interrupt_pin
    
    #Get water level state
    water_level_state = get_water_level() #Returns 'water_high' or 'water_low'
    interrupt_pin = pin    

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
    global email_message
    global buzzer_muted
    global alarm_state
    global coldstart
    global water_level_state
    global alarm_activated_time
    global alarm_deactivated_time
    global mute_button_txt
    global enable_button_txt
    
    #Set up outputs
    buzzer_off()
    status_led.off()
    pwm0.deinit() # Turn off GPIO32 pwm
    
    #Preset flags
    buzzer_state = 'OFF'
    buzzer_muted = False
    water_level_state = 'water_low'
    buzzer_timer_status = 'BUZZER_TIMER_STOP'
    
    #Preset web template
    mute_button_txt = 'MUTE'
    enable_button_txt = 'DISABLE'
    
    #Set up interrupt pin
    level_sensor.irq(trigger=Pin.IRQ_RISING, handler=level_sensor_interrupt_handler) # Triggers interrupt when level sensor sw opens

    #Restart visual indication
    blink_led(0.25, 2)

    # Get unit ID
    get_id()
    
    #Set first pass flag
    coldstart = True
    
    #Configure wake up
    esp.osdebug(None)
    
    #Main loop
    while True:
        #print("Entering outer while loop")
        wdt.feed() #Keep watch dog from triggering
        
        if  wake_reason() == 2:       
            #print("Water level sensor interrupt, entering alarm handler")
            blink_led(0.25, 2)
            water_level_state = get_water_level()
            
            #Inner loop
            while water_level_state  == 'water_high':
                #print("Entering alarm handler loop")
                
                #Connect to WiFi     
                if not wlan_connected:
                    print('...Connecting to WiFi')                                                                                      
                    asyncio.create_task(connect_to_wifi()) 

                if wlan_connected and (server_connect_state == False):
                    print('...Setting up webserver') #Start a TCP server on the given host and port. The callback will be called with incoming, accepted connections,
                    asyncio.create_task(asyncio.start_server(serve_client, "0.0.0.0", 80))  #and be passed 2 arguments: reader and writer streams for the connection. Returns a Server object.
                    print('Web server ready\n')
                    server_connect_state = True
                    
                await asyncio.sleep(LOOP_REFRESH_SEC) #1sec
                        
                #Get water level state
                water_level_state = get_water_level() #Returns 'water_high' or 'water_low', need to check each loop
        
                #Process water level result
                if water_level_state == 'water_high' and alarm_state == 'inactive' and buzzer_muted == False and buzzer_enabled =='ENABLED': #Water level goes 'high'
                    buzzer_on() #Buzzer turned on
                    alarm_state = 'active' #Activate alarm
                    buzzer_timer_status = 'BUZZER_TIMER_RUN' #Start timer
                    print("Sump pump water level high") #Setup SMS
                   
                    #Get time
                    result = get_time()
                    #print("Result ", result)
                    if result == '...':
                        alarm_activated_time = 'Time unavailable...'
                    else:
                        alarm_activated_time = result
                        print("Alarm activated at ", alarm_activated_time)
                    alarm_deactivated_time = '...'
                    
                    #Send email
                    if wlan_connected == True:
                        print("Attempt to send email")
                        #smtp = umail.SMTP('smtp.gmail.com', 465, ssl=True) # Gmail's SSL port
                        smtp = umail.SMTP('smtp.gmail.com', 587, ssl=False) # Gmail's TLS port
                        print("Log in to email")
                        smtp.login(USER_NAME, MAIL_PASS)
                        smtp.to(USER_NAME)
                        smtp.write("From:" + "ESP32" + "<"+ USER_NAME + ">\n")
                        email_message = "Alarm: Pump " + SUMP_PUMP_NUMBER + " at " + alarm_activated_time + " \n\r Water level high, alarm activated!"
                        smtp.write("Subject:" + email_message + "\n")
                        smtp.write("Hello from ESP32")
                        smtp.send()
                        smtp.quit()
                    else:
                        print("WLAN not connected, unable to send email")
 
                if water_level_state == 'water_high' and alarm_state == 'active' and buzzer_muted == False and buzzer_enabled =='DISABLED':
                    pwm0.deinit() #PWM off
                    buzzer_state = 'OFF'

                if water_level_state == 'water_low' and alarm_state == 'active': #Water level goes 'low'
                    buzzer_off() #Buzzer turned off
                    buzzer_state = 'OFF'
                    water_level_state = 'water_low'
                    alarm_state = 'inactive'
                    buzzer_muted = False
                    buzzer_button_txt = 'MUTE'
                    print("Sump pump water level normal")
                    
                    #Get time
                    result = get_time()
                    if result == '...':
                        alarm_deactivated_time = 'Time unavailable...'
                    else:
                        alarm_deactivated_time = result
                        print("Alarm deactivated at ", alarm_deactivated_time)
                    sms_message = "Alarm deactivated at sump pump " + SUMP_PUMP_NUMBER + " at " + alarm_deactivated_time + "\n\r Water level normal!"
                    print("Sending alarm deactivated sms")
                    send_sms()
                
                #Task when buzzer on and buzzer timer active
                if buzzer_timer_status == 'BUZZER_TIMER_RUN' and buzzer_state == 'ON' and buzzer_muted == False and buzzer_enabled == 'ENABLED':
                    if buzzer_onPeriodCntr_secs != 0:
                        print("Buzzer_onPeriodCntr_secs", buzzer_onPeriodCntr_secs)
                        buzzer_onPeriodCntr_secs = buzzer_onPeriodCntr_secs - 1 #Decrement on period cntr      
                    else: 
                        pwm0.deinit() #Buzzer turned off
                        buzzer_state = 'OFF'
                        result = get_time()
                        if result == '...':
                            buzzer_swoff_time = 'Time unavailable...'
                        else:
                            buzzer_swoff_time = result
                        #print("Buzzer_turned off: {}".format(result))
                        buzzer_offPeriodCntr_secs = buzzer_offPeriod_secs #Cntr preset
                                
                if buzzer_timer_status == 'BUZZER_TIMER_RUN' and buzzer_state == 'OFF' and buzzer_muted == False and buzzer_enabled == 'ENABLED':
                    if buzzer_offPeriodCntr_secs != 0:
                        print("Buzzer_offPeriodCntr_secs", buzzer_offPeriodCntr_secs)
                        buzzer_offPeriodCntr_secs = buzzer_offPeriodCntr_secs - 1 #Decrement off period cntr    
                    else:
                        pwm0.init(1200, duty=512) #Buzzer turned on
                        buzzer_state = 'ON'
                        result = get_time()
                        if result == '...':
                            buzzer_swon_time = 'Time unavailable...'
                        else:
                            buzzer_swon_time = result
                        #print("Buzzer_re-started: {}".format(result)) 
                        buzzer_onPeriodCntr_secs = buzzer_onPeriod_secs #Cntr preset
                    
                if buzzer_timer_status == 'BUZZER_TIMER_RUN' and buzzer_state == 'ON' and buzzer_muted == True and buzzer_enabled == 'ENABLED':
                    pwm0.deinit() #PWM off
                    buzzer_state = 'OFF'
                    
                if buzzer_timer_status == 'HEATER_TIMER_STOP' and (buzzer_state == 'OFF' or buzzer_state == 'ON'): #Default
                    buzzer_off()
                    water_level_state = 'water_low'
                    alarm_state = 'inactive'
                      
                if wlan.isconnected() == False:
                    print('Network not connected...')
                    await asyncio.sleep(5) #5sec
                    print('Confirming network not connected')
                    if wlan.isconnected() == False: #Confirm disconnected
                        #Config for retry 
                        wlan_connected = False #Re-connect flag cleared
                        server_connect_state = False #Server flag cleared
                        wlan.active(False) #Deactivate interface
                        wdt.feed() #Keep watch dog from triggering
                        await asyncio.sleep(5)
                        wdt.feed() #Keep watch dog from triggering
                        
                wdt.feed() #Keep watch dog from triggering
                #print("Going back to inner loop")
                #Inner loop...
                
        elif wake_reason() == 4: #Time out interrupt
            print("Keep alive timeout interrupt")
            
        wdt.feed() #Keep watch dog from triggering
        #print("Blink LED 1 time")
        blink_led(0.25, 1)
        # put the device into deep sleep for 20 seconds
        esp32.wake_on_ext0(pin = level_sensor, level = esp32.WAKEUP_ANY_HIGH)
        print("Going to sleep")
        time.sleep(1) #Delay to allow for print command
        lightsleep(20000)
        #Main loop...
    
try:
    asyncio.run(main())
except KeyboardInterrupt:
    print('Exiting program on keyboard interrupt')
    sys.exit()
finally:
    asyncio.new_event_loop()   #Reset the event loop and return it.

