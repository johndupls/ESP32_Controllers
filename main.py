"""
    BME Temperature Sensor with Wi-Fi, no display
    Version: V1.4
    Date:2025-09-25
    Static IP Address: 192.168.2.15 

    Updates:
        Window closure status added to webpage
      
    Latest Updates:
        Added temperature sensor number
        Added notConnectedCounter to cold start setup_variables()
        Replaced sys.exit() with machine reset in I2C get_data() functions
"""

# Imports
import time
import network
import ntptime
import uasyncio as asyncio
import machine
from machine import Pin, I2C, WDT, Timer, RTC, idle
import bme280
import sys
from credentials import WIFI_NAME, WIFI_PASS, TEMP_SENSOR_NUMBER, STATIC_ADDR, API_KEY
import gc
import errno
import urequests

# Const declarations
FIRMWARE_VERSION = '1.4'
INTERVAL_SEC = 0.25
LOOP_REFRESH_SEC = 2.0
ON = 1
OFF = 0
HIGH = 1
LOW = 0
ENABLED = 1
DISABLED = 0
UTC_OFFSET = 4 * 60 * 60  # Seconds, Ottawa offset = 4/5
CLIENT_REFRESH_PERIOD = 30 # Seconds
CTRL_LIVE_PERIOD = 120 # Flash LED every 120 seconds
GC_TIMEOUT = 1800 # 30mins x 60
EXT_WEATHER_PERIOD = 60 # Check weather every 15 min

# Global timer variables
timer_tick = False
first_pass = False
local_time = ''
ctrl_live_counter = CTRL_LIVE_PERIOD
external_dewpoint_counter = EXT_WEATHER_PERIOD

# Garbage collection timeout
gc_timeout_counter = GC_TIMEOUT

# Global 'sensor' data variables
amb_temp = ''
pressure = ''
humidity = ''
sensor_dewpoint = ''
sensor_status = 'Unknown'

# Global 'external' data variables
external_temp = ''
external_humidity = ''
external_pressure = ''
external_dewpoint = ''
external_weather = ''
window_permissions = ''

#Global controller variables
unit_id = ''

# Global startup variable
coldstart = False

# Global WLAN variables
ip_addr = ''
wlan_connect_time = ''  # Time and date connected to network
server_connect_state = False  # False indicates server disconnected
notConnectedCounter = 0 # Number of re-connect attempts
wlan_reconnect = False
rssi = '' # Signal strength

# I2C device addresses
bmp_addr1 = 0x76  # BMP280 address 1
bmp_addr2 = 0x77  # BMP280 alternative address 2

# I2C variables
num_I2C_devices = 0  # Number of I2C devices detected
i2c_addr_found = [] # List of valid addresses

# Create LED object
status_led = Pin(33, Pin.OUT, value=1)  # LED off
status_led_state = ''  # LED state "ON" or "OFF"

# Create WLAN object
wlan_connected = False
WLAN_TIMEOUT = 20  # Number of attempts to connect. Period = WLAN_TIMEOUT * LOOP_REFRESH_SEC
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

# Get unit id
def  get_id():
    global unit_id
    
    id = STATIC_ADDR
    id = id.split(".")
    unit_id = id[3] # Last value in address used as ID

# Cold start variable setup
def setup_variables():
    global ip_addr
    global wlan_connect_time
    global server_connect_state
    global first_pass
    global window_permissions
    global notConnectedCounter

    ip_addr = '0,0,0,0'
    wlan_connect_time = '...'
    server_connect_state = False
    window_permissions = 'Keep closed'
    notConnectedCounter = 0

# Tim 0 callback function
def tim0_callback(tim0):
    global ctrl_live_counter
    global gc_timeout_counter
    global external_dewpoint_counter

    if ctrl_live_counter != 0: # Decrement while counters not zero
        ctrl_live_counter -= 1
        
    if gc_timeout_counter > 0:
        gc_timeout_counter -= 1

    if external_dewpoint_counter > 0:
        external_dewpoint_counter -= 1
        
# Get RF sensitivity
def get_rssi():
    global rssi
    
    try:
        result = wlan.status('rssi')
        if result <=-50 and result >= -64:
            print('RSSI...strong signal: {}dBm'.format(result))
        elif result <= -65 and result >= -79:
            print('RSSI...moderate signal: {}dBm'.format(result))
        elif result <= -80:
            print('RSSI...exceeding minimum acceptable signal for connection: {}dBm'.format(result))
        print("\n")
        rssi = str(result)
        except:
            print('WLAN problem')

# Create server webpage
def webpage(
            amb_temp, pressure, humidity, sensor_dewpoint, ip_addr, FIRMWARE_VERSION, unit_id, rssi, local_time,
            window_permissions, external_temp, external_pressure, external_humidity, external_dewpoint, external_weather,TEMP_SENSOR_NUMBER):
 
    # HTML Template
    html = f"""
            <!DOCTYPE html>
            <html lang="en">
           
            <head>
                <meta charset="UTF-8">
                <meta name="viewport" content="width=device-width, initial-scale=1">
                <link rel="icon" href="data:,">
                <title>Internal Temp Sensor</title>
                <meta http-equiv="refresh" content={CLIENT_REFRESH_PERIOD}>
                <!--meta http-equiv="refresh" content="15; URL=192.168.2.xx"/-->
            </head>
               
            <body>
        
            <p><center><h2>Internal Temperature Sensor{TEMP_SENSOR_NUMBER} {FIRMWARE_VERSION}</h2></center></p>
            
            <p><center>Local Date: <em>{local_time[0]}:{local_time[1]}:{local_time[2]}</em> &nbsp Local Time: <em>{local_time[4]}:{local_time[5]}:{local_time[6]}</em></center></p>
            <p><center>Unit ID: <em>{unit_id}</em> &nbsp Signal Strength: <em>{rssi}dBm</em></center></p>
            <p><center>IP Address:<em> {ip_addr}</em> </center></p>
            
            <p><center><h3>Internal Sensor Data</center></h3></p> 
            <p><center>Temperature:<em> {amb_temp}DegC</em></center></p>
            <p><center>Pressure:<em> {pressure}</em></center></p>
            <p><center>Humidity:<em> {humidity}%</em></center></p>
            <p><center>Dewpoint:<em> {sensor_dewpoint}DegC</em></center></p>
            
            <p><center><h3>External Weather Data</center></h3></p>
            <p><center>Temperature:<em> {external_temp}DegC</em></center></p>
            <p><center>Pressure:<em> {external_pressure}hPa</em></center></p>
            <p><center>Humidity:<em> {external_humidity}%</em></center></p>
            <p><center>Dewpoint:<em> {external_dewpoint}DegC</em</center></p>
            <p><center>Weather:<em> {external_weather}</em</center></p>
            
            <p><center><h3>Window Status</center></h3></p>
             <p><center><em> {window_permissions}</em> </center></p>
            
            </body>
            </html>
            """
    return str(html)


# Search for any devices on bus
def find_I2C_devices():
    # Scan
    print('Scanning for I2C devices')
    devices = I2C1.scan()
    if len(devices) >= 1:
        print('...{} x I2C devices found'.format(len(devices)))
    else:
        print('...{} x I2C device found'.format(len(devices)))
    # Return number of devices
    return devices #List of devices


# Create I2C object
def create_I2C_obj(addr,addr_list):
    if addr in addr_list: 
        if addr == bmp_addr1 or addr == bmp_addr2:
            # print("Creating BME I2C instance for {}...\n".format(hex(addr)))
            dev_obj = bme280.BME280(i2c=I2C1, address=addr)  # Create BME280 object
            return dev_obj
        

def test_I2C():
    global i2c_addr_found
    
    # Scan for I2C device/s
    I2C_devices = find_I2C_devices()
    count = 0
    
    # Compare found I2C devices to expected addresses
    if len(I2C_devices) != 0:
        for d in I2C_devices:  # Iterate through devices
            if d == bmp_addr1:
                print('...BME280 device found @ address={}'.format(hex(d)))
                i2c_addr_found.append(bmp_addr1)
                count += 1
            elif d == bmp_addr2:
                print('...BME280 device found @ address={}'.format(hex(d)))
                i2c_addr_found.append(bmp_addr2)
                count += 1
            else:
                count = 0
        if count == 0:
            print('Device addresses not recognised')
            return False
        else:
            print('{} x I2C devices recognized'.format(count))
            return True
    elif len(I2C_devices) == 0:
        print('No I2C devices found')
    return False


# Flash LED at rate defined
def blink_led(frequency=0.5, num_blinks=3):
    global status_led_state

    for _ in range(num_blinks):
        status_led.off()  # Inverted, LED on
        status_led_state = 'ON'
        time.sleep(frequency)
        status_led.on()  # Inverted, LED off
        status_led_state = 'OFF'
        time.sleep(frequency)


# Get sensor data
def get_sensor_data(bmp):
    global amb_temp
    global pressure
    global humidity
    global sensor_status

    try:
        # Get sensor temperature data
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

# Calculate dew point
def dewpoint_calc(temp, humidity):
    if temp == '' or humidity == '':
        return ''
    else:
        temp_dp = 0.0
        temp_dp = float(temp) - ((100-float(humidity))/5)
        temp_dp = round(temp_dp,2)
        return str(temp_dp) 

# Setup real time clock
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
    except:
        print('RTC error')
        return False

# Connect to Wi-Fi network
async def connect_to_wifi():
    global WLAN_TIMEOUT
    global wlan_connected
    global wlan_connect_time
    global ip_addr
    global local_time

    wdt.feed()  # Keep watch dog from triggering

    wlan.active(True)  # Activate interface
    wlan.ifconfig( (STATIC_ADDR, '255.255.255.0', '192.168.2.1', '192.168.2.1')) # Set static address, subnet, gateway & dns
    wlan.connect(WIFI_NAME, WIFI_PASS)

    # Wait for connect or fail
    max_wait = WLAN_TIMEOUT  # 20 secs
    while max_wait > 0 and wlan.status() != 1010:  # 1010 for ESP32
        max_wait -= 1
        print('...Waiting for connection...{}'.format(max_wait))
        await asyncio.sleep(LOOP_REFRESH_SEC)
        wdt.feed()  # Keep watch dog from triggering

    # Handle connection error
    if wlan.status() != 1010:
        blink_led(0.1, 5)
        wlan_connected = False
        await asyncio.sleep(2)  # 2sec
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
        
        # Get weather data on start up
        request_weather_data() #Get external sensor_dewpoint and compare to internal sensor
        
        status = wlan.ifconfig()
        ip_addr = status[0]
        print('IP Address: ', ip_addr)
        print('...WLAN parameters: {}\n'.format(wlan.ifconfig()) )
        wlan_connected = True
   
# Client handler
async def serve_client(reader, writer):
    global amb_temp
    global pressure
    global humidity
    global sensor_dewpoint
    global FIRMWARE_VERSION
    global ip_addr
    global unit_id
    global window_permissions

    wdt.feed()  # Keep watch dog from triggering
    
    # print("Client connected")
    blink_led(0.1, 1)

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

    # find valid heater commands within the request
    request = str(request_line)
    print('Request:\n', request_line)

    cmd_refresh = request.find('REFRESH=REFRESH')

    # Carry out a command if it is found (found at index: 8)
    if cmd_refresh == 8:
        blink_led(0.1, 2)

    wdt.feed()  # Keep watch dog from triggering every second

    # Free memory
    gc.collect() #Run garbage collection
    free_mem = gc.mem_free()
    print('Memory freed for response: ', free_mem)
    if free_mem < 10000:
        print('Not enough memeory for response message')
        return

    try:
        response = webpage(amb_temp, pressure, humidity, sensor_dewpoint, ip_addr, FIRMWARE_VERSION, unit_id, rssi,
                                               local_time, window_permissions, external_temp, external_pressure, external_humidity, external_dewpoint, external_weather, TEMP_SENSOR_NUMBER)
        writer.write('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
        writer.write(response)
        await writer.drain()
        await writer.wait_closed()
        print('Client disconnected')
    except OSError as exc:
        # Free memory
        if exc.errno == errno.ENOMEM:
            gc.collect()
            print('Free memory: ', gc.mem_free())
            

def wlan_test():
     # Handle connection status
    if  wlan.status() == 1010:
        print('WiFi connected') 
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
    return wlan.status()

# Check if okay to open windows. Okay if external > internal sensor dewpoint.
def request_weather_data():
    global window_permissions
    global external_temp
    global external_humidity
    global external_pressure
    global external_dewpoint
    global external_weather
        
    # Get dew point from the web
    url = f'http://api.openweathermap.org/data/2.5/weather?q=Ottawa,ca&appid=22f3788997c8b8ae89d3f787d4be2bfc&units=metric' # Standard mode
    print('Fetching weather data...')
    
    try:
        response = urequests.get(url)
        
        if response.status_code == 200:
            data = response.json()
            
            print('Weather data')
            print(f"City: {data['name']}")
            print(f"Temperature: {data['main']['temp']}°C")
            print(f"Pressure: {data['main']['pressure']}hPa")
            print(f"Humidity: {data['main']['humidity']}%")
            print(f"Weather: {data['weather'][0]['description']}")

            external_temp = data['main']['temp']
            external_pressure = data['main']['pressure']
            external_humidity = data['main']['humidity']
            external_weather = data['weather'][0]['description']
            
            # Calculate external sensor_dewpoint
            external_dewpoint = dewpoint_calc(external_temp, external_humidity)
            
            # Compare web sensor_dewpoint to internal sensor
            if external_dewpoint < sensor_dewpoint:
                window_permissions = 'Windows may be opened'
            else:
                window_permissions = 'Keep windows closed'
            
        else:
            print(f"Failed to fetch data. Status code: {response.status_code}")   
        response.close()
            
    except Exception as e:
        print(f"Skipping weather request.......Error: {e}") 
            
# Main loop
async def main():
    global LOOP_REFRESH_SEC
    global server_connect_state
    global wlan_connected
    global sensor_status
    global first_pass
    global local_time
    global ctrl_live_counter
    global status_led_state
    global wlan_reconnect
    global notConnectedCounter
    global i2c_addr_found
    global gc_timeout_counter
    global external_dewpoint_counter
    global sensor_dewpoint

    # Start timer 0
    tim0.init(period=1000, mode=Timer.PERIODIC, callback=tim0_callback)

    # Restart visual indication
    blink_led(0.1, 5)

    # Look for I2C devices
    if test_I2C() == False:
        sensor_status = 'Sensor error'
        blink_led(1, 10)
        print('I2C error')
        sys.exit()
    else:
        sensor_status = 'Sensor active'
        print('Creating I2C objects')
        if bmp_addr1 in i2c_addr_found:
            bmp = create_I2C_obj(bmp_addr1, i2c_addr_found)
        elif bmp_addr2 in i2c_addr_found:
            bmp = create_I2C_obj(bmp_addr2, i2c_addr_found)
 
    # Preset states
    status_led_state = 'OFF'
    coldstart = True

    # Set up outputs
    status_led.off()

    # Get data
    if sensor_status == 'Sensor active':
        get_sensor_data(bmp)
        sensor_dewpoint = dewpoint_calc(amb_temp, humidity)   # Calculate dew point
    elif sensor_status == 'Sensor error':
        await asyncio.sleep(2)  # 2sec
        get_sensor_data(bmp)  # Try again
        if sensor_status == 'Sensor error':
            blink_led(0.1, 10)
            print('Failed I2C retry...Resetting')
            await asyncio.sleep(1)
            machine.reset()  # Reset ESP
            
    # Update unit ID
    get_id() # Last value in IP addr    

    wdt.feed()  # Keep watch dog from triggering

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

        #print('Entering idle')
        idle()
        #print('2 sec sleep')
        await asyncio.sleep(LOOP_REFRESH_SEC)  # 2 sec


        # Update RTC  global storage variable
        local_time = rtc.datetime()
        
        # Check garbage collection timeout
        if gc_timeout_counter == 0:
            gc.collect() #Free up memory
            print('Memory free: ', gc.mem_free())
            gc_timeout_counter = GC_TIMEOUT #Preset counter
        
        # Check controller alive counter
        if ctrl_live_counter == 0 :
            blink_led(frequency=0.05, num_blinks=1) #Flash LED
            ctrl_live_counter = CTRL_LIVE_PERIOD #Preset counter
        
        # Get external sensor_dewpoint
        if external_dewpoint_counter == 0 :
            request_weather_data() #Get external sensor_dewpoint and compare to internal sensor
            external_dewpoint_counter = EXT_WEATHER_PERIOD #Preset counter

        # Refresh data
        if sensor_status == 'Sensor active':
            get_sensor_data(bmp)
            sensor_dewpoint = dewpoint_calc(amb_temp, humidity)
        elif sensor_status == 'Sensor error':
            await asyncio.sleep(2)  # 2sec
            get_sensor_data(bmp)  # Try again
            if sensor_status == 'Sensor error':
                blink_led(0.1, 10)
                print('Failed I2C retry...Resetting')
                await asyncio.sleep(1)
                machine.reset()  # Reset ESP
            
        # Print wlan flags
        print('wlan_connected:', wlan_connected)
        print('server_connect_state', server_connect_state)
        print('Config:', wlan.ifconfig())
        
        #Get RSSI
        get_rssi()
        
         # Check wlan connection state
        wifi_state  = wlan_test()
        if wifi_state != 1010: # Check if wlan connected
            wlan_disconnect_time = str(local_time[0]) + ':' + str(local_time[1])  + ':' +  str(local_time[2])  + '...' +  str(local_time[4])  + ':' +  str(local_time[5]) + ':' +  str(local_time[6])
            #print('Network connected at ', wlan_connect_time)
            print('Network disconnected at ', wlan_disconnect_time)
            # Config for restart
            wlan_connected = False  # Re-connect flag cleared
            server_connect_state = False  # Server flag cleared
            wlan_reconnect = True # Indicate re connection is required
            if notConnectedCounter > 10: # Check re-connect timeout counter
                print('Resetting ESP32...')
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


