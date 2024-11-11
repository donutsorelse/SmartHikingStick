import serial
import threading
from queue import Queue
from notecard import notecard
from pinpong.board import *
from pinpong.extension.unihiker import *
from time import sleep, time
from datetime import datetime, timedelta
from math import radians, cos, sin, sqrt, atan2
import pygame
import sys
import logging
import pyttsx3
import speech_recognition as sr
import re
import os
import paho.mqtt.publish as publish

sys.path.append("/root/mindplus/.lib/thirdExtension/liliang-gravitygnss-thirdex")

# Configuration
PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
PRODUCT_UID = "com.<company name>:<project name>"

# Constants for GNSS Device
GNSS_DEVICE_ADDR = 0x20
MODE_GPS_BEIDOU_GLONASS = 0x07
I2C_LAT_1 = 0x07
I2C_LON_1 = 0x0D

home_location = {"lat": 0, "lng": 0, "radius": 50}  # Home radius in meters
home_set = False
walk_active = False
walk_start_time = None
user_left_home=False # in case the user starts the walk themselves, we need to ensure they leave the house before eventually triggering the walk end when they return home
walk_extension = 0
all_weather_updates = False # whether or not to say every weather update, or only inform the user when there's bad weather
lat=0
lng=0

walk_timer = None
fell_timer = None

activate_blues_alerts = False
notecard_port = None

# Specify the card and device number - this seems to be needed for it to work correctly
os.environ['ALSA_CARD'] = '1'  # Use card 1 for the onboard audio codec
os.environ['ALSA_DEVICE'] = '0'
os.environ['AUDIODEV'] = 'hw:1,0'

# Initialize TTS engine
engine = pyttsx3.init()

# Directly set the voice to "english" (from running some tests it seems to sound better)
engine.setProperty('voice', 'english')

# Set volume to maximum
engine.setProperty('volume', 1.0)

# MQTT Broker Settings
BROKER = "<ip_of_your_server>"
PORT = 1883
TOPIC = "home/<your_topic>"
USERNAME = "<your_username>"
PASSWORD = "<your_password>"

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
settings_file = 'settings.json'

if not os.path.exists(settings_file):
    print("No settings file. Creating one.")
    with open(settings_file, 'w') as file:
        file.write('{}')    # start with an empty file if it doesn't exist - we'll write to it with defaults shortly
    print(f"Created default settings file at {settings_file}")

def save_settings(settings):
    try:
        with open(settings_file, 'w') as file:
            json.dump(settings, file)
        logging.info("Settings saved successfully.")
    except Exception as e:
        logging.error(f"Error saving settings: {e}")

def load_settings():
    try:
        settings = {
                "home_location": {"lat": None, "lng": None},
                "home_set": False,
                "ultrasonic_sensor_enabled": True,
                "activate_blues_alerts": True,
                "all_weather_updates": False
            }
            
        if os.path.exists(settings_file):
            with open(settings_file, 'r') as file:
                settings = json.load(file)
                home_location = settings.get('home_location', {"lat": None, "lng": None})
                home_set = settings.get('home_set', False)
                ultrasonic_sensor_enabled = settings.get('ultrasonic_sensor_enabled', True)
                activate_blues_alerts = settings.get('activate_blues_alerts', True) 
                all_weather_updates = settings.get('all_weather_updates', False)
                logging.info("Settings loaded successfully.")
                return settings, home_location, home_set, ultrasonic_sensor_enabled, activate_blues_alerts, all_weather_updates
        else:
            logging.warning("Settings file not found. Creating default settings.")
            save_settings(settings)
            
    except Exception as e:
        logging.error(f"Error loading settings: {e}")
        save_settings(settings)
    return settings, settings['home_location'], settings['home_set'], settings['ultrasonic_sensor_enabled'], settings['activate_blues_alerts'], settings['all_weather_updates']

settings, home_location, home_set, ultrasonic_sensor_enabled, activate_blues_alerts, all_weather_updates = load_settings()

geofences = [
    {"name": "add your locations here", "center": {"lat": 31.0959, "lng": -73.2668}, "radius": 25000, "triggered": False}
]

Board().begin()
ultrasonic_sensor_up = SR04_URM10(Pin(Pin.P0), Pin(Pin.P1))
ultrasonic_sensor_down = SR04_URM10(Pin(Pin.P2), Pin(Pin.P8))
tone = Tone(Pin(Pin.P26))  # Create a tone object with Pin 26
LOG_ULTRASONIC_VALUES = False  # These values are useful but can be annoying if you're debugging something else

#Credit to "Lupin" from GNSS tutorial - this is a simplified version of the class they use (only keeping what we need for the smart hiking stick)
class DFRobot_GNSS_I2C:
    def __init__(self, i2c_addr=0x20, bus=0):
        self._i2c = I2C(bus)
        self._addr = i2c_addr
        self._lock = threading.Lock()

    def set_gnss_mode(self, mode):
        with self._lock:
            self._i2c.writeto_mem(self._addr, 0x22, bytearray([mode]))
            sleep(0.1)

    def enable_power(self):
        with self._lock:
            self._i2c.writeto_mem(self._addr, 0x23, bytearray([0x00]))

    def get_coordinate(self, reg):
        with self._lock:
            try:
                result = self._i2c.readfrom_mem(self._addr, reg, 6)
                if result is None:
                    raise ValueError("Received no data from the GNSS sensor.")
                degree = result[0] + result[1] / 60.0 + (result[2] * 65536 + result[3] * 256 + result[4]) / 100000.0 / 60.0
                return degree
            except Exception as e:
                logging.error(f"Unexpected error encountered: {str(e)}")
                return None  # Handle other possible exceptions

    def get_latitude(self):  
        # return 31.5582  # Test loc (Burnt Corn Alabama) if/when your gnss is red (needs to be green to get values) - uncomment to test items that require gps while in low signal areas
        with self._lock:
            global lat
            lat = self.get_coordinate(0x07)
            return self.get_coordinate(0x07)  # Latitude register

    def get_longitude(self):
        # return 87.1728 # Test loc
        with self._lock:
            global lng
            lng = self.get_coordinate(0x07)
            return self.get_coordinate(0x0D)  # Longitude register

def set_home_location():
    global settings, home_location, home_set
    # latitude = get_latitude()
    # longitude = get_longitude()
    global lat,lng
    if lat is not None and lng is not None:
        settings['home_location'] = {"lat": lat, "lng": lng}
        settings['home_set'] = True
        home_set = True
        save_settings(settings)
        logging.info(f"Home location set to latitude: {lat}, longitude: {lng}")
        speak("Home location set")
        
        home_location["lat"] = lat
        home_location["lng"] = lng
    else:
        settings['home_set'] = False
        home_set = False
        speak("Failed to set home location. Please try again.")

# Save home location to file
def save_home_location(location):
    with open(home_location_file, 'w') as file:
        json.dump(location, file)
    logging.info("Home location saved.")

def check_geofence(latitude, longitude, geofence):
    distance = haversine_distance(latitude, longitude, geofence["lat"], geofence["lng"])
    return distance <= geofence["radius"]

def manage_walk():
    global walk_active, walk_start_time, walk_extension, home_location
    current_time = datetime.now()
    global lat,lng
    if not walk_active and check_geofence(lat,lng,home_location):
        # User leaves home geofence
        walk_active = True
        user_left_home = True
        walk_start_time = current_time
        logging.info("Walk has started.")
        speak("You have started a walk.")
    
    elif walk_active:
        if check_geofence(lat,lng,home_location) and user_left_home:
            # User returns to home geofence
            walk_active = False
            user_left_home = False
            logging.info("Walk has ended.")
        elif (current_time - walk_start_time).total_seconds() > 3600 + walk_extension:
            user_left_home = True

            # Walk time exceeded, check for extension or send alert
            speak("You've passed your expected walk duration.  Are you ok?  Say. Yes. or. no. ") # we check for other responses, but yes or no is easier to understand.  Adding periods for tts clarity.

            # Listen for a response from the user
            response = listen_for_response()
            response_evaluation = evaluate_response(response) # Provides a positive or negative response, so we're just using a simple flag to represent that
            
            if response_evaluation:
                print("User is ok - Extend the walk")
                speak("Extending walk duration.")
                walk_timer.cancel()
            elif response_evaluation == False:
                send_alert("User has fallen and is injured.  Last known location is:") # we always send location data
            if not button_b.is_pressed():  # Assuming button B is used for extending the walk
                send_alert("Walk time exceeded without extension.")
                logging.info("Alert sent - user passed walk time.")
            else:
                walk_extension += 1800  # Extend by 30 minutes
                logging.info("Walk time extended by 30 minutes.")
        else:
            user_left_home = True

def calculate_pitch(distance, is_up, min_distance, max_distance):
    print("calculate_pitch")
    try:
        if distance <= min_distance:
            if LOG_ULTRASONIC_VALUES:
                print(f"Calculating pitch for very close distance: {distance} cm")
            return 440  # Lowest pitch for close proximity
        elif distance > max_distance:
            pitch = 880 if is_up else 220
            if LOG_ULTRASONIC_VALUES:
                print(f"Calculating pitch for distance beyond max threshold: {distance} cm, Pitch: {pitch}")
            return pitch  # High pitch for up, low pitch for down for far distances
        else:
            # Scale pitch dynamically based on the distance
            dynamic_pitch = int(440 + (440 * (distance - min_distance) / (max_distance - min_distance))) if is_up \
                else int(220 - (110 * (distance - min_distance) / (max_distance - min_distance)))
            if LOG_ULTRASONIC_VALUES:
                print(f"Calculating dynamic pitch for distance: {distance} cm, Pitch: {dynamic_pitch}")
            return dynamic_pitch
    except Exception as e:
        if LOG_ULTRASONIC_VALUES:
            print(f"Error calculating pitch for distance: {distance} cm, is_up: {is_up}. Error: {str(e)}")
        return 440  # Default to a safe pitch if there's an error so the user still knows that there's an obstacle

def get_distance():
    return ultrasonic_sensor.distance_cm()

gnss = DFRobot_GNSS_I2C()
gnss.set_gnss_mode(0x07)
gnss.enable_power()

def gps_monitor():

    while True:
        latitudes = []
        longitudes = []
        for _ in range(5):  # Collect 5 samples rapidly
            latitudes.append(gnss.get_latitude())
            longitudes.append(gnss.get_longitude())
            sleep(0.2)  # Rapid sampling interval

        # Pull the median to get rid of outliers - outliers would still skew an average, so the median is safest
        median_latitude = sorted(latitudes)[len(latitudes)//2]
        median_longitude = sorted(longitudes)[len(longitudes)//2]
        global lat,lng
        lat=median_latitude
        lng=median_longitude

        location_name = check_geofence(median_latitude, median_longitude)
        if location_name:
            send_message_over_blues(f"User has arrived at {location_name}")

        sleep(60)  # Wait for a minute before next readings

def haversine_distance(lat1, lon1, lat2, lon2):
    # Calculate the great circle distance between two points
    R = 6371000  # Earth radius in meters
    dLat = radians(lat2 - lat1)
    dLon = radians(lon2 - lon1)
    a = sin(dLat/2) ** 2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon/2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c

def button_watch():
    last_state_a = False
    last_state_b = False
    try:
        while True:
            current_state_a = button_a.is_pressed()
            current_state_b = button_b.is_pressed()

            if current_state_a and not last_state_a:
                logging.info("Button A pressed")
                handle_button_press()
            if current_state_b and not last_state_b:
                logging.info("Button B pressed")
                sos_button_pressed()

            last_state_a = current_state_a
            last_state_b = current_state_b

            sleep(0.1)
    except KeyboardInterrupt:
        logging.info("Button watch stopped.")

def speak(text):
    print("Tts: ",text)
    engine.say(text)
    engine.runAndWait()
    print("Tts done")

if not home_set:
    speak("Say set home to set your home location, or press button a")

def sos_button_pressed():
    global sos_sent
    if sos_sent:
        logging.info("SOS Cancelled.")
        send_alert("SOS alert has been cancelled. Last known location is: ") # We always send the location with all messages
        speak("Emergency alert cancelled.")
        sos_sent = False  # Reset the SOS sent flag
    else:
        logging.info("SOS activated")
        send_alert("SOS! User needs help at location: ")
        speak("Emergency alert sent.")
        sos_sent = True

def send_alert(message):
    global lat,lng
    location= f"Latitude: {lat}, Longitude: {lng}"
    if activate_blues_alerts:
        req = {"req": "note.add", "body": {"text": message + f" Last known coordinates: {location}"}}
        rsp = notecard_port.Transaction(req)
        print(f"Alert sent: {rsp}")

fall_detected = False
def handle_button_press():
    logging.info("Button was pressed!")

    if not home_set:
        set_home_location()
    if fall_detected:
        fall_detected = False

def detect_fall():
    global fall_detected, walk_timer
    g = 9.81
    threshold_acceleration = 1.15 * g
    upright_threshold = 0.1 * g
    cos_threshold_angle = math.cos(math.radians(90 - 30))   # adjust the 30 here if you want to adjust your angle threshold

    while True:
        Ax = accelerometer.get_x() / 16384.0
        Ay = accelerometer.get_y() / 16384.0
        Az = accelerometer.get_z() / 16384.0
        
        # Calculate angles to check if the stick is significantly tilted
        cos_angle_x = Ax / math.sqrt(Ax**2 + Ay**2 + Az**2)
        cos_angle_y = Ay / math.sqrt(Ax**2 + Ay**2 + Az**2)

        acc_magnitude = math.sqrt(Ax**2 + Ay**2 + Az**2) * g
        # Uncomment if you're having issues with your accelerometer values
        # print(f"Acc magnitude: {acc_magnitude} m/s^2, Angle X: {math.degrees(math.acos(cos_angle_x))}, Angle Y: {math.degrees(math.acos(cos_angle_y))}")

        if acc_magnitude > threshold_acceleration and (cos_angle_x < cos_threshold_angle or cos_angle_y < cos_threshold_angle):
            user_fell()

        # Check if the device is upright - if a user fell and they move their stick upright, we can assume theyre ok and that they can request an alert if not
        if fall_detected and abs(acc_magnitude - g) < upright_threshold:
            fall_detected = False
            speak("Normal stick usage detected. Cancelling alert. You can send an alert by saying 'I fell' or 'Help'.")
            if walk_timer.is_alive():
                walk_timer.cancel()

        sleep(0.5)

def user_fell():
    global fall_detected,fell_timer
    fall_detected = True

    # Start a timer to send an alert if no clear response
    fell_timer = threading.Timer(60, send_fall_alert)
    fell_timer.start()

    print("Fall detected!")
    speak("A fall was detected. Are you okay? Please say yes or no.") # we check for other responses, but yes or no is easier to understand

    # Listen for a response from the user
    response = listen_for_response()
    response_evaluation = evaluate_response(response)
    
    if response_evaluation:
        print("Timer and alert cancelled.")
        speak("Cancelling fall alert.")
        fell_timer.cancel()
    elif response_evaluation == False:
        send_alert("User has fallen and is injured.  Last known location is:") # we always send location data

def send_fall_alert():
    global fall_detected # Check to see if the user cancelled detected fall
    if fall_detected:
        send_alert("User has fallen and no response has been received in the allocated time.  Last known location: ")
        speak("Fall alert sent")

import speech_recognition as sr

def listen_for_voice_commands():
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()
    print("listen_for_voice_commands")
    while True:
        try:
            with microphone as source:
                recognizer.adjust_for_ambient_noise(source)
                audio = recognizer.listen(source, timeout=5.0, phrase_time_limit=10.0)  # 5 seconds to start speaking, 10 seconds for speaking
            
            # process speech with Google's speech recognition
            command = recognizer.recognize_google(audio)
            print("You said:", command)
            handle_voice_command(command, microphone, recognizer)

        except sr.UnknownValueError:
            print("Google Speech Recognition could not understand audio")
        except sr.RequestError as e:
            print("Could not request results; {0}".format(e))
        except sr.WaitTimeoutError:
            print("Listening timed out while waiting for phrase to start")
        except KeyboardInterrupt:
            print("Voice command listening terminated.")
            break

def handle_voice_command(command, microphone, recognizer):
    command = command.lower()  # Normalize the command to lowercase for easier comparison
    global ultrasonic_sensor_enabled, activate_blues_alerts, sos_sent
    print(f"Voice command: {command}")

    if "set home" in command "i'm home" in command "i'm at home" in command:
        set_home_location()
    elif "where am i" in command:
        speak(process_location())
    elif "help" in command:
        sos_sent = False
        activate_blues_alerts = True    # turn blues alerts back on in the case of an emergency
        sos_button_pressed()
    elif "i'm okay" in command or "im ok" in command:
        speak("Glad to hear that you're okay. Sending status message.")
        send_alert("User has stated that they are ok. Location:")
    elif "enable blues alerts" in command or "enable alerts" in command:
        settings['activate_blues_alerts'] = True
        save_settings(settings)
        activate_blues_alerts = True
        # initialize_blues_service()
        speak("Blues alerts enabled.")
    elif "disable blues alerts" in command or "disable alerts" in command:
        settings['activate_blues_alerts'] = False
        save_settings(settings)
        activate_blues_alerts = False
        # terminate_blues_service() # I commented this out so we can get weather updates
        speak("Blues alerts disabled.")
    elif "enable ultrasonic sensor" in command or "enable proximity sensor" in command or "enable distance sensor" in command:
        settings['ultrasonic_sensor_enabled'] = True
        save_settings(settings)
        ultrasonic_sensor_enabled = True
        speak("Ultrasonic sensor enabled.")
    elif "disable ultrasonic sensor" in command or "disable proximity sensor" in command or "disable distance sensor" in command or "disable distance" in command or "disable object sensor" in command or "disable distant sensor" in command:
        settings['ultrasonic_sensor_enabled'] = False
        save_settings(settings)
        ultrasonic_sensor_enabled = False
        print("Ultrasonic sensor disabled.")
        speak("Ultrasonic sensor disabled.")
    elif "extend walk by" in command:
        time_phrase = command.split("extend walk by")[-1].strip()
        minutes = parse_time_to_minutes(time_phrase)
        if minutes > 0:
            extend_walk(minutes)
        else:
            speak("Could not understand the duration. Please repeat the command with a number.")
    elif "start a" in command and "walk" in command:
        time_phrase = command.split("start a")[-1].split("walk")[0].strip()
        minutes = parse_time_to_minutes(time_phrase)
        if minutes > 0:
            start_timed_walk(minutes)
        else:
            speak("Could not understand the walk duration. Please specify a number of minutes.")
    elif "extend walk" in command:
        extend_walk()
    elif "start walk" in command:
        start_walk()
    elif "end walk" in command or "i am home" in command:
        end_walk()
    elif "i fell" in command:
        speak("User fell and requested an alert be sent.")
    elif "what time is it" in command:
        current_time = datetime.now().strftime('%H:%M')
        speak(f"The current time is {current_time}")
    elif "how far am i from home" in command:
        report_distance_from_home()
    elif "how fast am i going" in command:
        get_speed()
    elif "what's my elevation" in command:
        get_elevation()
    elif "send a message" in command or "send message" in command:
        prompt_message(microphone, recognizer)
    elif "send my location" in command:
        send_my_location()
    elif "what is the weather" in command:
        speak_weather(fetch_latest_weather_data())
    elif "what is the temperature" in command:
        get_temperature()
    elif "what is the chance of rain" in command:
        get_chance_of_rain()
    elif "what is the wind speed" in command:
        get_wind_speed()
    elif "what is the humidity" in command:
        get_humidity()
    elif "list voice commands" in command:
        list_voice_commands()
    elif 'butler' in command:
            command = command.split('butler', 1)[1].strip()  # Get the part after 'butler'
            if command:
                print(f"Command received: {command}")
                # Send the command to the MQTT broker
                publish.single(TOPIC, command, hostname=BROKER, port=PORT, auth={'username': USERNAME, 'password': PASSWORD})
                print("Command sent to the Flask server via MQTT.")
            else:
                print("No command detected after the trigger word.")

def parse_time_to_minutes(input_time):
    """
    Convert spoken time phrases into minutes.
    Handles phrases like '30 minute walk', '1 hour walk', 'an hour and a half walk'.
    """
    input_time = input_time.lower()
    # Regex patterns to capture numbers and time units
    number_pattern = r'\d+'
    hour_pattern = r'hour'
    minute_pattern = r'minute'
    
    # Check for minutes and hours in the input
    minutes = sum(map(int, re.findall(number_pattern, re.sub(minute_pattern, '', input_time))))
    hours = sum(map(int, re.findall(number_pattern, re.sub(hour_pattern, '', input_time))))
    
    # Special cases for "a" or "an" which typically mean one
    if 'hour' in input_time and ('an ' in input_time or ' a ' in input_time):
        hours += 1
    if 'minute' in input_time and ('a ' in input_time or ' an ' in input_time):
        minutes += 1
    
    # Calculate total minutes
    total_minutes = minutes + hours * 60
    
    # Additional logic to handle complex phrases like "an hour and a half"
    if 'half' in input_time and 'hour' in input_time:
        total_minutes += 30
    
    return total_minutes

def listen_for_response():
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()
    with microphone as source:
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)
    try:
        response = recognizer.recognize_google(audio).lower()
        return response
    except sr.UnknownValueError:
        print("Speech Recognition could not understand audio")
    except sr.RequestError as e:
        print(f"Could not request results from Google Speech Recognition service; {e}")
    return ""

def evaluate_response(response):
    affirmative_responses = ["yes", "i am ok", "i'm ok", "i am okay", "i'm okay", "all good", "no problem", "fine"]
    negative_responses = ["no", "i fell", "help", "i'm not okay", "not ok", "not okay", "i am hurt", "injured", "bad"]

    if any(word in response for word in affirmative_responses):
        print("User confirmed they are okay.")
        return True
    elif any(word in response for word in negative_responses):
        print("User indicated they are not okay or needs help.")
        return False
    else:
        print("Unrecognized response.")
        return None
    
def initialize_blues_service():
    global notecard_port
    if not notecard_port:
        try:
            serial_port = setup_serial_connection(PORT, BAUD_RATE)
            if serial_port is not None:
                notecard_port = setup_notecard(serial_port)
            else:
                print("Failed to set up the serial port connection.")
        except Exception as e:
            speak("Failed to connect to Blues services.")
            print(f"Error: {str(e)}")

def get_weather_data(lat, lon):
    print("get_weather_data")
    req = {
        "req": "web.get",
        "route": "weatherInfo",  # Replace with your for Proxy Route if it's different
        "name": "",  # Leave this empty - we will use placeholders in the route
        "body": {
            "lat": lat,
            "lon": lon
        },
        "content": "application/json"
    }
    response = notecard_port.Transaction(req)
    return response

# This is a nice way to visualize a bunch of different weather information
def print_weather_data(weather_data):
    print("Current Weather:")
    current = weather_data.get("current", {})
    print(f"  Temperature: {kelvin_to_fahrenheit(current.get('temp', 'N/A'))}C")
    print(f"  Feels Like: {current.get('feels_like', 'N/A')}C")
    print(f"  Humidity: {current.get('humidity', 'N/A')}%")
    print(f"  Pressure: {current.get('pressure', 'N/A')} hPa")
    print(f"  Wind Speed: {current.get('wind_speed', 'N/A')} m/s")
    print(f"  Wind Direction: {current.get('wind_deg', 'N/A')}")
    print(f"  UV Index: {current.get('uvi', 'N/A')}")
    print(f"  Cloudiness: {current.get('clouds', 'N/A')}%")
    print(f"  Visibility: {current.get('visibility', 'N/A')} meters")
    print(f"  Weather Description: {current.get('weather', [{}])[0].get('description', 'N/A')}")
    print(f"  Rain Volume (1h): {current.get('rain', {}).get('1h', 'N/A')} mm")
    print(f"  Snow Volume (1h): {current.get('snow', {}).get('1h', 'N/A')} mm")

    print("\nDaily Forecast:")
    daily = weather_data.get("daily", [])[0]  # Get today's forecast
    print(f"  Morning Temperature: {kelvin_to_fahrenheit(daily.get('temp', {}).get('morn', 'N/A'))}C")
    print(f"  Day Temperature: {kelvin_to_fahrenheit(daily.get('temp', {}).get('day', 'N/A'))}C")
    print(f"  Evening Temperature: {kelvin_to_fahrenheit(daily.get('temp', {}).get('eve', 'N/A'))}C")
    print(f"  Night Temperature: {kelvin_to_fahrenheit(daily.get('temp', {}).get('night', 'N/A'))}C")
    print(f"  Min Temperature: {kelvin_to_fahrenheit(daily.get('temp', {}).get('min', 'N/A'))}C")
    print(f"  Max Temperature: {kelvin_to_fahrenheit(daily.get('temp', {}).get('max', 'N/A'))}C")
    print(f"  Humidity: {daily.get('humidity', 'N/A')}%")
    print(f"  Pressure: {daily.get('pressure', 'N/A')} hPa")
    print(f"  Wind Speed: {daily.get('wind_speed', 'N/A')} m/s")
    print(f"  Wind Direction: {daily.get('wind_deg', 'N/A')}")
    print(f"  UV Index: {daily.get('uvi', 'N/A')}")
    print(f"  Cloudiness: {daily.get('clouds', 'N/A')}%")
    print(f"  Probability of Precipitation: {daily.get('pop', 'N/A') * 100}%")
    print(f"  Rain Volume: {daily.get('rain', 'N/A')} mm")
    print(f"  Snow Volume: {daily.get('snow', 'N/A')} mm")
    print(f"  Weather Description: {daily.get('weather', [{}])[0].get('description', 'N/A')}")

def terminate_blues_service():
    global notecard_port
    if notecard_port:
        notecard_port.close()
        notecard_port = None
        speak("Blues service terminated.")

def extend_walk(additional_minutes=30):
    global walk_extension, walk_start_time
    if walk_active:
        walk_extension += additional_minutes
        new_end_time = walk_start_time + timedelta(minutes=walk_extension)
        speak(f"Walk extended by {additional_minutes} minutes. New end time is {new_end_time.strftime('%H:%M')}.")
    else:
        speak("There is no active walk to extend.")

def start_walk():
    global walk_active, walk_start_time
    if not walk_active:
        walk_active = True
        walk_start_time = datetime.now()
        speak("Walk started. Have a good trip!")
    else:
        speak("A walk is already active.")
        
def end_walk():
    global walk_active
    if walk_active:
        walk_active = False
        speak("Walk has ended.")
    else:
        speak("No active walk to end.")

def check_geofence_status():
    global lat,lng
    for geofence in geofences:
        if check_geofence(lat, lng, geofence):
            speak(f"You are within the geofence of {geofence['name']}.")
            return
    speak("You are not within any known geofence.")

def get_speed():
    sog = gnss.get_sog()
    speak(f"Your speed is {sog} meters per second.")

def get_elevation():
    alt = gnss.get_alt()
    speak(f"Your elevation is {alt} meters.")

def send_my_location():
    lat = gnss.get_lat()
    lon = gnss.get_lon()
    message = f"My current location is latitude {lat}, longitude {lon}."
    send_message_over_blues(message)

def prompt_message(microphone, recognizer):
    speak("What's your message?")
    with microphone as source:
        audio = recognizer.listen(source, timeout=5.0, phrase_time_limit=10.0) 
        try:
            message = recognizer.recognize_google(audio)
            confirm_and_send_message(message, microphone, recognizer)
        except sr.UnknownValueError:
            speak("I didn't catch that. Please say your message again.")
            prompt_message(microphone, recognizer)

def confirm_and_send_message(message, microphone, recognizer):
    speak(f"You said: {message}. Would you like to send it?")
    audio = recognizer.listen(source, timeout=5.0, phrase_time_limit=10.0) 
    try:
        # Repurpose evaluate_response so the user can say more than just yes or no
        response = evaluate_response(recognizer.recognize_google(audio).lower())
        
        # if "yes" in response:
        if response:
            send_message_over_blues(message)
            speak("Message sent.")
        # elif "no" in response:
        elif response == False:
            speak("Would you like to start over?")
            handle_start_over(microphone, recognizer)
        else:
            speak("Please say yes or no.")
            confirm_and_send_message(message, microphone, recognizer)
    except sr.UnknownValueError:
        speak("Please say yes or no.")
        confirm_and_send_message(message, microphone, recognizer)

def handle_start_over(microphone, recognizer):
    audio = recognizer.listen(source, timeout=5.0, phrase_time_limit=10.0) 
    try:
        response = recognizer.recognize_google(audio).lower()
        if "yes" in response:
            prompt_message(microphone, recognizer)
        elif "no" in response:
            speak("Okay, not sending the message.")
    except sr.UnknownValueError:
        speak("Please say yes or no.")
        handle_start_over(microphone, recognizer)

def send_message_over_blues(message):
    if activate_blues_alerts:
        req = {"req": "note.add", "body": {"text": message}}
        notecard_port.Transaction(req)

def list_voice_commands():
    commands = [
        "Set home.",
        "Where am I?",
        "Help or SOS for emergencies",
        "Extend walk by [duration].",
        "Start a [duration] walk.",
        "Extend walk.",
        "Start walk.",
        "End the walk.",
        "What time is it?",
        "How far am I from home?",
        "How fast am I going?",
        "What's my elevation?",
        "Send a message.",
        "Send my location.",
        "Disable distance sensor.",
        "Enable distance sensor.",
        "What is the weather?",
        "What is the temperature?",
        "What is the chance of rain?",
        "What is the wind speed?",
        "What is the humidity?",
        "List voice commands."
    ]
    for command in commands:
        speak(command)
        sleep(1)


def report_distance_from_home():
    if not home_set:
        speak("Home location is not set.")
        return
    global lat,lng
    current_position = (lat, lng)
    home_position = (home_location["lat"], home_location["lng"])
    distance = geodesic(current_position, home_position).meters
    speak(f"You are approximately {distance:.2f} meters from home.")

def setup_notecard(serial_port):
    card = notecard.OpenSerial(serial_port)
    req = {"req": "hub.set", "product": PRODUCT_UID, "mode": "continuous"}
    rsp = card.Transaction(req)
    print(f"Setup response from Notecard: {rsp}")
    return card

def setup_serial_connection(port, baud_rate):
    try:
        return serial.Serial(port, baud_rate)
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        return None

def process_location():
    try:
        for geofence in geofences:
            if haversine_distance(lat, lng, geofence["center"]["lat"], geofence["center"]["lng"]) <= geofence["radius"]:
                return f"You are in {geofence['name']}."
        return f"Your coordinates are approximately {latitude:.4f}, {longitude:.4f}."
        
    except Exception as e:
        print(f"Error in process_location: {e}")
        sleep(1)

def check_weather():
    global all_weather_updates

    while True:
        try:
            print("checking weather")
            weather_data = fetch_latest_weather_data()
            if weather_data is not None:
                if all_weather_updates:
                    speak_weather(weather_data)
                else:
                    check_for_bad_weather(weather_data)
            sleep(900)  # Check weather every 15 minutes

        except Exception as e:
            print(f"Error in check_weather: {e}")
            sleep(90)  # Try again sooner if we error out


def check_for_bad_weather(weather_data):
    current = weather_data.get("current", {})
    rain = current.get("rain", {}).get('1h', 0)
    snow = current.get("snow", {}).get('1h', 0)
    wind_speed = current.get("wind_speed", 0)

    if rain > 40:
        speak("Chance of rain is {rain} within the hour")
    if snow > 40:
        speak("Chance of snow is {snow} within the hour")
    if wind_speed > 10:
        speak("There will be wind speeds of {wind_speed} within the hour")

def kelvin_to_fahrenheit(kelvin):
    return round((kelvin - 273.15) * 9/5 + 32, 2)

def speak_weather(weather_data):
    current = weather_data.get("current", {})
    temp_f = kelvin_to_fahrenheit(current.get("temp", "N/A"))
    weather_description = current.get("weather", [{}])[0].get("description", "N/A")

    speak(f"The current weather is {weather_description} with a temperature of {temp_f}F.")

def get_temperature():
    weather_data = fetch_latest_weather_data()
    temp_f = kelvin_to_fahrenheit(weather_data.get("current", {}).get("temp", "N/A"))
    speak(f"The current temperature is {temp_f}F.")

def get_chance_of_rain():
    weather_data = fetch_latest_weather_data()
    daily = weather_data.get("daily", [])[0]
    pop = daily.get("pop", "N/A")
    speak(f"The chance of rain is {pop * 100}%.")

def get_wind_speed():
    weather_data = fetch_latest_weather_data()
    wind_speed = weather_data.get("current", {}).get("wind_speed", "N/A")
    speak(f"The current wind speed is {wind_speed} meters per second.")

def get_humidity():
    weather_data = fetch_latest_weather_data()
    humidity = weather_data.get("current", {}).get("humidity", "N/A")
    speak(f"The current humidity is {humidity}%.")

def fetch_latest_weather_data():
    # print("fetch latest weather")
    try:
        print("lat long found for fetch weather")
        global lat,lng
        weather_response = get_weather_data(lat, lng) 
        print("weather response found")
        if weather_response.get("result") == 200:
            print("Good weather response received")
            return weather_response.get("body", {})
        else:
            print("Failed to fetch weather data")
            print("Response received:", weather_response)
            return {}
    except Exception as e:
        print(f"Error in fetch_latest_weather_data: {e}")
        sleep(1)
        return {}

def handle_ultrasonic_sensors():
    global tone, ultrasonic_sensor_up, ultrasonic_sensor_down, ultrasonic_sensor_enabled
        
    buzz_duration = .5
    max_distance_top = 100
    max_distance_bottom = 50    # make the bottom one have a lower range so we arent triggering constantly
    min_distance = 1
    is_up_sensor = True  # Start with the 'up' sensor
    while True:
        if ultrasonic_sensor_enabled:
            try:
                # Choose the current sensor based on the flag
                current_sensor = ultrasonic_sensor_up if is_up_sensor else ultrasonic_sensor_down
                max_distance = max_distance_top if is_up_sensor else max_distance_bottom
                distance = current_sensor.distance_cm()
                
                if LOG_ULTRASONIC_VALUES:
                    print(f"{'Up' if is_up_sensor else 'Down'} sensor distance: {distance} cm")

                if distance is None or distance == 0:
                    sleep(1)  # Adjust sleep time as needed
                    is_up_sensor = not is_up_sensor  # Switch to the other sensor
                    continue

                if distance > max_distance:
                    tick_threshold = 10
                elif distance <= min_distance:
                    tick_threshold = 1
                else:
                    tick_threshold = int(1 + 9 * (distance - min_distance) / (max_distance - min_distance))

                pitch = calculate_pitch(distance, is_up_sensor, min_distance, max_distance)
                
                # Using this setup instead of the buzzer function works way better with the threading - don't switch to the other way or it'll lock
                tone.freq(pitch)
                tone.on()
                sleep(buzz_duration)
                tone.off()
                
                sleep(1)
                is_up_sensor = not is_up_sensor  # Switch to the other sensor after handling the current one
            except Exception as e:
                print(f"Error in ultrasonic sensor thread: {e}")
                sleep(1)
        else:
            sleep(10)    


def manage_walk_thread():
    while True:
        manage_walk()
        time.sleep(15)

def main():
    try:
        initialize_blues_service()

        threading.Thread(target=listen_for_voice_commands).start()
        threading.Thread(target=gps_monitor).start()
        threading.Thread(target=handle_ultrasonic_sensors).start()
        threading.Thread(target=manage_walk_thread).start()
        threading.Thread(target=button_watch).start()   # You can comment this thread out - most users will prefer voice commands
        threading.Thread(target=check_weather).start()
        threading.Thread(target=detect_fall).start()

    except KeyboardInterrupt:
        logging.info("Program terminated by user.")
    except Exception as e:
        logging.error(f"Unhandled exception: {e}")
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()