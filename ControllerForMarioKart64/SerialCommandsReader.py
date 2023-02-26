import serial
from pynput.keyboard import Key, Controller
import time

keyboard = Controller()

serial_port = serial.Serial()
serial_port.baudrate = 115200
serial_port.port = 'COM9'
serial_port.timeout = 1
error = False

# Gives you time to open the desired target for the inputs
time.sleep(5) 

# Open serial port communicaiton
try:
    serial_port.open()
except:
    error = True

# Check for error presence in communication
if not serial_port.is_open or error:
    print("Something went wrong!")
    exit()
    
while 1:
    try:
        # In case some data is sent from serial port, store it in key
        if serial_port.in_waiting!=0:
            key = serial_port.read().decode('ascii')#.strip('\n').strip('\r')
             # If pause command is sent, press it for 0.1 s
            if key == "p" :
                keyboard.press(key)
                time.sleep(0.1)
                keyboard.release(key)
            # If 0 is passed, do nothing   
            elif key == "0":
                pass
            # If soft steer left command is passed, steering button is held pressed for shorter time (together with acceleration button)
            elif key == "l":
                keyboard.press('z') 
                keyboard.press('f')
                time.sleep(0.01)     
                keyboard.release('f')
             # If soft steer right command is passed, steering button is held pressed for shorter time(together with acceleration button)
            elif key == "k":
                keyboard.press('z') 
                keyboard.press('h')
                time.sleep(0.01)     
                keyboard.release('h')
            # All other commands are performed keeping buttons pressed for 0.03 seconds
            else:
                keyboard.press('z') 
                keyboard.press(key)
                time.sleep(0.03)     
                keyboard.release(key)
    except:
        pass










