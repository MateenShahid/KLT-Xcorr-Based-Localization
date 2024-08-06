import numpy as np
import pandas as pd
import cv2 as cv
import queue
import threading
import math, time
import xlsxwriter
from mav_Comm import Drone
from datetime import datetime

stamp = '-'.join(str(datetime.now()).split())

file_name = '/home/odroid/lucas-kanade-tracker-master/logs/log-visp-'+stamp+'.txt'
log = open(file_name, 'w+')
log.write('\n####################################\n')
log.write(f'{datetime.now()}\n\n')

connection_string = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0"
drone = Drone(connection_string=connection_string,stamp = stamp)
ret = drone.connect_to_vehicle()
time.sleep(1)
# drone.set_drone_home_location()
print('Connection Done')

log.write(f'{datetime.now()}        drone connected {drone.drone_location}\n')
# model_gps_pxl = pickle.load(open('rev_finalized_model.pkl', 'rb'))
   
alt_thresh = 50  ## Set the threshold for Altitude

# ########################## Loop Untill Threshold is acheived ######################
alt = 0
while alt < alt_thresh or alt == 0:

    print('#####################')
    if drone.alt is not None:
        alt = drone.alt
    print('Current Altitude',alt)
    time.sleep(1)

    if drone.head is not None:
        head = drone.head
    print('Current Heading:', head)

    if drone.lat is not None:
        lat = drone.lat
    print('Current Latitude:',lat)

    if drone.lon is not None:
        lon = drone.lon
    print('Current Longitude:',lon)

    print('Waiting to Reach Altitude')
    time.sleep(1)
    if alt > alt_thresh:
        print('Altitude Achieved!!!!')
        break
#####################################################################################

cap = cv.VideoCapture(0)

# Set the frame rate
fps = 20
cap.set(cv.CAP_PROP_FPS, fps)

# Set the video width and height
width = 640
height = 360
cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)

# Create a queue to store frames
frame_queue = queue.Queue()

# Define a function to continuously read frames from the camera and put them in the queue
def capture_frames():
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        frame_queue.put(frame)

# Start a separate thread to capture frames
thread = threading.Thread(target=capture_frames)
thread.start()

# Define the codec and create VideoWriter object
fourcc = cv.VideoWriter_fourcc(*'XVID')
out = cv.VideoWriter('/home/odroid/Desktop/Lums_Web_Vid.mp4', fourcc, fps, (width,  height))

all_alt = np.array([], dtype=float)
all_head = np.array([], dtype=float)
all_lat = np.array([], dtype=float)
all_lon = np.array([], dtype=float)

# index = 1
j = 1
# alt = 0

max_frames = 2400 #900 frames means 30 sec video

############################## Main Loop #########################################
print('Entering Loop')
while alt > alt_thresh and max_frames > 0 or alt == 0:
    # while cap.isOpened():
    frame = frame_queue.get()
    out.write(frame)
    # cv.imshow('frame', frame)
    cv.imwrite('/home/odroid/lucas-kanade-tracker-master/Saved_Frames/Frame# ' + str(j) +'.png', frame)
    
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
    max_frames -= 1

    if drone.alt is not None:
        alt = drone.alt
        head = drone.head
        lat = drone.lat
        lon = drone.lon
    print('###############################')
    print('Frame No. ' + str(j))
    print('Current Altitude',alt) 
    print('Current Heading:', head)
    print('Current Latitude:',lat)
    print('Current Longitude:',lon)
    print('###############################')
    
    all_alt = np.append(all_alt, alt)
    all_head = np.append(all_head, head)
    all_lat = np.append(all_lat, lat)
    all_lon = np.append(all_lon, lon)
    
    #index = index + 1
    j = j+1    

    if alt < alt_thresh or max_frames == 0:
        print('\nExiting Loop!!!!!')
        break

print('\nExited Loop!')
# Release everything if job is finished
cap.release()
out.release()
cv.destroyAllWindows()

########################### Creating an  Excel File and Storing Parameters ######################################
workbook = xlsxwriter.Workbook('/home/odroid/lucas-kanade-tracker-master/data/Post_Vid_Data.xlsx')
worksheet = workbook.add_worksheet()
worksheet.write('A1', 'latitude')
worksheet.write('B1', 'longitude')
worksheet.write('C1', 'altitude')
worksheet.write('D1', 'heading')
for x in range(len(all_alt)):   ##
    m = x+2
    worksheet.write('A'+str(m), all_lat[x])
    worksheet.write('B'+str(m), all_lon[x])
    worksheet.write('C'+str(m), all_alt[x])
    worksheet.write('D'+str(m), all_head[x])
workbook.close()
##########################################################################

log.write(f'{datetime.now()}        drone connected {drone.drone_location}')
