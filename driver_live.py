import numpy as np
import pandas as pd
from KLT_Tracker import KltTracker
from xcorr import CrossCorr
from threading import Thread
import cv2
import math, time
import xlsxwriter
import pickle
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

index = 1
j = 2
alt = 0

while alt < 60 or alt == 0:

    if drone.alt is not None:
        alt = drone.alt
    print('Current Altitude',alt)
    #worksheet.write('C'+str(j), alt)
    time.sleep(1)

    if drone.head is not None:
        head = drone.head
    print('Current Heading:', head)
    # worksheet.write('D'+str(j), head)
    time.sleep(1)

    if drone.lat is not None:
        lat = drone.lat
    print('Current Latitude:',lat)
    # worksheet.write('A'+str(j), lat)
    time.sleep(1)

    if drone.lon is not None:
        lon = drone.lon
    print('Current Longitude:',lon)
    # worksheet.write('B'+str(j), lon)
    time.sleep(1)  

    #index = index + 1
    j = j+1
# #workbook.close()

log.write(f'{datetime.now()}        drone connected {drone.drone_location}\n')
# model_gps_pxl = pickle.load(open('rev_finalized_model.pkl', 'rb'))

######################### Parameters to be used ################################
frame_skips = 30
frame_skips_from_start = 0
fps = 30
total_frames_to_process = 100
klt_error_meters = 5

compression_ratio = 0.1
#reference_res = 0.05 #Ravi Block
reference_res = 0.0575  #LUMS_Video
#reference_res = 0.0928571428571429 #LUMS Webcam
#reference_res = 0.092 ##For Fortress

#global_res = 0.0501851851851852  ##Ravi Block 
global_res = 0.1031941031941032   ##LUMS
#global_res = 0.0353846153846154  ##For LUMSU by Turyal
#global_res = 0.06715328467 ## For Fortress 

#heading_error = 0 ##For Fortress
#heading_error = 0   ##For Ravi Block
heading_error = 0 #For LUMS_Video
#heading_error = 5  #For LUMSU Video by Turyal
meters_per_pixels = reference_res / compression_ratio
leeway = 10

# #heading_data = pd.read_excel('data/ravi_first.xlsx') 
# heading_data = pd.read_excel('/home/odroid/lucas-kanade-tracker-master/data/LUMS/lums1.xlsx')
# heading_data = pd.DataFrame(heading_data, columns=[' compass_heading(degrees)'])
# heading_data = heading_data[0:-1:int((frame_skips / fps) * 10)]
# head = head[0:-1:int((frame_skips / fps) * 10)]
#############################################
# defining a helper class for implementing multi-threaded processing 
class WebcamStream :
    def __init__(self, stream_id = 0):
        self.stream_id = stream_id   # default is 0 for primary camera 
        
        # opening video capture stream 
        self.vcap = cv2.VideoCapture(self.stream_id)
        if self.vcap.isOpened() is False :
            print("Error accessing webcam stream.")
            exit(0)
        fps_input_stream = int(self.vcap.get(5))
        print("FPS of webcam input stream: {}".format(fps_input_stream))
            
        # reading a single frame from vcap stream for initializing 
        self.grabbed , self.frame = self.vcap.read()
        if self.grabbed is False :
            print('No more frames to read')
            self.frame = None
            exit(0)

        # self.stopped is set to False when frames are being read from self.vcap stream 
        self.stopped = True 

        # reference to the thread for reading next available frame from input stream 
        self.t = Thread(target=self.update, args=())
        self.t.daemon = True # daemon threads keep running in the background while the program is executing 
        
    # method for starting the thread for grabbing next available frame in input stream 
    def start(self):
        self.stopped = False
        self.t.start() 

    # method for reading next frame 
    def update(self):
        while True :
            if self.stopped is True :
                break
            self.grabbed , self.frame = self.vcap.read()
            if self.grabbed is False :
                print('No more frames to read')
                self.frame = None
                self.stopped = True
                break 
        self.vcap.release()

    # method for returning latest read frame 
    def read(self):
        return self.frame

    # method called to stop reading frames 
    def stop(self):
        self.stopped = True 
#############################################

#cap = cv2.VideoCapture(r'data/ravi_first.MP4')
#cap = cv2.VideoCapture(r'data/LUMS/lums.MP4')
#cap = cv2.VideoCapture(6) 
webcam_stream = WebcamStream(stream_id = 0) #  stream_id = 0 is for primary camera 
webcam_stream.start()

#oldPosition = [3071.69 * compression_ratio, 1770.14 * compression_ratio] ## For Ravi Block
oldPosition = [1856 * compression_ratio, 4288 * compression_ratio] ##LUMS
#oldPosition = [4500 * compression_ratio, 7640 * compression_ratio] #For LUMS by Turyal
#oldPosition = [3000 * compression_ratio, 4110 * compression_ratio] #For Fortress

#xcorr_obj = CrossCorr('/home/odroid/lucas-kanade-tracker-master/data/ravi_block_150m.tif', compression_ratio, reference_res, global_res, oldPosition)
xcorr_obj = CrossCorr('/home/odroid/lucas-kanade-tracker-master/data/LUMS/lums.tif', compression_ratio, reference_res, global_res, oldPosition)
#xcorr_obj = CrossCorr('/home/odroid/lucas-kanade-tracker-master/data/Fortress/fortress_pc.tif', compression_ratio, reference_res, global_res, oldPosition)

#ret, frame = cap.read()
frame = webcam_stream.read() 
# for i in range(frame_skips_from_start):
    #ret, frame = cap.read()
frame = webcam_stream.read()     
frame = cv2.resize(frame, (0, 0), fx=compression_ratio, fy=compression_ratio)
frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
obj = KltTracker(frame)

heading_index = 0
# xcorr_obj.find_cross_corr(frame, heading_data.iloc[heading_index, 0], heading_error, 5, leeway)
heading_index += 1

acc_x = 0
acc_y = 0

all_x = np.array([], dtype=int)
all_y = np.array([], dtype=int)

###############
all_alt = np.array([], dtype=float)
all_head = np.array([], dtype=float)
all_lat = np.array([], dtype=float)
all_lon = np.array([], dtype=float)
###############


frame_no = 1
number = 1
tim = time.time()
print("Entering Loop")
j=2
i=1 ##For Saving the input Frames
num_frames_processed = 0
#start = time.time()
while True:    
    try:
        
        print("\n**** Frame No. ", i, "****")

        if drone.alt is not None:
            alt = drone.alt
            head = drone.head
            lat = drone.lat
            lon = drone.lon

        print('\nCurrent Altitude',alt)
        # worksheet.write('C'+str(j), alt)
            
        print('Current Heading:', head)
        # worksheet.write('D'+str(j), head)
            
        print('Current Latitude:',lat)
        # worksheet.write('A'+str(j), lat)
            
        print('Current Longitude:',lon)
        # worksheet.write('B'+str(j), lon)
        # time.sleep(1)

        # if drone.head is not None:  ##Just to check
        #     head = drone.head
        # print('Current Heading:', head)

        drone_loc = drone.drone_location 
        #for i in range(frame_skips):
            #ret, frame = cap.read()
        frame = webcam_stream.read()
        # adding a delay for simulating time taken for processing a frame 
        # delay = 0.005 # delay value in seconds. so, delay=1 is equivalent to 1 second 
        # time.sleep(delay) 
        num_frames_processed += 1 
        #cv2.imshow('frame' , frame)
        # print("Frame No. ", j)

        if frame is None:
            break
        ############### #Saving Original Frames
        cv2.imwrite(r'/home/odroid/lucas-kanade-tracker-master/InputFrames/original/Frame_'+ str(j) + '.png',frame) ##Saving the Input Frames
        ###############
        
        frame = cv2.resize(frame, (0, 0), fx=compression_ratio, fy=compression_ratio)  
        t_frame = frame
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        ############### #Saving the compressed gray frames
        cv2.imwrite(r'/home/odroid/lucas-kanade-tracker-master/InputFrames/resized/Frame_'+ str(j) + '.png',frame) ##Saving the Input Frames
        i+=1
        ####################################
        # print("frame", len(frame))
        # if int(frame_no) % 2 == 0:
        obj.update_feature_to_track(frame)
            # frame_no = 1

        disp_x_klt, disp_y_klt = obj.step(frame)
        acc_x = acc_x + disp_x_klt
        acc_y = acc_y + disp_y_klt

        # Calculate the displacement in meters returned by the KLT
        displace_meters_klt = math.sqrt(disp_x_klt ** 2 + disp_y_klt ** 2)
        displace_meters_klt *= (reference_res / compression_ratio)
        #print("Displacement in Meters:", displace_meters_klt) 
        # Get the X, Y position on the global Map
        a = time.time()
        #x, y = xcorr_obj.find_cross_corr(frame, heading_data.iloc[heading_index, 0], heading_error, displace_meters_klt, leeway)
        # head = head[0:-1:int((frame_skips / fps) * 10)]
        x, y = xcorr_obj.find_cross_corr(frame, head, heading_error, displace_meters_klt, leeway)

        # print("HEADDDDDIINNGGGGGG:" , head)
        print("Localised at: ",x,y, "in time", 1/(time.time()-a))
        # worksheet.write('E'+str(j), x)
        # worksheet.write('F'+str(j), y)
        print("\n#####################################################")
        # localisation object from the frame onto global map
        # xcorr_obj.object_localisation(x, y, t_frame, heading_data.iloc[heading_index, 0], heading_error)

        heading_index += 1
        log.write(f"{datetime.now()}        {number}: {time.time()-tim}s : x = {x} , y = {y}\n")
        log.write(f"{datetime.now()}            drone_locations: {drone_loc}\n")
        all_x = np.append(all_x, x)
        all_y = np.append(all_y, y)

        # #######################
        all_alt = np.append(all_alt, alt)
        all_head = np.append(all_head, head)
        all_lat = np.append(all_lat, lat)
        all_lon = np.append(all_lon, lon)
        # #######################
        frame_no = frame_no + 1
        number = number + 1
        j = j + 1
    
        ########################### Creating an  Excel File and saving values in that to print later ######################################
        workbook = xlsxwriter.Workbook('/home/odroid/lucas-kanade-tracker-master/data/Flight_Data.xlsx')
        worksheet = workbook.add_worksheet()
        worksheet.write('A1', 'latitude')
        worksheet.write('B1', 'longitude')
        worksheet.write('C1', 'altitude')
        worksheet.write('D1', 'heading')
        worksheet.write('E1', 'estimated_x')
        worksheet.write('F1', 'estimated_y')
        #####################################################
        for x in range(len(all_alt)):   ##
            m = x+2
            worksheet.write('A'+str(m), all_lat[x])
            worksheet.write('B'+str(m), all_lon[x])
            worksheet.write('C'+str(m), all_alt[x])
            worksheet.write('D'+str(m), all_head[x])
            worksheet.write('E'+str(m), all_x[x])
            worksheet.write('F'+str(m), all_y[x])
        workbook.close()
        # ############################################

    except Exception as e:
            log.write(f"{datetime.now()}        Exception : {e} : at {number}: {time.time()-tim}s \n")
            log.close()

    # if number == total_frames_to_process:
    #     break

    if alt < 60:
        print('Exiting Loop!!!!')
        break

# workbook.close()
log.close()

fps = total_frames_to_process / (time.time()-tim)
print("\nProcessed", total_frames_to_process," frames in",time.time()-tim, "seconds")
print("\nFrames per Seconds (FPS) = ", fps)

# xcorr_obj.plot_points_on_map(all_x, all_y, 0)
############################################

#################### Reading the Recently Stored Excel File #####################
# loading pixel to lat-long model
regg = pickle.load(open('/home/odroid/lucas-kanade-tracker-master/GPS_Pixel_Models/LUMS_gps_pxl.pkl', 'rb'))
#regg = pickle.load(open('/home/odroid/lucas-kanade-tracker-master/GPS_Pixel_Models/FT_gps_pxl.pkl', 'rb'))

print("Reading Heading Data")
data = pd.read_excel('/home/odroid/lucas-kanade-tracker-master/data/Flight_Data.xlsx') 
#heading_data = pd.read_excel('data/LUMS/lums1.xlsx')
heading_data = pd.DataFrame(data, columns=['heading'])
# heading_data = heading_data[0:-1:int((frame_skips / fps) * 10)]
print("Heading Data Read")

print("Reading  Latitude from Excel File")
#lat_data = pd.read_excel('data/LUMS/lums1.xlsx')
lat_data = pd.DataFrame(data, columns=['latitude'])
# lat_data = lat_data[0:-1:int((frame_skips / fps) * 10)]

print("Reading  Longitude from Excel File")
long_data = pd.DataFrame(data, columns=['longitude'])
# long_data = long_data[0:-1:int((frame_skips / fps) * 10)]
print("Lat, Long read")
# print('Lat Data:', lat_data)
# print('Long Data:', long_data)
################## Converting GPS to Pixels #######################
all_gps_x = np.array([], dtype=float)
all_gps_y = np.array([], dtype=float)

long_data = long_data.to_numpy()
lat_data = lat_data.to_numpy()
long_lat_data = np.concatenate((long_data, lat_data), axis=1)
#print(regg.predict(long_lat_data))

result = regg.predict(long_lat_data)
for pixel in result:
    all_gps_x = np.append(all_gps_x, pixel[0]*compression_ratio)
    all_gps_y = np.append(all_gps_y, pixel[1]*compression_ratio)
#print('Original X,Y:', all_gps_x, all_gps_y)
###################################################################
################################################################################
#xcorr_obj.plot_points_on_map(all_x, all_y, head, 'plot-'+stamp)
xcorr_obj.plot_points_on_mapp(all_x, all_y, all_gps_x, all_gps_y, 0) #Plotting Both 
xcorr_obj.plot_points_on_map(all_x, all_y, 0)
# xcorr_obj.plot_points_on_map(all_x, all_y, 0)

# print(all_gps_x)
# print(all_gps_y)
# print(all_x)
# print(all_y)