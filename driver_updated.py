import numpy as np
import pandas as pd
from KLT_Tracker import KltTracker
from xcorr import CrossCorr
import cv2
import math, time
import pickle
import sklearn
#from mav_Comm import Drone


# Parameters to be used
frame_skips = 10
frame_skips_from_start = 0
fps = 30
total_frames_to_process = 75
klt_error_meters = 0
compression_ratio = 0.1
# reference_res = 0.05 #Ravi Block
#reference_res = 0.0575  #LUMS HD Video 
#reference_res = 0.0928571428571429 #LUMS webcam at 65m
# reference_res = 0.09325301205 #LUMS Webcam at 61m/lums22-f2
reference_res = 0.06 #0.05081967213 #LUMS Webcam at 42m 
# reference_res = 0.09725301205 #LUMS Webcam at 51m/lums22-f1

# global_res = 0.0501851851851852  ##Ravi Block 
global_res = 0.1031941031941032   ##LUMS

heading_error = 42   ##For Ravi Block
# heading_error = 19 #For LUMS_Video
# heading_error = 10 ##For LUMS march 2 61m
#heading_error = 5  ## For LUMSU Video by Turyal
meters_per_pixels = reference_res / compression_ratio
leeway = 10

# loading pixel to lat-long model
#reg = pickle.load(open('finalized_model.pkl', 'rb'))
# regg = pickle.load(open('GPS_Pixel_Models/ravi_gps_pxl.pkl', 'rb'))
regg = pickle.load(open('GPS_Pixel_Models/LUMS_gps_pxl.pkl', 'rb'))

print("Reading Heading Data")
# heading_data = pd.read_excel('data/Ravi/ravi_first.xlsx') 
data = pd.read_excel('data/LUMS/lums-march-2/f4-42bm.xlsx') 
#heading_data = pd.read_excel('data/LUMS/lums1.xlsx')

# heading_data = pd.DataFrame(heading_data, columns=[' compass_heading(degrees)'])
heading_data = pd.DataFrame(data, columns=['heading'])
# heading_data = heading_data[0:-1:int((frame_skips / fps) * 10)]
print("Heading Data Read", heading_data)

########################
# data = pd.read_excel('data/Ravi/ravi_first.xlsx')
# data = pd.read_excel('data/lums22/lums22-f1.xlsx')
#data = pd.read_excel('data/LUMS/lums1.xlsx')

print("Reading  Latitude from Excel File")
lat_data = pd.DataFrame(data, columns=['latitude'])
lat_data = lat_data[0:-1:int((frame_skips / fps) * 10)]
print("Reading  Longitude from Excel File")
long_data = pd.DataFrame(data, columns=['longitude'])
long_data = long_data[0:-1:int((frame_skips / fps) * 10)]

print("Lat, Long read")
########################

# cap = cv2.VideoCapture(r'data/Ravi/ravi_first.MP4')
cap = cv2.VideoCapture(r'Webcam_Videos/lums-march-2/f4-42m.MP4')
# cap = cv2.VideoCapture(r'data/LUMS/lums.MP4')
# cap = cv2.VideoCapture(6)

# oldPosition = [3071.69 * compression_ratio, 1770.14 * compression_ratio] #Ravi Block
#oldPosition = [1856 * compression_ratio, 4288 * compression_ratio] ##LUMS webcam
# oldPosition = [1790 * compression_ratio, 4840 * compression_ratio] ##LUMS webcam_Videos
# oldPosition = [2420 * compression_ratio, 3950 * compression_ratio] ##LUMS webcam_2b_Video
oldPosition = [1835 * compression_ratio, 4750 * compression_ratio] ##LUMS webcam_42m march-2

# xcorr_obj = CrossCorr(r'data/Ravi/ravi_block_150m.tif', compression_ratio, reference_res, global_res, oldPosition) ##Ravi Block
xcorr_obj = CrossCorr(r'data/LUMS/lums.tif', compression_ratio, reference_res, global_res, oldPosition) ##LUMS

# Configurations END

ret, frame = cap.read()
for i in range(frame_skips_from_start):
    ret, frame = cap.read()
frame = cv2.resize(frame, (0, 0), fx=compression_ratio, fy=compression_ratio)
frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

obj = KltTracker(frame)

heading_index = 0
# xcorr_obj.find_cross_corr(frame, heading_data.iloc[heading_index, 0], heading_error, 5, leeway)
heading_index += 1

frame_no = 1
acc_x = 0
acc_y = 0

all_x = np.array([], dtype=int)
all_y = np.array([], dtype=int)

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

number = 1
tim = time.time()
print("Entering loop")
j=1 ##For saving the input frames
while True:
    for i in range(frame_skips):
        ret, frame = cap.read()

    print("\nFrame No. ", j)        
    if frame is None or not ret:
        break
    ############### #Saving Original Frames
    cv2.imwrite(r'InputFrames/original/Frame_'+ str(j) + '.png',frame) ##Saving the Input Frames
    ###############
    frame = cv2.resize(frame, (0, 0), fx=compression_ratio, fy=compression_ratio)
    t_frame = frame
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #################################
    cv2.imwrite(r'InputFrames/resized/Frame_'+ str(j) + '.png',frame) ##Saving the Input Frames
    j+=1
    ####################################

    # print("frame", len(frame))
    if int(frame_no) % 2 == 0:
        obj.update_feature_to_track(frame)
        frame_no = 1

    disp_x_klt, disp_y_klt = obj.step(frame)
    acc_x = acc_x + disp_x_klt
    acc_y = acc_y + disp_y_klt
   
    # Calculate the displacement in meters returned by the KLT
    displace_meters_klt = math.sqrt(disp_x_klt ** 2 + disp_y_klt ** 2)
    displace_meters_klt *= (reference_res / compression_ratio)

    # Get the X, Y position on the global Map
    x, y = xcorr_obj.find_cross_corr(frame, heading_data.iloc[heading_index, 0], heading_error, displace_meters_klt, leeway)
    print("Localised at (X, Y): ",x,y)

    # print("Original  GPS Value:",long_lat_data[heading_index])

    # print("Original X/Y Values:", all_gps_x[heading_index],all_gps_y[heading_index])

    #print("Original Coordinates:", lat_data,long_data)
    # localisation object from the frame onto global map
    # xcorr_obj.object_localisation(x, y, t_frame, heading_data.iloc[heading_index, 0], heading_error)
    
    ##################################################
    ## Predicting Lat long from Pixels.
    # pred_long, pred_lat = reg.predict([[acc_x, acc_y]])[0]
    # pred_long, pred_lat = reg.predict([[x, y]])[0]
    # print('Predicted Lat/Long (From Pixels): ', pred_lat, pred_long)
    
    #print('Original Lat/Long (From GPS): ', lat_data, long_data)
    ## Prediciting (x,y) pixels from lat long. 
    # gps_x, gps_y = regg.predict([[long_data, lat_data]])[0]
    # print('Original X,Y:', gps_x, gps_y)
    
    #pred_y, pred_x = regg.predict([[long_data, lat_data]])[0]
    #print('Predicted X,Y:', pred_x, pred_y)
    ##################################################

    heading_index += 1
    all_x = np.append(all_x, x)
    all_y = np.append(all_y, y)
    ###########################
    # all_gps_x = np.append(all_gps_x, gps_x)
    # all_gps_y = np.append(all_gps_y, gps_y)
    ###########################
    frame_no = frame_no + 1
    number = number + 1
    if number == total_frames_to_process:
        break

fps = total_frames_to_process / (time.time()-tim)
print("\nProcessed", j,"in",time.time()-tim, "seconds")
print("\nFrames per Seconds (FPS) = ", fps)

# xcorr_obj.plot_points_on_map(all_x, all_y, 0) #Plotting the pixels on Orthomosaic

xcorr_obj.plot_points_on_mapp(all_x, all_y, all_gps_x, all_gps_y, 0) #Plotting Both 

#xcorr_obj.plot_points_on_mapp(all_gps_x, all_gps_y, 0) # Plotting the Predicted Pixels (after conversion)

# print(all_x)
# print(all_y)

