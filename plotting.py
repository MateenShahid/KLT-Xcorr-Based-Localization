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


compression_ratio = 0.1
reference_res = 0.092
global_res = 0.06715328467

all_x = np.array([], dtype=int)
all_y = np.array([], dtype=int)


oldPosition = [3000 * compression_ratio, 4110 * compression_ratio] #For Fortress
xcorr_obj = CrossCorr('/home/odroid/lucas-kanade-tracker-master/data/Fortress/fortress_pc.tif', compression_ratio, reference_res, global_res, oldPosition)
#################### Reading the Recently Stored Excel File #####################
# loading pixel to lat-long model
#reg = pickle.load(open('finalized_model.pkl', 'rb'))
regg = pickle.load(open('/home/odroid/lucas-kanade-tracker-master/GPS_Pixel_Models/FT_gps_pxl.pkl', 'rb'))

print("Reading Heading Data")
data = pd.read_excel('/home/odroid/lucas-kanade-tracker-master/data/FT_2.xlsx') 
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

print("Reading  X from Excel File")
all_x = pd.DataFrame(data, columns=['estimated_x'])
print("X read", all_x)

print("Reading  Y from Excel File")
all_y = pd.DataFrame(data, columns=['estimated_y'])
print("Read read", all_y)
################## Converting GPS to Pixels #######################
all_gps_x = np.array([], dtype=float)
all_gps_y = np.array([], dtype=float)

all_x = all_x.to_numpy()
all_y = all_y.to_numpy()

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
# xcorr_obj.plot_points_on_map(all_x, all_y, 0)

print(all_gps_x)
print(all_gps_y)
# print(all_x)
# print(all_y)
