import numpy as np
import pandas as pd
from KLT_Tracker import KltTracker
from xcorr import CrossCorr
import cv2
import math, time
import pickle

#from mav_Comm import Drone

# Parameters to be used
frame_skips = 30
frame_skips_from_start = 0
fps = 30
total_frames_to_process = 50
klt_error_meters = 5
compression_ratio = 0.1
reference_res = 0.05 #Ravi Block
# reference_res = 0.0575  #LUMS Video
#reference_res = 0.0928571428571429 #LUMS Webcam

global_res = 0.0501851851851852  ##Ravi Block 
# global_res = 0.1031941031941032   ##LUMS
#global_res = 0.0353846153846154  ##For LUMSU by Turyal

heading_error = 0   ##For Ravi Block
# heading_error = 19                    ## For LUMSU Video by Turyal
meters_per_pixels = reference_res / compression_ratio
leeway = 10

heading_data = pd.read_excel('data/Ravi/ravi_first.xlsx') 
# heading_data = pd.read_excel('data/LUMS/lums1.xlsx')
heading_data = pd.DataFrame(heading_data, columns=[' compass_heading(degrees)'])
heading_data = heading_data[0:-1:int((frame_skips / fps) * 10)]

cap = cv2.VideoCapture(r'data/Ravi/ravi_first.MP4')
# cap = cv2.VideoCapture(r'data/LUMS/lums.MP4')
# cap = cv2.VideoCapture(6)

##################################
# textfile for log
file_name = 'log.txt'
log = open(file_name, 'a')
log.write('\n####################################\n')


##################################   
#while(True):
# Capture frame-by-frame
    #ret, fra = cap.read()
# Display the resulting frame
    #frame.empty():
    
# Waits for a user input to quit the application
    #if cv2.waitKey(1) & 0xFF == ord('q'):
       # break
# When everything done, release the capture
#cap.release()
#cv2.destroyAllWindows()
##################################

oldPosition = [3071.69 * compression_ratio, 1770.14 * compression_ratio] ## For Ravi Block
# oldPosition = [1856 * compression_ratio, 4288 * compression_ratio] ##LUMS webcam
# oldPosition = [4500 * compression_ratio, 7640 * compression_ratio] #For LUMSU by Turyal

xcorr_obj = CrossCorr(r'data/Ravi/ravi_block_150m.tif', compression_ratio, reference_res, global_res, oldPosition)
# xcorr_obj = CrossCorr(r'data/LUMS/lums.tif', compression_ratio, reference_res, global_res, oldPosition)

# xcorr_obj = CrossCorr(r'C:\Users\ruhan\Desktop\Xisys\lucas-kanade-tracker-master\data'
#                       r'\Ravi_Block_150_meters_transparent_mosaic_group1.tif', compression_ratio, reference_res,
#                       global_res)
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

number = 1
tim = time.time()
print("Entering loop")
j=1 ##For Saving the input Frames

while True:
    # for i in range(frame_skips):
    ret, frame = cap.read()
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
    # if int(frame_no) % 2 == 0:
    obj.update_feature_to_track(frame)
        # frame_no = 1

    disp_x_klt, disp_y_klt = obj.step(frame)
    acc_x = acc_x + disp_x_klt
    acc_y = acc_y + disp_y_klt
    
    
    #pred_lat, pred_long = reg.predict([[acc_x, acc_y]])[0]
    #print('prediction: ', pred_lat, pred_long)

    # Calculate the displacement in meters returned by the KLT
    displace_meters_klt = math.sqrt(disp_x_klt ** 2 + disp_y_klt ** 2)
    displace_meters_klt *= (reference_res / compression_ratio)

    #print("Displacement in Meters:", displace_meters_klt) 
    
    # Get the X, Y position on the global Map

    x, y = xcorr_obj.find_cross_corr(frame, heading_data.iloc[heading_index, 0], heading_error, displace_meters_klt, leeway)
    print("Localised at: ",x,y)
    # localisation object from the frame onto global map
    # xcorr_obj.object_localisation(x, y, t_frame, heading_data.iloc[heading_index, 0], heading_error)

    heading_index += 1
    log.write(f"{number}: {time.time()-tim}s : x = {x} , y = {y}\n")
    all_x = np.append(all_x, x)
    all_y = np.append(all_y, y)
    frame_no = frame_no + 1
    number = number + 1
    # if number == total_frames_to_process:
    #     break

tot_time = time.time()-tim

print("FPS",total_frames_to_process/tot_time)
xcorr_obj.plot_points_on_map(all_x, all_y, 0)

log.write(f'\n\n frame_rate = {total_frames_to_process/ tot_time} \n####################################\n ')
log.close()
print(total_frames_to_process,"infer time",tot_time)
# print(all_x)
# print(all_y)

