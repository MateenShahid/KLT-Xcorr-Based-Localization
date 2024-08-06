#!/bin/bash
source /home/odroid/miniforge3/bin/activate visp
echo $(date) >> /home/odroid/lucas-kanade-tracker-master/out.txt
#python3 /home/odroid/lucas-kanade-tracker-master/driver_2.py >> /home/odroid/lucas-kanade-tracker-master/out.txt
python3 /home/odroid/lucas-kanade-tracker-master/vid_rec.py >> /home/odroid/lucas-kanade-tracker-master/out.txt