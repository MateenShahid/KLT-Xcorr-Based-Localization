import matplotlib.pyplot as plt
import cv2
import numpy as np
from skimage.feature import match_template

heading =118.9
refer_res =0.05
global_res = 0.0502
ratio = refer_res/global_res
angle = heading
global_map = cv2.imread(r'C:\Users\ruhan\Desktop\Xisys\lucas-kanade-tracker-master\data\Ravi_Block_150_meters_transparent_mosaic_group1.tif', cv2.IMREAD_GRAYSCALE)
(h, w) = global_map.shape[:2]
(cX, cY) = (w // 2, h // 2)

M = cv2.getRotationMatrix2D((cX, cY), heading, 1.0)
global_map = cv2.warpAffine(global_map, M, (w, h))

onboard_image = cv2.imread(r'C:\Users\ruhan\Desktop\Xisys\lucas-kanade-tracker-master\data\frame_1_video1.png', cv2.IMREAD_GRAYSCALE)

global_map = cv2.resize(global_map, (0, 0), fx=0.5, fy=0.5)
onboard_image = cv2.resize(onboard_image, (0, 0), fx=0.5, fy=0.5)

onboard_image = cv2.resize(onboard_image, (0, 0), fx=ratio, fy=ratio)


result = match_template(global_map, onboard_image)
ij = np.unravel_index(np.argmax(result), result.shape)
x, y = ij[::-1]

fig = plt.figure(figsize=(8, 3))
ax1 = plt.subplot(1, 3, 1)
ax2 = plt.subplot(1, 3, 2)
ax3 = plt.subplot(1, 3, 3, sharex=ax2, sharey=ax2)

ax1.imshow(onboard_image, cmap=plt.cm.gray)
ax1.set_axis_off()
ax1.set_title('template')

ax2.imshow(global_map, cmap=plt.cm.gray)
ax2.set_axis_off()
ax2.set_title('image')
# highlight matched region
hcoin, wcoin = onboard_image.shape
rect = plt.Rectangle((x, y), wcoin, hcoin, edgecolor='r', facecolor='none')
ax2.add_patch(rect)

ax3.imshow(result)
ax3.set_axis_off()
ax3.set_title('`match_template`\nresult')
# highlight matched region
ax3.autoscale(False)
ax3.plot(x, y, 'o', markeredgecolor='r', markerfacecolor='none', markersize=10)

plt.show()
