import cv2
from skimage.feature import match_template
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import math


def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy


class CrossCorr:
    def __init__(self, global_path, comp_ratio, ref_res, global_res, oldpos):
        """
        global_path: the path to the global map image
        comp_ratio: The ratio by which we compress the images
        ref_res: This is the meters/pixels of the onboard/reference image
        global_res: This is the meters/pixels of the global image
        oldpos: This is the initial position of the drone that we use further on

        The angle should be given in radians.
        """
        self.compression_ratio = comp_ratio
        self.reference_res = ref_res
        self.global_res = global_res
        self.ratio = self.reference_res / self.global_res
        self.global_map = cv2.imread(global_path, cv2.IMREAD_GRAYSCALE)
        self.global_map = cv2.resize(self.global_map, (0, 0), fx=self.compression_ratio, fy=self.compression_ratio)
        (self.h, self.w) = self.global_map.shape[:2]
        (self.cX, self.cY) = (self.w // 2, self.h // 2)

        self.oldPosition = oldpos

    # def object_localisation(self, x, y, onboard_image, heading, heading_error ):
        # activate the pygame library .
        # initiate pygame and give permission
        # to use pygame's functionality.
        # pygame.init()
        # onboard_image = cv2.resize(onboard_image, (0, 0), fx=self.ratio, fy=self.ratio)

        # # onboard image dims for plotting
        # height, width, _ = onboard_image.shape

        # # create the display surface object
        # # of specific dimension..e(X, Y).
        # display = pygame.display.set_mode((width, height))

        # # set the pygame window name
        # pygame.display.set_caption('Click target')

        # # create a surface object, image is drawn on it.
        # opencv_image = onboard_image[:, :, ::-1]
        # shape = opencv_image.shape[1::-1]
        # pygame_image = pygame.image.frombuffer(opencv_image.tostring(), shape, 'RGB')

        # display.blit(pygame_image, (0, 0))
        # pygame.display.update()  # Update screen

        # end = False

        # while not end:
        #     for event in pygame.event.get():
        #         if event.type == pygame.QUIT:
        #             pygame.quit()
        #             end = True
        #             break
        #         elif event.type == pygame.MOUSEBUTTONDOWN:
        #             pos = pygame.mouse.get_pos()
        #             # print('click', pos)
        #             fig = plt.figure(figsize=(12, 8))
        #             ax2 = plt.subplot(1, 1, 1)
        #             ax2.imshow(self.global_map, cmap=plt.cm.gray)
        #             ax2.set_axis_off()
        #             ax2.set_title('image')
        #             ax2.plot(x, y, marker="o", markersize=8,
        #                      markeredgecolor="blue", )
        #             target = (x - width / 2 + pos[0], y - height / 2 + pos[1])
        #             target = rotate((x, y), target, math.radians(heading + heading_error))
        #             ax2.plot(target[0], target[1], marker="x", markersize=8,
        #                      markeredgecolor="red", )
        #             plt.show()



    def rotate_global_map(self, angle):
        rot_mat = cv2.getRotationMatrix2D((self.cX, self.cY), angle, 1.0)
        self.global_map = cv2.warpAffine(self.global_map, rot_mat, (self.w, self.h))

    def find_cross_corr(self, onboard_image, heading, heading_error, disp_meters, leeway):
        """
        Finds cross correlation between onboard image and global image to see where is it on global map
        onboard_image: the frame captured on drone
        heading: The angle of the drone during frame capture, calculated from north
        heading_error: The error of compass
        disp_meters: The displacement of drone between previous and current frame, fed through KLT

        :return: The x, y position of onboard image on global map
        """
        # print(heading + heading_error)

        # Rotate the global_map (Ortho or gmap according to the heading of the onboard image)
        self.rotate_global_map(heading + heading_error)

        # Scale the onboard image according to the global map. This is so that onboard image and global map both have
        # same pixel density or pixel/meters
        onboard_image = cv2.resize(onboard_image, (0, 0), fx=self.ratio, fy=self.ratio)

        # Cross Correlation of global and onboard image
        result = match_template(self.global_map, onboard_image)
        # print(result.shape)

        # this is the leeway we add to search only the space around the previous point, this makes sure our XCorr
        # results do not deviate too far from the previous spot on Global-map, this also increase efficiency and acc
        displacement = (disp_meters + leeway) / (self.global_res / self.compression_ratio)

        # Rotating the old point according to the heading angle, because the point is on Global-Map and its rotated
        pos = rotate(((len(self.global_map[1, :])) / 2, (len(self.global_map)) / 2),
                     (self.oldPosition[0], self.oldPosition[1]),
                     math.radians(360 - (heading + heading_error)))

        # This is to account for the XCorr result which has corners cut off during processing
        pos = [pos[0] - (len(onboard_image[1, :])) / 2, pos[1] - (len(onboard_image)) / 2]

        # Figuring where to crop our region from
        crop_reg_x1 = int(pos[0] - displacement / 2)
        crop_reg_x2 = int(pos[0] + displacement / 2)
        crop_reg_y1 = int(pos[1] - displacement / 2)
        crop_reg_y2 = int(pos[1] + displacement / 2)
        if crop_reg_x1 < 0:
            crop_reg_x1 = 0
        if crop_reg_x2 < 0:
            crop_reg_x2 = 0
        if crop_reg_y1 < 0:
            crop_reg_y1 = 0
        if crop_reg_y2 < 0:
            crop_reg_y2 = 0

        # Cropping the XCorr result
        temp_res = result
        # print('pos',pos, 'dis',displacement)
        # print('resukt', result.shape)
        # print('regions',crop_reg_y1, crop_reg_y2, crop_reg_x1,crop_reg_x2)
        result = result[crop_reg_y1: crop_reg_y2, crop_reg_x1:crop_reg_x2]
        # print('result resize', result.shape)

        # Get the x,y peak of Cropped result
        ij = np.unravel_index(np.argmax(result), result.shape)
        x, y = ij[::-1]

        addition_to_x = crop_reg_x1
        addition_to_y = crop_reg_y1

        # onboard image dims for plotting
        # height, width = onboard_image.shape

        # # Just plotting everything
        # fig = plt.figure(figsize=(12, 8))
        # ax1 = plt.subplot(1, 2, 1)
        # ax2 = plt.subplot(1, 2, 2)
        # # ax3 = plt.subplot(1, 3, 3, sharex=ax2, sharey=ax2)
        #
        # ax1.imshow(onboard_image, cmap=plt.cm.gray)
        # ax1.set_axis_off()
        # ax1.set_title('template')
        # ax1.plot(width / 2, height / 2, marker="x", markersize=8,
        #          markeredgecolor="red", )
        #
        # ax2.imshow(self.global_map, cmap=plt.cm.gray)
        # ax2.set_axis_off()
        # ax2.set_title('image')
        # # highlight matched region
        # rect = plt.Rectangle((x + addition_to_x, y + addition_to_y), width, height, edgecolor='r', facecolor='none')
        # ax2.plot(x + addition_to_x + width/2, y + addition_to_y + height/2, marker="x", markersize=8, markeredgecolor="red",)
        # ax2.add_patch(rect)

        # ax3.imshow(temp_res)
        # ax3.set_axis_off()
        # ax3.set_title('`match_template`\nresult')
        # # highlight matched region
        # ax3.autoscale(False)
        # ax3.plot(x, y, 'o', markeredgecolor='r', markerfacecolor='none',
        #          markersize=10)
        # plt.show()

        # This addition is to account for the fact that our resultant was cropped, so we add the crop values
        x = x + addition_to_x
        y = y + addition_to_y

        # Rotating the global map back to the original, so it can be used for frames further on according to their angle
        self.rotate_global_map(360 - (heading + heading_error))

        # This addition to account for the fact that our XCorr gets cut off at the sides when computing XCorr
        x = x + (len(onboard_image[1, :])) / 2
        y = y + (len(onboard_image)) / 2

        # Rotating the point according to the angle that was rotated before
        x, y = rotate(((len(self.global_map[1, :])) / 2, (len(self.global_map)) / 2), (x, y),
                      math.radians(heading + heading_error))

        # Storing the point in the oldPosition because it will be used in the next frame
        self.oldPosition = (x, y)
        return x, y

    # def plot_points_on_map(self, all_x, all_y, heading,filename = 'plot'):
    #     """
    #     This method just plots the point on the GlobalMap and displays it
    #     :return: nothing
    #     """
    #     self.rotate_global_map(heading)
    #     fig = plt.figure(figsize=(12, 8))
    #     ax1 = plt.subplot(1, 1, 1)
    #     ax1.imshow(self.global_map, cmap=plt.cm.gray)
    #     ax1.set_axis_off()
    #     ax1.set_title('image')

    #     x = np.array(range(300))
    #     ax1.plot(all_x, all_y, ls='solid', linewidth=2, color='red')
    #     plt.savefig('/home/odroid/lucas-kanade-tracker-master/InputFrames/plot.png') ##Saving the Plot to Desktop
    #     #plt.show()
    # #     self.rotate_global_map(360 - heading)


    def plot_points_on_map(self, all_x, all_y, heading):
        """
        This method just plots the point on the GlobalMap and displays it
        :return: nothing
        """
        self.rotate_global_map(heading)
        fig = plt.figure(figsize=(12, 8))
        ax1 = plt.subplot(1, 1, 1)
        ax1.imshow(self.global_map, cmap=plt.cm.gray)
        ax1.set_axis_off()
        ax1.set_title('image')

        x = np.array(range(300))
        ax1.plot(all_x, all_y, ls='solid', linewidth=2, color='red')
        # ax1.plot(all_gps_x, all_gps_y, ls='solid', linewidth=2, color='green')
        
    
        plt.savefig('/home/odroid/Desktop/plot') ##Saving the Plot to Desktop
        # plt.show()
        self.rotate_global_map(360 - heading)

    def plot_points_on_mapp(self, all_x, all_y, all_gps_x, all_gps_y, heading):
        """
        This method just plots the point on the GlobalMap and displays it
        :return: nothing
        """
        self.rotate_global_map(heading)
        fig = plt.figure(figsize=(12, 8))
        ax1 = plt.subplot(1, 1, 1)
        ax1.imshow(self.global_map, cmap=plt.cm.gray)
        ax1.set_axis_off()
        ax1.set_title('image')

        x = np.array(range(300))
        ax1.plot(all_x, all_y, ls='solid', linewidth=2, color='red')
        ax1.plot(all_gps_x, all_gps_y, ls='solid', linewidth=2, color='green')
        
        plt.savefig('/home/odroid/Desktop/plot2') ##Saving the Plot to Desktop
        # plt.show()
        
   
        
    # def plot_points_on_mapp(self, all_gps_x, all_gps_y, heading):
    #     """
    #     This method just plots the point on the GlobalMap and displays it
    #     :return: nothing
    #     """
    #     self.rotate_global_map(heading)
    #     fig = plt.figure(figsize=(12, 8))
    #     ax1 = plt.subplot(1, 1, 1)
    #     ax1.imshow(self.global_map, cmap=plt.cm.gray)
    #     ax1.set_axis_off()
    #     ax1.set_title('image')

    #     x = np.array(range(300))
    #     ax1.plot(all_gps_x, all_gps_y, ls='solid', linewidth=2, color='green')
        
    #     plt.savefig('/home/odroid/Desktop/plot2') ##Saving the Plot to Desktop
    #     plt.show()
        
        
