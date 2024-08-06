from __future__ import print_function
import threading
import time
from pymavlink import mavutil
from pymavlink import mavwp
# from main.utils.gps_utils import distace_btw_gps
import math, time
from datetime import datetime


class Drone:
    def __init__(self, connection_string="com7", connection_type="com", baud_rate=1500000, armable=False, uplink=False,
                 sensors_check=False, gps_lock=False, error_message=None, model_gps_pxl=None, glb_compression=0.1,
                 glb_crop=130, stamp = '-'.join(str(datetime.now()).split())):
        # Either com or Wi-Fi, com for telemetry and Wi-Fi for the ED
        self.conn_type = connection_type
        self.connection_string = connection_string
        self.baud_rate = baud_rate
        self.vehicle = None
        # Placeholder for when the drone connects
        self.connected = False
        self.status = 'None'
        self.drone_location = None
        self.pixel_location = (0, 0)
        self.ortho_location = (0, 0)
        self.alt = 0
        # self.lat = 0
        # self.lon = 0
        # self.head = 0
        self.type = 'QuadCopter'
        self.armable = armable
        self.battery_percentage = 100
        self.uplink = uplink
        self.sensors_check = sensors_check
        self.gps_lock = gps_lock
        self.compass_cal = False
        self.error_message = error_message
        self.model_gps_pxl = model_gps_pxl
        self.glb_compression = glb_compression
        self.glb_crop = glb_crop
        self.rtl = False
        self.fil = f'/home/odroid/lucas-kanade-tracker-master/logs/log-flight-' + stamp + '.txt'


    def location_thread(self):
        self.flightlog = open(self.fil, 'w+')
        self.flightlog.write('\n####################################\n')
        self.flightlog.write(f'{datetime.now()}\n\n')
        self.flightlog.close()
        time.sleep(1)
        while True:
            try:
                # print("Test")
                msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                #print('MSG',msg)
                self.drone_location = (msg.lat / 10000000, msg.lon / 10000000, msg.relative_alt / 1000)

                self.flightlog = open(self.fil, 'a')
                self.flightlog.write(f'{datetime.now()}        {self.drone_location}\n')
                self.flightlog.close()
                #x, y = self.model_gps_pxl.predict([[msg.lon / 10000000, msg.lat / 10000000]])[0]
                #self.ortho_location = (x, y)
                #self.pixel_location = (math.floor(self.glb_compression * x - self.glb_crop),
                #                       math.floor(self.glb_compression * y - self.glb_crop))
                self.alt = msg.relative_alt / 1000
                ###################################
                self.lat = msg.lat / 10000000
                self.lon = msg.lon / 10000000
                ###################################

                msg1 = self.vehicle.recv_match(type='VFR_HUD', blocking=False)
                self.head = msg1.heading
                #print('Heading:', msg1.heading)

                #if self.rtl and self.alt < 5:
                #    self.rtl = False
                #    self.status = "Connection Established"
                #    self.connected = True
                time.sleep(1)
                # print(self.drone_location)
                # print(msg)
                

            except Exception as e:
                print(self.connection_string, ": ", e)
                # print('No GPS_RAW_INT message received')
                time.sleep(1)

    def connect_to_vehicle(self):
        print('Connecting to vehicle on: %s' % self.connection_string)
        tries = 5
        while tries != 0:
            try:
                a = time.time()
                # Start a connection listening on a UDP port
                self.vehicle = mavutil.mavlink_connection(self.connection_string, baud=self.baud_rate)
                print("Waiting for heartbeat ")
                # Wait for the first heartbeat
                self.vehicle.wait_heartbeat()
                print("Heartbeat from system (system %u component %u)" % (
                    self.vehicle.target_system, self.vehicle.target_component))

                b = time.time() - a
                print('Connection with vehicle established in', b)

                self.connected = True
                self.status = 'Connection Established'
                break

            except Exception as e:
                print('Failed to connect to vehicle :', e)

                tries -= 1
                time.sleep(0.5)
        if tries == 0:
            print("Could not establish connection with vehicle on: %s" % self.connection_string)
            return False
        loc_thread = threading.Thread(target=self.location_thread, args=())
        loc_thread.start()

        # Drone mode change 4 = guided
        self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component,
                                           176, 0, 1, 4, 0, 0, 0, 0, 0)

        return True

    def get_location(self):
        if self.status:
            # loc = self.vehicle.location.global_relative_frame
            # self.drone_location = (loc.lat, loc.lon, loc.alt)
            return self.drone_location
        else:
            return None

    def set_drone_home_location(self):
        lat = self.drone_location[0]
        long = self.drone_location[1]
        alt = self.drone_location[2]
        while lat == 0:
            time.sleep(0.5)

        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            1,  # set position
            0,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            lat,  # lat
            long,  # lon
            alt)

    # def upload_to_drone(self, gps_list):
    #     """
    #     Clears previous commands and uploads new mission to the drone
    #     """
    #     print("Uploading mission waypoints to the drone")
    #     wp = mavwp.MAVWPLoader()

    #     for c in range(0, len(gps_list)):
    #         lat = gps_list[c][0]
    #         long = gps_list[c][1]
    #         height = gps_list[c][2]

    #         if c == 0:
    #             p = mavutil.mavlink.MAVLink_mission_item_message(self.vehicle.target_system,
    #                                                              self.vehicle.target_component,
    #                                                              c + 1, 3, 16, 0, 1, 0, 0, 0, 0, lat, long, height)
    #             wp.add(p)
    #             wp.add(p)

    #         else:
    #             p = mavutil.mavlink.MAVLink_mission_item_message(self.vehicle.target_system,
    #                                                              self.vehicle.target_component,
    #                                                              c + 1, 3, 16, 0, 1, 0, 0, 0, 0, lat, long, height)
    #             wp.add(p)

    #     # Clear previous waypoints
    #     self.vehicle.waypoint_clear_all_send()

    #     # Send new waypoints
    #     self.vehicle.waypoint_count_send(wp.count())

    #     for i in range(wp.count()):
    #         msg = self.vehicle.recv_match(type=['MISSION_REQUEST'], blocking=True)
    #         # print(msg)
    #         self.vehicle.mav.send(wp.wp(msg.seq))
    #         # print('Sending waypoint {0}'.format(msg.seq))

    #     print("Mission Uploaded Successfully")
    #     self.status = 'Mission Uploaded'
    #     return gps_list

    def arm(self):
        """
        Arms vehicle
        """
        result = 4
        while result != 0:
            print("Trying to Arm")
            self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component,
                                               mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
            print("Command Sent")
            result = 0
            # result = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True)
            # result = result.result
            # print(result)
            # if result != 0:
            #     print("Can not arm, trying again in 5 seconds")
            #     while True:
            #         msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=False)
            #         if msg:
            #             mode = mavutil.mode_string_v10(msg)
            #             print(mode)
            #             break
            #     time.sleep(5)
        print("Arm Successful")
        self.status = "Drone is Armed"

    def takeoff(self, alt):
        """
        Takeoff vehicle to target altitude
        """
        print("Taking Off")

        self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component,
                                           mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)

        self.status = "Drone is Taking off"
       # msg = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True)
       # print(msg)

    # def mission_start(self):
    #     self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component,
    #                                        mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
    #     msg = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True)
    #     self.status = "Mission is being executed"

    # def return_to_launch(self):
    #     self.rtl = True
    #     self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component,
    #                                        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)
    #     msg = self.vehicle.recv_match(type='COMMAND_ACK', blocking=False)
    #     self.status = "Drone is returning to Launch"

    def arm_and_takeoff(self, takeOffAlt=10):

        # Drone mode change 4 = guided
        self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component,
                                           176, 0, 1, 4, 0, 0, 0, 0, 0)
        while self.connected != True:
            print('waiting for drone connection', self.connected)
            time.sleep(5)

        self.arm()
        time.sleep(5)
        self.takeoff(takeOffAlt)
        while self.alt < 0.75 * takeOffAlt:
            time.sleep(0.1)
        self.connected = False

    # def send_waypoint(self, lat, long, alt):
    #     self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
    #         0,  # time_boot_ms
    #         self.vehicle.target_system,
    #         self.vehicle.target_component,
    #         mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # frame of reference
    #         3576,  # type mask
    #         int(lat * 10000000),  # lat int
    #         int(long * 10000000),  # lon int
    #         alt,  # alt
    #         0,  # vx
    #         0,  # vy
    #         0,  # vz
    #         0,  # acc fx
    #         0,  # acc fy
    #         0,  # acc fz
    #         0,  # yaw radians
    #         0,  # yaw rate rad/s

    #     ))

    # def distance_from_waypoint(self, lat2, lon2):
    #     # msg = self.vehicle.recv_match(type='AHRS2', blocking=True)
    #     # lat1 = msg.lat / 10000000
    #     # lon1 = msg.lng / 10000000
    #     return distace_btw_gps(self.drone_location[0], self.drone_location[1], lat2, lon2)

    # def get_drone_info(self):

    #     # Placeholder for when the drone connects

    #     res = {}
    #     res['type'] = self.type
    #     res['conn_type'] = self.conn_type
    #     res['connection_string'] = self.connection_string
    #     res['baud_rate'] = self.baud_rate
    #     res['vehicle'] = None
    #     res['armable'] = self.armable
    #     res['conn_type'] = self.conn_type
    #     res['connected'] = self.connected
    #     res['status'] = self.status
    #     res['drone_location'] = self.drone_location
    #     res['battery_percentage'] = self.battery_percentage
    #     res['uplink'] = self.uplink
    #     res['compass_cal'] = self.compass_cal
    #     res['error_message'] = self.error_message
    #     res['pixel_location'] = self.pixel_location
    #     res['gps_lock'] = self.gps_lock
    #     res['sensors_check'] = self.sensors_check
    #     return res

    def update_drone_info(self, lat, long, alt, batt):

        # Placeholder for when the drone connects
        self.connected = True
        self.status = 'Connected to Drone '

        self.alt = alt
        self.drone_location = [lat, long]
        self.battery_percentage = batt
        self.uplink = True
        self.sensors_check = True
        self.gps_lock = True
        self.compass_cal = True
        self.error_message = []
        self.status = "Drone is Connected"
        x, y = self.model_gps_pxl.predict([[long, lat]])[0]
        self.ortho_location = (x, y)
        self.pixel_location = (
        math.floor(self.glb_compression * x - self.glb_crop), math.floor(self.glb_compression * y - self.glb_crop))

    # def get_curr_loc(self):
    #     x = self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component,
    #                                        512, 0, 33, 0, 0, 0, 0, 0, 0)
    #     print(x)



if __name__ == "__main__":

     connection_string = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0"
     drone = Drone(connection_string=connection_string)
     drone.connect_to_vehicle()
     print('Drone is connected',connection_string)
#     #
#     # print(drone.get_location())
#     time.sleep(4)
    # print(drone.get_location())
    # 31.419917, 74.254190
    # 31.419940, 74.253737
    # 31.420223, 74.254151

    # drone.arm()
    # drone.takeoff(10)
    # time.sleep(10)
    # for i in range(9):
    #     drone.send_waypoint(31.419917, 74.254190, 20)
    #     time.sleep(1)
    # drone.return_to_launch()
    # for i in range(9):
    #     drone.send_waypoint(31.419940, 74.253737, 20)
    #     time.sleep(1)
    # for i in range(9):
    #     drone.send_waypoint(31.420223, 74.254151, 20)
    #     time.sleep(1)
    # for i in range(9):
    #     drone.send_waypoint(31.419917, 74.254190, 20)
    #     time.sleep(1)
    # print(drone.get_location())

    # drone.execute_mission(10)
    # mission = {'waypoints': [(31.419917, 74.254190,20), (31.419940, 74.253737, 20)]}
    # execute_mission_following(mission, "com4")
