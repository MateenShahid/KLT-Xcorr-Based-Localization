from main.ED_Communication.mav_Comm import *
from random import randrange
import queue, math
import socket
import time


class Warfare_Network:
    def __init__(self, list_coms, model_gps_pxl, glb_compression, glb_crop):
        self.model_gps_pxl = model_gps_pxl
        self.glb_compression = glb_compression
        self.glb_crop = glb_crop
        self.queued_missions = queue.Queue()
        self.list_drones = []
        for item in list_coms:
            self.list_drones.append(Drone(item, "com",model_gps_pxl = self.model_gps_pxl, glb_compression=  self.glb_compression, glb_crop = self.glb_crop, error_message=self.gen_error_msg()))

        # The last index is for the drone connected with Dr. Umer ED Pipeline
        self.list_drones.append(Drone("", "Wi-Fi", error_message=self.gen_error_msg()))
        # connect_drones = threading.Thread(target=self.connect_to_drones, args=())
        # connect_drones.start()

    def gen_error_msg(self):
        errors = [["The drone is not ready"],
                  ["The drone can not be armed"],
                  ["The battery is not sufficient for this mission"]]
        return errors[randrange(len(errors))]

    def connect_to_drones(self):
        for item in self.list_drones:
            if item.conn_type == "com":
                ret = item.connect_to_vehicle()
                if ret:
                    try:
                        item.update_drone_info(item.drone_location[0], item.drone_location[1], item.drone_location[2], 80 )
                    except Exception as e:
                        print(e)

            time.sleep(3)
        time.sleep(5)
        ping_drones = threading.Thread(target=self.ping_thread, args=())
        # ping_drones.start()

    def ping_thread(self):
        while True:
            for drone in self.list_drones:
                if drone.connected and drone.conn_type == "com":
                    drone.heartbeat()
            time.sleep(5)

    def queue_mission(self, mission):
        self.queued_missions.put(mission)

    def get_oldest_queued_mission(self):
        try:
            return self.queued_missions.get(timeout=0.5)
        except:
            return None

    def simmulate_execute(self, mission, drone_id, model_gps_pxl, glb_compression, glb_crop):
        wps = mission
        self.list_drones[drone_id].connected = False

        time.sleep(5)
        self.list_drones[drone_id].status = "Drone is Armed"
        print("Drone is Armed")

        time.sleep(5)
        self.list_drones[drone_id].status = "Drone is Taking off"
        print("Drone is Taking off")

        for i, wp in enumerate(wps):
            time.sleep(5)
            x, y = model_gps_pxl.predict([[wp[1], wp[0]]])[0]
            self.list_drones[drone_id].drone_location = wp
            self.list_drones[drone_id].pixel_location = (
            math.floor(glb_compression * x - glb_crop), math.floor(glb_compression * y - glb_crop))
            self.list_drones[drone_id].status ="Reached WP " + str(i+1)

        self.list_drones[drone_id].status = 'Reached target'
        print("Reached target")
        time.sleep(5)
        print("Drone returning to base")


    def send_target_info_to_ed(self, target, lat, lng, alt):


        PORT = 1111
        IP = '172.20.10.12'
        try:
            client_socket = socket.socket()
            client_socket.connect((IP, PORT))

            # bytes = target.read()
            target_size = len(target)
            print("Target Size: ", target_size)

            # Sending Lat Lng ALt
            message = '#TARGET_INFO: ' + str(lat) + ' ' + str(lng) + ' ' + str(alt) + ' ' + str(target_size) + ' \n'
            client_socket.send(message.encode())
            time.sleep(1)

            # Sending Target Frame
            client_socket.sendall(target)

            # Closet socket
            client_socket.close()
            return True, None
        except Exception as e:
            return False, e

    def send_cmd_to_ed(self, cmd):
        PORT = 1111
        IP = '172.20.10.12'
        try:
            client_socket = socket.socket()
            client_socket.connect((IP, PORT))
            client_socket.send(cmd.encode())
            time.sleep(1)
            client_socket.close()
            return True, None
        except Exception as e:
            return False, e



if __name__ == "__main__":
    import cv2, base64
    drone = Warfare_Network(["com4", "com5", "com6", "com7", "com8"])


    frame = cv2.imread('D:\Drone\Drone_Pipeline\datasets\Drone_Images\car.png')
    retval, buffer = cv2.imencode('.jpg', frame)
    target = bytes(buffer)

    while True:
        succ, error = drone.send_cmd_to_ed("$ARM")
        print('arm succ', succ, 'error', error)
        a = input('arm?')
        if a == 'y':
            print('armed suucc')
            break

    succ, error = drone.send_target_info_to_ed(target, 31.420481, 74.253589, 100)
    print('target sending succ', succ, 'error', error)

    if succ:
        print('sending comand')
        succ, error = drone.send_cmd_to_ed("#EXECUTE")
        print('execute succ', succ, 'error', error)
    else:
        print("Error in sending to ed",error)
