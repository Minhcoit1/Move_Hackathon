from vehicle_motion_api import VehicleMotionAPI
import rospy
from std_msgs.msg import Header, String, Int32
import time
import math
import threading

# Global variables
closest_traffic_light_state = None
closest_traffic_light_distance = None
state_lock = threading.Lock()

# calculate distance
def calculate_distance_to_traffic_light(traffic_light):
    distance = None




# Function to get the closest traffic light of the vehicle
def find_closest_traffic_light(vehicle_location,traffic_lights):
    closest_traffic_light = None
    min_distance = float('inf')
    try:
        for traffic_light in traffic_lights:
            if traffic_light is not None:
                distance = math.sqrt((vehicle_location.x - traffic_light.transform.position.x) ** 2 +
                                    (vehicle_location.y - traffic_light.transform.position.y) ** 2 +
                                    (vehicle_location.z - traffic_light.transform.position.z) ** 2)
                if distance < min_distance:
                    min_distance = distance
                    closest_traffic_light = traffic_light
    except:
        print("Error")

    return closest_traffic_light,min_distance

# Check traffic light status function
def check_traffic_light(vehicle_location,traffic_lights):
    global closest_traffic_light_state
    while True:
        traffic_light = find_closest_traffic_light(vehicle_location,traffic_lights)
        if traffic_light is not None:
            with state_lock:
                closest_traffic_light_state = traffic_light.state
        time.sleep(0.1)

def main():
    """
    Main function
    """
    try:
        VM = VehicleMotionAPI()
        time.sleep(1)
        while (not VM.General.exit_flag):
            """ Begin application code """
            VM.General.vehicle_control_manual_override(False)
            # time.sleep(0.1)
            # VM.General.vehicle_control_manual(throttle=1, brake=0)
            # time.sleep(1)
            # VM.General.vehicle_control_manual(throttle=0, brake=1)
            VM.General.vehicle_control_light("On")
            time.sleep(1)
            vehicle_location = VM.General.get_vehicle_location()
            # print(vehicle_location)
            traffic_lights = VM.General.get_traffic_lights()
            # print(traffic_lights)
            closest_traffic_light,min_distance = find_closest_traffic_light(vehicle_location,traffic_lights)
            # break
            print("Closest Traffic Light: " ,closest_traffic_light.id)
            
            """ End application code """
    except KeyboardInterrupt:
        print("Shutting down")
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


    