from vehicle_motion_api import VehicleMotionAPI
import rospy
from std_msgs.msg import Header, String, Int32
import time
import threading
import math
import carla

# Global variable
detect_distance_threshold = 5
traffic_light_state = None
min_distance_global = None
vehicle_location = None
turn_right_signs = []
# state_lock = threading.Lock()
VM = VehicleMotionAPI()

# def find_all_turn_right_sign(traffic_signs):
#     global turn_right_signs
#     for sign in traffic_signs:
#         # Check if the sign indicates "turn right"
#         if sign.id is 6:
#             turn_right_signs.append(sign)

# def Is_turn_right_near(vehicle_position, turn_right_signs):
#     """
#     vehicle_position
#     turn_right_signs: list of turn right sign on the map
#     """
#     if not turn_right_signs:
#         for sign in turn_right_signs:
#             # get traffic sign location
#             sign_location = sign.transform.positionss
        
#             # caculate vector from vehicle to traffic sign
#             relative_position_vector = sign_location - vehicle_position

#             distance = relative_position_vector.length()
            
#             #distance = sign_location.distance(vehicle_position)

#             if distance <= detect_distance_threshold:
#                 # show warning 
#                 print("WARNING: Direct Turn Right in {} meters", distance )
#                 return True
#     else:
#         print("No Direct Turn Right on the map")
#     return False

# def Is_traffic_sign_near(vehicle_position, traffic_signs):
#     traffic_sign_list_near = []
#     if not traffic_signs:
#         for sign in traffic_signs:
#             # get traffic sign location
#             sign_location = sign.transform.position
        
#             # caculate vector from vehicle to traffic sign
#             relative_position_vector = sign_location - vehicle_position
#             distance = relative_position_vector.length()

#             #distance = sign_location.distance(vehicle_position)

#             if distance <= detect_distance_threshold:
#                 # show warning 
#                 if sign.id is 1:
#                     print("WARNING: Stop sign in {} meters", distance )
#                 elif sign.id is 2:
#                     print("WARNING: Speed Limited 40 sign in {} meters", distance )
#                 elif sign.id is 3:
#                     print("WARNING: Speed Limited 60 sign in {} meters", distance )
#                 elif sign.id is 4:
#                     print("WARNING: Speed Limited 90 sign in {} meters", distance )
#                 elif sign.id is 5:
#                     print("WARNING: Direct Turn Left sign in {} meters", distance )
#                 elif sign.id is 6:
#                     print("WARNING: Direct Turn Right sign in {} meters", distance )
#                 elif sign.id is 7:
#                     print("WARNING: Direct Straight sign in {} meters", distance )
#                 elif sign.id is 8:
#                     print("WARNING: Prohibiting right turn sign in {} meters", distance ) 
#                 elif sign.id is 9:
#                     print("WARNING: Prohibiting left turn sign in {} meters", distance ) 
#                 elif sign.id is 10:
#                     print("WARNING: Prohibiting straight turn sign in {} meters", distance ) 
                
#                 traffic_sign_list_near.append(sign)
#     else:
#         print("No Traffic Sign on the map")

#     if not traffic_sign_list_near:
#         return True, traffic_sign_list_near
#     else:
#         return False, traffic_sign_list_near

# Function to get the closest traffic light state
def get_closest_traffic_light():
    global vehicle_location
    closest_light = None
    min_distance = float('inf')
    traffic_lights = VM.General.get_traffic_lights()
    vehicle_location = VM.General.get_vehicle_location()
    if traffic_lights is not None:
        for traffic_light in traffic_lights:
            if traffic_light is not None:
                # print(traffic_light)
                distance = math.sqrt((vehicle_location.x - traffic_light.transform.position.x) * (vehicle_location.x - traffic_light.transform.position.x) +
                                    (vehicle_location.y - traffic_light.transform.position.y) *(vehicle_location.y - traffic_light.transform.position.y))
                if distance < min_distance:
                    min_distance = distance
                    closest_light = traffic_light
    return closest_light,min_distance


# Thread function to check traffic light status
def check_traffic_light():
    global traffic_light_state
    global min_distance_global
    while True:
        traffic_light,min_distance = get_closest_traffic_light()
        time.sleep(0.1)
        if traffic_light is not None:
            try:
                # with state_lock:
                traffic_light_state = traffic_light.state
                min_distance_global = min_distance
                    
            except:
                print("failed to get traffic light")
        time.sleep(0.1)

# Thread function to automatic stop vehicle when traffic light is red
def automatic_stop_redl():
    global traffic_light_state
    global min_distance_global
    control_flag = 0
    while(1):
        # with state_lock:
        current_state = traffic_light_state
        current_distance = min_distance_global
        if(current_state is not None and current_distance is not None):
            if(control_flag):
                VM.General.vehicle_control_manual_override(False)
                time.sleep(5)
                control_flag = 0
                continue
            elif((current_distance <= 5) and (current_state == 0)):
                VM.General.vehicle_control_manual_override(True)
                VM.General.vehicle_control_manual(throttle=0, brake=1)
                print("--------------------------Stop at RED LIGHT-----------------------------------")
                continue
                
            elif( (current_distance <= 10) and (current_state == 1) ):
                VM.General.vehicle_control_manual_override(True)
                VM.General.vehicle_control_manual(throttle=0, brake=0)
                # time.sleep(3)
                print("-------------------Decrease velocity at YELLOW LIGHT---------------------------")
                VM.General.vehicle_control_manual_override(False)
                # control_flag = 1
            
            elif(current_distance <= 5):
                VM.General.vehicle_control_manual_override(False)
                control_flag = 1
            else:
                continue

# Check obstaccle
def process_at_obstacle():
    while(True):
        obstacle = VM.General.get_vehicle_obstacle()
        if(obstacle.is_obstacle):
            VM.General.vehicle_control_manual_override(True)
            VM.General.vehicle_control_manual(throttle=0, brake=1)
            print("--------------------------Stop at obstacle-----------------------------------")
            # --------------------------------Turn on Haz light------------------------------
            time.sleep(1)
            VM.General.vehicle_control_manual_override(False)


def check_weather():
    weather = VM.General.get_weather()
    if(weather == "ClearNight"):
        VM.General.vehicle_control_light("On")
    else:
        VM.General.vehicle_control_light("Off")


def find_closest_traffic_sign():
    global vehicle_location
    closest_traffic_sign = None
    min_distance = float('inf')
    traffic_signs = VM.General.get_traffic_signs()
    # print("Traffic sign", traffic_signs)
    try:
        if (traffic_signs is not None and len(traffic_signs) != 0):
            for traffic_sign in traffic_signs:
                if traffic_sign is not None:
                    distance = math.sqrt((vehicle_location.x - traffic_sign.transform.position.x) ** 2 +
                                        (vehicle_location.y - traffic_sign.transform.position.y) ** 2 +
                                        (vehicle_location.z - traffic_sign.transform.position.z) ** 2)
                    if distance < min_distance:
                        min_distance = distance
                        closest_traffic_sign = traffic_sign
    except Exception as e:
        print(e)

    return closest_traffic_sign,min_distance

def process_traffic_sign():
    control_flag = 0
    while(True):
        traffic_sign, distance = find_closest_traffic_sign()
        if traffic_sign is not None:
            if distance <= detect_distance_threshold:
                if traffic_sign.type == 1:
                    # Stop sign, stop car when distance is less than or equal to 10 meters in 3 seconds
                    VM.General.vehicle_control_manual_override(True)
                    VM.General.vehicle_control_manual(throttle=0, brake=1)
                    time.sleep(3)
                    VM.General.vehicle_control_manual_override(False)
                    control_flag = 1
                    if(control_flag):
                        time.sleep(5)
                        control_flag = 0
                elif traffic_sign.type == 2:
                    print("WARNING: Speed Limited 40 sign in {} meters", distance )
                elif traffic_sign.type == 3:
                    print("WARNING: Speed Limited 60 sign in {} meters", distance )
                elif traffic_sign.type == 4:
                    print("WARNING: Speed Limited 90 sign in {} meters", distance )
                elif traffic_sign.type == 5:
                    print("WARNING: Direct Turn Left sign in {} meters", distance )
                elif traffic_sign.type == 6:
                    print("WARNING: Direct Turn Right sign in {} meters", distance )
                elif traffic_sign.type == 7:
                    print("WARNING: Direct Straight sign in {} meters", distance )
                elif traffic_sign.type == 8:
                    print("WARNING: Prohibiting right turn sign in {} meters", distance ) 
                elif traffic_sign.type == 9:
                    print("WARNING: Prohibiting left turn sign in {} meters", distance ) 
                elif traffic_sign.type == 10:
                    print("WARNING: Prohibiting straight turn sign in {} meters", distance )

def main():
    """
    Main function
    """

    
    try:
        VM.General.vehicle_control_manual_override(False)

        traffic_light_thread = threading.Thread(target=check_traffic_light)
        automatic_stop_red_thread = threading.Thread(target=automatic_stop_redl)
        traffic_sign_thread = threading.Thread(target=process_traffic_sign)
        process_at_obstacle_thread = threading.Thread(target=process_at_obstacle)

        traffic_light_thread.start()
        automatic_stop_red_thread.start()
        traffic_sign_thread.start()
        process_at_obstacle_thread.start()
        
        time.sleep(1)
        door = VM.General.get_vehicle_door()
        if(door.FL == "Open"):
            VM.General.vehicle_control_toggle_door_FL()
        if(door.FR == "Open"):
            VM.General.vehicle_control_toggle_door_FR()
        if(door.RR == "Open"):
            VM.General.vehicle_control_toggle_door_RR()
        if(door.RL == "Open"):
            VM.General.vehicle_control_toggle_door_RL()
        while (not VM.General.exit_flag):
            """ Begin application code """
            check_weather(VM.General)
            print("Start manually")
            VM.General.vehicle_control_manual(throttle=0.3, brake=0)
            # break
        
        
            
            """ End application code """
    except KeyboardInterrupt:
        print("Shutting down")
    
    traffic_light_thread.join()
    automatic_stop_red_thread.join()
    traffic_sign_thread.join()
    process_at_obstacle_thread.join()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    