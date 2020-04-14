#!/usr/bin/env python
import sys
import carla
import controller_button
import rospy
import rosbag
from msg_folder.msg import MyRosMsg

# Function that reads rosbag files and returns what is the type of the vehicle when the record was made.
def get_car_from_rosbagfile(rosbag_file):
    bag=rosbag.Bag(rosbag_file)
    text=''
    for msg in bag.read_messages():
        text=str(msg)
        break
    text=text[text.find("name"):]
    vehicle=text[text.find('"')+1:text.find('",')]
    return vehicle

def get_car_cordinates(vehicle):
    return vehicle.get_location()


def drawing(cordinates,drawing):
    drawing.draw_point(cordinates,carla.Colore(0,255,0))

client = carla.Client('localhost', 2000)
client.set_timeout(10.0) 
world=client.get_world()
blueprint_library = world.get_blueprint_library()
draw=world.debug
vehicle = None

autopilot=False
button_autopilot=controller_button.Button()
control_car=carla.VehicleControl()


rosbag_file=sys.argv[1]
type_car=get_car_from_rosbagfile(rosbag_file)

car=blueprint_library.find(type_car)

transform = carla.Transform(carla.Location(x=30, y=0, z=3), carla.Rotation(yaw=180))
# print(transform)
vehicle=world.spawn_actor(car,transform)
control_car.brake=1.0
vehicle.apply_control(control_car)

# Function that recreates the movement of the car, when the vehicle was recorded.
def callback(data):
    global autopilot
    global button_autopilot
    global vehicle
    global draw
    global world
    if data.is_exit==True:
        vehicle.destroy()
        print("The console stoped :)")
        rospy.signal_shutdown("exit")
    if data.is_autopilot==True:
        if len(button_autopilot.get_list())!=0:
            if (button_autopilot.is_diferent_date()):
                spawning_actors.deleteAllelements(button_autopilot.get_list())
        if len(button_autopilot.get_list())==0:       
            button_autopilot.add_new_date_now()
            button_autopilot.set_state_of_pushed_button(True)

        if button_autopilot.get_button() is True:
            if autopilot is False:
                vehicle.set_autopilot(True)
                autopilot=True
            elif autopilot is True:
                autopilot=False
                vehicle.set_autopilot(False)
            button_autopilot.set_state_of_pushed_button(False)
    if autopilot == False:
        control_car=vehicle.get_control()
        control_car.throttle=data.throttle
        control_car.steer=data.steer
        control_car.reverse=data.is_reverse
        control_car.brake=data.breaks
        vehicle.apply_control(control_car)
        
        # control_car.reverse=data.is_reverse
        # control_car.brake=data.breaks
        # control_car.throttle=data.throttle
        # control_car.steer=data.steer
        # vehicle.apply_control(control_car)

# Main function that declares that this is a subscriber script.
def main():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("controler_info/full_info", MyRosMsg, callback)
    rospy.spin()

if __name__ == '__main__':
    main()