#!/usr/bin/env python
import carla
from msg_folder.msg import MyRosMsg
from std_msgs.msg import Int16
import controller_button
import rospy
import publishing_all_information
import autopilot
import PID_Controller
import detecting_actors
import waypoints

client = carla.Client('localhost', 2000)
client.set_timeout(10.0) 
world=client.get_world()
button_autopilot=controller_button.Button()
autopilot_flag=False
control_car=carla.VehicleControl()
vehicle_id=None

pub = None

autopilot_module=None
vehicle = None 
rate=None

# Function that is called with the message as the first argument, when the subscriber receives new messages.
def callback(data):
    try:
        global autopilot
        global button_autopilot
        global vehicle_id
        global pub
        global vehicle
        global autopilot_module
        global autopilot_flag

        if vehicle is None:
            vehicle=world.get_actors().find(vehicle_id)
            road=waypoints.Creating_Path(vehicle)
            
            control_throttle=PID_Controller.PID_Controller_Throttle(vehicle,20)
            
            control_steer=PID_Controller.PID_Controller_Steer(vehicle)
            pid_control=PID_Controller.Controller(vehicle,control_throttle,control_steer)
            radar=detecting_actors.MyRadar(vehicle,15)
            autopilot_module = autopilot.Autopilot(vehicle,pid_control,radar,road)


        if data.is_exit==True:
            print("The console stoped :)")
            rospy.signal_shutdown("exit")

        if data.is_autopilot==False:
            if len(button_autopilot.get_when_the_button_is_released()) == 0 and len(button_autopilot.get_when_the_button_is_pushed()) != 0:
                button_autopilot.add_released_time()
                
        if data.is_autopilot==True:
            
            if len(button_autopilot.get_when_the_button_is_pushed())!=0 and len(button_autopilot.get_when_the_button_is_released())!=0:
                button_autopilot.set_date_of_pushed_and_released([])
                button_autopilot.reset_released_data([])
            if len(button_autopilot.get_when_the_button_is_pushed())==0:       
                button_autopilot.add_pushed_time()
                button_autopilot.set_state_of_button(True)
            # if button_autopilot.get_state_of_button() is True:
            #     if autopilot_flag is False:
            #         vehicle.set_autopilot(True)
            #         autopilot_flag=True
            #         print("AUTOPILOT ON")
            #     elif autopilot_flag is True:
            #         autopilot_flag=False
            #         vehicle.set_autopilot(False)
            #         print("AUTOPILOT OFF")
            #     button_autopilot.set_state_of_button(False)

        


            if button_autopilot.get_state_of_button() is True:
                autopilot_module.start(button_autopilot)
            # print("here")
            if status is 1:
                print("The console stoped :)")
                rospy.signal_shutdown("exit")
            elif status is 0:
                print("You are not longer in autonomous regime")
        if autopilot_flag == False:
            control_car=vehicle.get_control()
            control_car.throttle=data.throttle
            control_car.steer=data.steer
            control_car.reverse=data.is_reverse
            control_car.brake=data.breaks
            vehicle.apply_control(control_car)
        publishing_all_information.publishing(pub,data,vehicle,rate)
        

    except AttributeError:
        return
    finally:
        return

def getInt(data):
    global vehicle_id
    vehicle_id=data.data

# Function that initializes that this script is a subscriber.
def listener():
    global pub
    global rate
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher("/controler_info/full_info",MyRosMsg,queue_size=1)
    rate = rospy.Rate(120)
    rospy.Subscriber("controler_info", MyRosMsg, callback)
    rospy.Subscriber('/vehicle_id',Int16,getInt)
    rospy.spin()

if __name__ == '__main__':
    listener()