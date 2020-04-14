#!/usr/bin/env python
import pygame
import rospy
import carla
import waypoints
import autopilot
import PID_Controller
import detecting_actors
from msg_folder.msg import MyRosMsg
import time
import threading

class Autopilot:
    def __init__(self,vehicle,pid_controller=None,my_radar=None,creating_road=None):
        self.vehicle=vehicle
        if pid_controller == None:
            control_throttle=PID_Controller.PID_Controller_Throttle(vehicle,30)
            control_steer=PID_Controller.PID_Controller_Steer(vehicle)
            self.pid_controller=PID_Controller.Controller(vehicle,control_throttle,control_steer)
        else:
            self.pid_controller=pid_controller
        if my_radar == None:
            self.my_radar=detecting_actors.MyRadar(vehicle,15)
        else:
            self.my_radar=my_radar
        if creating_road == None:
            self.creating_road=waypoints.Creating_Path(vehicle)
        else:
            self.creating_road=creating_road
        self.publisher=rospy.Publisher("/controler_info/full_info",MyRosMsg,queue_size=1)
        self.button=None
        # self.subscriber=rospy.Subscriber("controler_info", MyRosMsg, self.callback)
        # rospy.spin()
        
        
    def callback(self,data):
        print("Callbacka maina")
        # if data.is_autopilot==True:
            # print("Butoon is pressed")


    def publish_MyRosMsg(self,vehicle_control):
        msg=MyRosMsg()
        msg.throttle=vehicle_control.throttle
        msg.steer=vehicle_control.steer
        msg.breaks=vehicle_control.brake
        msg.is_reverse=False
        msg.is_autopilot=False
        msg.is_exit=False
        msg.vehicle_id=self.vehicle.id
        msg.vehicle_name=self.vehicle.type_id
        self.publisher.publish(msg)

# Number of nodes: 69
# Number of edges: 154

    def start(self,button=None):
        self.button=button
        # self.subscriber=rospy.Subscriber("controler_info", MyRosMsg, self.callback)
        #
        thread1= threading.Thread(name='create_path_to_closesest_croassroad', target=self.creating_road.create_path_to_closest_crossroad())
        thread2= threading.Thread(name='create_shortest_path', target=self.creating_road.create_path())
        thread1.start()
        thread2.start()
        thread1.join()
        thread2.join()
        self.creating_road.printing_path()
        while True:     
            try:
                if self.creating_road.current_waypoint is None or self.creating_road.compare_waypoint_and_vehicle(self.creating_road.current_waypoint) < 1:

                    self.creating_road.current_waypoint=self.creating_road.getting_waypoint(self.creating_road.waypoints_from_the_path)
                    self.creating_road.waypoints_from_the_path.remove(self.creating_road.waypoints_from_the_path[0])

                if len(self.creating_road.waypoints_from_the_path)<10:
                    thread1= threading.Thread(name='create_path_to_closesest_croassroad', target=self.creating_road.create_path())
                    thread1.start()
                    thread1.join()
                    self.creating_road.printing_path()

                # actors=self.my_radar.get_only_front_actors(self.creating_road.waypoints_from_the_path[0:10])

                vehicle_control=carla.VehicleControl()
                self.pid_controller.set_speed_on_car_and_steer(self.creating_road.current_waypoint)
                vehicle_control.throttle=self.pid_controller.throttle
                vehicle_control.steer=self.pid_controller.steer
                vehicle_control.brake=self.pid_controller.breaks
                self.vehicle.apply_control(vehicle_control)
                self.publish_MyRosMsg(vehicle_control)

            except (IndexError,KeyboardInterrupt) as exception:
                break