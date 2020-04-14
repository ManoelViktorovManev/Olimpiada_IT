from msg_folder.msg import MyRosMsg
def publishing(pub,data,vehicle,rate):
    msg=MyRosMsg()
    msg.throttle=data.throttle
    msg.steer=data.steer
    msg.breaks=data.breaks
    msg.is_autopilot=data.is_autopilot
    msg.is_reverse=data.is_reverse
    msg.is_exit=data.is_exit
    msg.vehicle_id=vehicle.id
    msg.vehicle_name=vehicle.type_id
    pub.publish(msg)
    rate.sleep()