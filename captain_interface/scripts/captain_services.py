#!/usr/bin/python
import rospy

from std_srvs.srv import SetBool, SetBoolResponse
from lolo_msgs.msg import CaptainService
import random
import time
from threading import Thread

#from scientistmsg.h
SERVICE_CONTROLLER_WAYPOINT  = 210 
SERVICE_CONTROLLER_PITCH     = 211
SERVICE_CONTROLLER_ROLL      = 212
SERVICE_CONTROLLER_YAW       = 213
SERVICE_CONTROLLER_YAWRATE   = 214
SERVICE_CONTROLLER_DEPTH     = 215
SERVICE_CONTROLLER_ALTITUDE  = 216
SERVICE_CONTROLLER_SPEED     = 217

SERVICE_ACTION_FAIL     = 0
SERVICE_ACTION_SUCCESS  = 1
SERVICE_ACTION_DISABLE  = 0
SERVICE_ACTION_ENABLE   = 1
                   


class srvHandler:

  def __init__(self, request_id):
    self.request_id = request_id;
    self.reply_msg = None
    self.sub = rospy.Subscriber("/lolo/core/captain_srv_out", CaptainService, self.lolo_reply_cb)
    self.pub = rospy.Publisher("/lolo/core/captain_srv_in", CaptainService, queue_size=1)

  def lolo_reply_cb(self,msg):
    self.reply_msg = msg;

  def srv_cb(self,msg):
    """
    Callback for the SetBool service
    """

    #Send request to the captain
    ref = random.randrange(0,32000)
    request = CaptainService()
    request.id = self.request_id
    request.ref = ref
    request.action = SERVICE_ACTION_ENABLE if msg.data else SERVICE_ACTION_DISABLE
    
    #set reply message to None
    self.reply_msg = None
    
    #publish request
    self.pub.publish(request);

    #Wait for lolo to reply
    before =time.time();
    while (time.time() - before ) < 1 and not rospy.is_shutdown(): # 1s timeout
      while (time.time() - before ) < 1 and not rospy.is_shutdown() and self.reply_msg is None:
        rospy.rostime.wallsleep(0.01)
      if(not self.reply_msg is None):
        if(self.reply_msg.ref == ref):
          #reply to our request received
          print("Reply to request received with the correct reference")
          if self.reply_msg.reply == SERVICE_ACTION_SUCCESS:
            return SetBoolResponse(success=True, message="it probably worked")
          else:
            return SetBoolResponse(success=False, message="it probably failed")
      self.reply_msg = None
      
    #timeout
    print("timeout :(")
    return SetBoolResponse(success=False, message="it probably failed")

def main():

  random.seed()
  print("Starting")
  rospy.init_node("lolo_captain_services")

  #Waypoint
  handler_waypoiny = srvHandler(SERVICE_CONTROLLER_WAYPOINT)
  service_waypoint = rospy.Service('/lolo/ctrl/toggle_onboard_waypoint_ctrl', SetBool, handler_waypoiny.srv_cb)
  
  #Pitch
  handler_pitch = srvHandler(SERVICE_CONTROLLER_PITCH)
  service_pitch = rospy.Service('/lolo/ctrl/toggle_onboard_pitch_ctrl', SetBool, handler_pitch.srv_cb)
  
  #roll
  #handler_roll = srvHandler(SERVICE_CONTROLLER_ROLL)
  #service_roll = rospy.Service('/lolo/ctrl/toggle_onboard_roll_ctrl', SetBool, handler_roll.srv_cb)
  
  #Yaw
  handler_yaw = srvHandler(SERVICE_CONTROLLER_YAW)
  service_yaw = rospy.Service('/lolo/ctrl/toggle_onboard_heading_ctrl', SetBool, handler_yaw.srv_cb)

  #Yawrate
  handler_yawrate = srvHandler(SERVICE_CONTROLLER_YAWRATE)
  service_yawrate = rospy.Service('/lolo/ctrl/toggle_onboard_yawrate_ctrl', SetBool, handler_yawrate.srv_cb)

  #Depth
  handler_depth = srvHandler(SERVICE_CONTROLLER_DEPTH)
  service_depth = rospy.Service('/lolo/ctrl/toggle_onboard_depth_ctrl', SetBool, handler_depth.srv_cb)

  #Altitude
  handler_altitude = srvHandler(SERVICE_CONTROLLER_ALTITUDE)
  service_altitude = rospy.Service('/lolo/ctrl/toggle_onboard_altitude_ctrl', SetBool, handler_altitude.srv_cb)

  #Speed
  handler_speed = srvHandler(SERVICE_CONTROLLER_SPEED)
  service_speed = rospy.Service('/lolo/ctrl/toggle_onboard_speed_ctrl', SetBool, handler_speed.srv_cb)

  rospy.spin()

if __name__ == "__main__":
  main()
