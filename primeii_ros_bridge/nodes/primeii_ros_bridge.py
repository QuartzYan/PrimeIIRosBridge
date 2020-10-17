#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import time
import json
import socket 
import threading

import rospy 
from primeii_ros_msgs.msg import FingerFlex, GloveData, GlovesData

sub = None
pub = None

fingerNames = [ "thumb", "index", "middle", "ring", "pinky" ]

def callBack(msg):
  pass

def str2RosMsg(str):
  msg = GlovesData()
  jj = json.loads(str)
  for i in range(len(jj)):
    gloveNum = 'glove' + str(i)
    jj[gloveNum]['deviceid']
    jj[gloveNum]['dongleid']
    jj[gloveNum]['handtype']
    jj[gloveNum]['wristIMU']['x']
    jj[gloveNum]['wristIMU']['y']
    jj[gloveNum]['wristIMU']['z']
    jj[gloveNum]['wristIMU']['w']
    fingersFlex = jj[gloveNum]['fingers']['fingersFlex']
    for j in range(len(fingersFlex)):
      fingersFlex[fingerNames[j]]['Joint1Spread']
      fingersFlex[fingerNames[j]]['Joint1Stretch']
      fingersFlex[fingerNames[j]]['Joint2Stretch']
      fingersFlex[fingerNames[j]]['Joint3Stretch']
    fingersIMU = jj[gloveNum]['fingers']['fingersIMU']
    for j in range(len(fingersIMU)):
      fingersIMU[fingerNames[j]]['x']
      fingersIMU[fingerNames[j]]['y']
      fingersIMU[fingerNames[j]]['z']
      fingersIMU[fingerNames[j]]['w']
  
  return msg

def main():
  global sub, pub
  rospy.init_node("primeii_ros_bridge")
  #get param
  hostname = rospy.get_param("~hostname", default="192.168.3.141")
  hostport = rospy.get_param("~hostport", default=10086)

  #init Subscriber and Publisher
  #sub = rospy.Subscriber("obj_point", PointStamped, callBack)
  pub = rospy.Publisher("GlovesData", GlovesData, queue_size=1)

  client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  client.settimeout(5)
  while not rospy.is_shutdown():
    er = client.connect_ex((hostname, hostport))
    if er == 0:
      rospy.loginfo("connect %s successful...", hostname)
      break
    else:
      rospy.loginfo("connect faild, error num:%d", er)
      rospy.loginfo("try connect %s again", hostname)
      time.sleep(1)
    

  def loop():
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
      string = client.recv(4096)
      msg = str2RosMsg(string)
      pub.publish(msg)
      r.sleep()
    
  t = threading.Thread(target=loop)
  t.start()
  rospy.spin()
  
if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass