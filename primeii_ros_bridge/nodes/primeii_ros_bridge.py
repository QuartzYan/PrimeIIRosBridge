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

def str2RosMsg(string):
  try:
    jj = json.loads(string)
  except ValueError as ee:
    #rospy.logerr()
    return None
  msg = GlovesData()
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = ''  
  for i in range(len(jj)):
    glove = GloveData()
    gloveNum = 'glove' + str(i+1)
    glove.deviceid = jj[gloveNum]['deviceid']
    glove.dongleid = jj[gloveNum]['dongleid']
    glove.handtype = jj[gloveNum]['handtype']
    glove.wristIMU.x = jj[gloveNum]['wristIMU']['x']
    glove.wristIMU.y = jj[gloveNum]['wristIMU']['y']
    glove.wristIMU.z = jj[gloveNum]['wristIMU']['z']
    glove.wristIMU.w = jj[gloveNum]['wristIMU']['w']
    fingersFlex = jj[gloveNum]['fingers']['fingersFlex']
    for j in range(len(fingersFlex)):
      glove.fingersFlex[j].Joint1Spread = fingersFlex[fingerNames[j]]['Joint1Spread']
      glove.fingersFlex[j].Joint1Stretch = fingersFlex[fingerNames[j]]['Joint1Stretch']
      glove.fingersFlex[j].Joint2Stretch = fingersFlex[fingerNames[j]]['Joint2Stretch']
      glove.fingersFlex[j].Joint3Stretch = fingersFlex[fingerNames[j]]['Joint3Stretch']
    fingersIMU = jj[gloveNum]['fingers']['fingersIMU']
    for j in range(len(fingersIMU)):
      glove.fingersIMU[j].x = fingersIMU[fingerNames[j]]['x']
      glove.fingersIMU[j].y = fingersIMU[fingerNames[j]]['y']
      glove.fingersIMU[j].z = fingersIMU[fingerNames[j]]['z']
      glove.fingersIMU[j].w = fingersIMU[fingerNames[j]]['w']
    msg.glovesData.append(glove)
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
    r = rospy.Rate(200)
    while not rospy.is_shutdown():
      try:
        string = client.recv(4096)
      except:
        rospy.logwarn("read message timeout!!!")    
        continue  
      #print len(string)
      msg = str2RosMsg(string)
      if msg:
        pub.publish(msg)
      else:
        pass
      r.sleep()
    
  t = threading.Thread(target=loop)
  t.start()
  rospy.spin()
  
if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass