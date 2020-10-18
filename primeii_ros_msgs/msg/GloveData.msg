#
uint32 deviceid
uint32 dongleid
uint32 handtype #0:Unknown, 1:LeftHand, 2:RightHand

FingerFlex[5] fingersFlex
geometry_msgs/Quaternion[5] fingersIMU
geometry_msgs/Quaternion wristIMU