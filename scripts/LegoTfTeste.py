#!/usr/bin/env python  
import roslib
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('Lego_Tf')
    print("Running...")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(100.0)

    x = 0 
    y = 0
    z = 0
    roll = pitch = yaw = 0

    while not rospy.is_shutdown():

	# OBS: o formato da transformada em C/C++ 
	# eh diferente (os parametros estao em outra ordem)
        br.sendTransform((x, y, x), # t
                         tf.transformations.quaternion_from_euler(roll, pitch, yaw), # R
                         rospy.Time.now(), # time
                         "lego", # child frame
                         "link0") # parent frame
        rate.sleep()
