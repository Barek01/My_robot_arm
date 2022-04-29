#! /usr/bin/env python

import rospy # Import the Python library for ROS
from std_msgs.msg import Float32 # Import the Float32 message from the std_msgs package
from sensor_msgs.msg import JointState
from tf.broadcaster import TransformBroadcaster 

rospy.init_node('topic_publisher') # Initiate a Node named 'topic_publisher'
pub = rospy.Publisher('/joint_states', JointState, queue_size=1) # Create a Publisher object, that will publish on the /valeur_moteur topic messages of type Float32
rate = rospy.Rate(50) # Set a publish rate of 2 Hz

count = Float32() # Create a var of type Int32
count.data = 0 # Initialize 'count' variable

# Robot state
joint_Base_Axe1 = 0  
joint_Axe1_Axe2 = 0  
joint_Axe2_Axe3 = -2.36
joint_Axe3_Axe4 = 0  
joint_Axe4_Axe5 = 0  
joint_Axe5_Axe6 = 0  

# Message declarations
joint_state = JointState()  

while not rospy.is_shutdown():  # Create a loop that will go until someone stops the program execution
  joint_state.header.stamp = rospy.get_rostime()

  joint_state.name.insert(0,"joint_Base_Axe1")
  joint_state.position.insert(0,joint_Base_Axe1)

  joint_state.name.insert(1,"joint_Axe1_Axe2")
  joint_state.position.insert(1,joint_Axe1_Axe2)

  joint_state.name.insert(2,"joint_Axe2_Axe3")
  joint_state.position.insert(2,joint_Axe2_Axe3)

  joint_state.name.insert(3,"joint_Axe3_Axe4")
  joint_state.position.insert(3,joint_Axe3_Axe4)

  joint_state.name.insert(4,"joint_Axe4_Axe5")
  joint_state.position.insert(4,joint_Axe4_Axe5)

  joint_state.name.insert(5,"joint_Axe5_Axe6")
  joint_state.position.insert(5,joint_Axe5_Axe6)

  # Send the joint state and transform
  pub.publish(joint_state)

  if(joint_Axe1_Axe2 <= -3.1415):
    joint_Axe1_Axe2 = 0

  if(joint_Axe2_Axe3 >= 2.36):
    joint_Axe2_Axe3 = -2.36
  
  joint_Axe1_Axe2 = joint_Axe1_Axe2 - 0.0031415
  joint_Axe2_Axe3 = joint_Axe2_Axe3 + 0.00236

  print("Valeur joint_Axe1_Axe2 : ", joint_Axe1_Axe2)
  print("Valeur joint_Axe1_Axe2 : ", joint_Axe2_Axe3)

  # This will adjust as needed per iteration
  rate.sleep()