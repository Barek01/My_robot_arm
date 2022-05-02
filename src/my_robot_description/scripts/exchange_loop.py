import rospy 

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from my_webpage.msg import position_moteurs
import math

rospy.init_node('exchange_loop')
pub = rospy.Publisher('/joint_position_cmd', JointTrajectoryPoint, queue_size=1) # Create a Publisher object, that will publish on topic messages

# Message declarations

Joint_point = JointTrajectoryPoint()

def callback(msg):

    #Joint_traj.header.stamp = rospy.get_rostime()

    #Joint_traj.joint_names = msg.name

    Joint_point.positions = []
    Joint_point.velocities = []

    #for position in msg.position:
    #    Joint_point.positions.append(math.radians(position))

    Joint_point.positions.append(math.radians(msg.position[0]))
    Joint_point.velocities.append(0.1)

    #Joint_traj.points = Joint_point

    pub.publish(Joint_point)
    rospy.loginfo('Je publie dans joint_position_cmd')

sub = rospy.Subscriber('/web_mvt', position_moteurs, callback)
rospy.spin()








