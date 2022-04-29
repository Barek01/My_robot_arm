import rospy 
from std_msgs.msg import Float32 
from sensor_msgs.msg import JointState
from tf.broadcaster import TransformBroadcaster 
from my_webpage.msg import position_moteurs
import math

rospy.init_node('exchange_loop')
pub = rospy.Publisher('/joint_states', JointState, queue_size=1) # Create a Publisher object, that will publish on topic messages

# Message declarations
joint_state = JointState()  

def callback(msg):

    joint_state.header.stamp = rospy.get_rostime()

    joint_state.name = msg.name

    joint_state.position = []

    for position in msg.position:
        joint_state.position.append(math.radians(position))

    pub.publish(joint_state)

sub = rospy.Subscriber('/web_mvt', position_moteurs, callback)
rospy.spin()








