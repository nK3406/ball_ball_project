import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

class JointPublisher:
    def __init__(self):
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self.ball_pose_sub = rospy.Subscriber("/ball_pose", Point, self.ball_pose_cb)
        self.rate = rospy.Rate(10) # 10hz
        self.joint_state = JointState()
        self.joint_state.header = Header()
        self.joint_state.name = ['ara_bag_j', 'ust_tabla_j']
        self.joint_state.position = [1.0, 1.0]
        self.ball_position = Point()

    def ball_pose_cb(self, msg):
        self.ball_position = msg # save the ball position message
        # here you can do something with the ball position message

    def publish_joints(self):
        while not rospy.is_shutdown():
            self.joint_state.header.stamp = rospy.Time.now()
            self.joint_state.position = [self.ball_position.x, self.ball_position.y] # set the position of the joints according to the ball position
            self.joint_pub.publish(self.joint_state)
            self.rate.sleep()

def main():
    rospy.init_node('joint_publisher')
    jp = JointPublisher()
    jp.publish_joints()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
