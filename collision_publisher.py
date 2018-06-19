import rospy
from moveit_msgs.msg import DisplayRobotState
from baxter_tests.msg import ContactInformationArray

class Republisher(object):
    def __init__(self):
        self.message = ContactInformationArray()
        rospy.init_node('collision_republisher', anonymous=True)
        self.pub = rospy.Publisher("/collision_contacts_republisher", ContactInformationArray, latch=True, queue_size=0)
        rospy.Subscriber("/robot_collision_contacts", ContactInformationArray, self.callback)

    def callback(self, msg):
        self.message = msg

if __name__ == '__main__':
    repub = Republisher()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        repub.pub.publish(repub.message)
        rate.sleep()
