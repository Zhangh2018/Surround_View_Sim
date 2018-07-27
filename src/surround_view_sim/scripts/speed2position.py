import rospy
from mvp_msgs import WheelSpeedReport, WheelPositionReport

def callback(speed_msg):
    # TODO: integration of speed, and publish position
    return

rospy.init_node('speed2position')
sub = rospy.Subscriber('/vehicle/wheel_speed_report', WheelSpeedReport, callback)
pub = rospy.Publisher('/vehicle/wheel_position_report', WheelPositionReport, 3)
rospy.spin()