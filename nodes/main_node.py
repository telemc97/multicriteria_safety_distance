import rospy
from multicriteria_safety_distance.main import multicriteria_safety_distance

if __name__ == "__main__":    
    rospy.init_node("multicriteria_safety_distance", anonymous=True)
    multicriteria_safety_distance()
    rospy.spin()