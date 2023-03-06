import rospy
import message_filters
import criteria_map

from multicriteria_safety_distance.msg import PointConfidenceStamped
from nav_msgs.msg import OccupancyGrid


class multicriteria_safety_distance:
    def __init__(self):


        self.pointWithConfidence = message_filters.Subscriber('/yolov5/Point_with_confidence', PointConfidenceStamped)
        self.occupancyGrid = message_filters.Subscriber('projected_map', OccupancyGrid)

        self.criterial_map = criteria_map()

        self.synch = message_filters.ApproximateTimeSynchronizer([self.pointWithConfidence, self.occupancyGrid], queue_size=10, slop=0.5)
        self.synch.registerCallback(self.main_callback)

    def main_callback(self, occupancy_msg, point_with_conf_msg):
        
        self.criterial_map.insert_to_map(point_with_conf_msg)


