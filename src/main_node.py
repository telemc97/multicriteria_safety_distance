import rospy
import numpy as np
import message_filters
from criteria_map import criteria_map
from occupancyGrid_to_numpy import numpy_occupancyGrid

from multicriteria_safety_distance.msg import PointConfidenceStamped
from nav_msgs.msg import OccupancyGrid


class multicriteria_safety_distance:
    def __init__(self):


        self.pointWithConfidence = message_filters.Subscriber('/yolov5/Point_with_confidence', PointConfidenceStamped)
        self.occupancyGrid = message_filters.Subscriber('projected_map', OccupancyGrid)

        self.criterialMap = criteria_map()
        self.numpyOccupancyGrid = numpy_occupancyGrid()

        self.synch = message_filters.ApproximateTimeSynchronizer([self.pointWithConfidence, self.occupancyGrid], queue_size=10, slop=0.5)
        self.synch.registerCallback(self.main_callback)

    def main_callback(self, occupancy_msg, point_with_conf_msg):

        self.numpyOccupancyGrid.to_matrix(occupancy_msg)
        self.criterialMap.insert_to_map(point_with_conf_msg)

        numpyOccupancyGrid = self.numpyOccupancyGrid.occupancyGrid

        it = np.nditer(numpyOccupancyGrid, op_flags=['multi_index'])
        for x in it:
            if (x == 100):
                rw_point = self.numpyOccupancyGrid.from_matrix_to_real_world(it.multi_index)
                decision = self.criterialMap.get_multicriterial_coef(rw_point)
                rospy.loginfo(decision)
        self.numpyOccupancyGrid.to_occupancy_grid
        

if __name__ == "__main__":    
    rospy.init_node("multicriteria_safety_distance", anonymous=True)
    multicriteria_safety_distance()
    rospy.spin()