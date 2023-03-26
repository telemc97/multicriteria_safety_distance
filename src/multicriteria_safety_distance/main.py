#!/usr/bin/env python3

import rospy
import numpy as np
import message_filters
from multicriteria_safety_distance.criteria_map import criteria_map
from multicriteria_safety_distance.occupancyGrid_to_numpy import numpy_occupancyGrid

from multicriteria_safety_distance.msg import PointConfidenceStamped
from nav_msgs.msg import OccupancyGrid


class multicriteria_safety_distance:
    def __init__(self):


        self.pointWithConfidence = message_filters.Subscriber('/yolov5/point_with_confidence', PointConfidenceStamped)
        self.occupancyGrid = message_filters.Subscriber('projected_map', OccupancyGrid)

        self.minSafetyRad = rospy.get_param('~min_safety_radius', 0)
        self.maxSafetyRad = rospy.get_param('~max_safety_radius', 3)

        self.criterialMap = criteria_map()
        self.numpyOccupancyGrid = numpy_occupancyGrid()

        self.synch = message_filters.ApproximateTimeSynchronizer([self.pointWithConfidence, self.occupancyGrid], queue_size=10, slop=0.5)
        self.synch.registerCallback(self.main_callback)

    def main_callback(self, point_with_conf_msg, occupancy_msg):

        self.criterialMap.insert_to_map(point_with_conf_msg)
        self.numpyOccupancyGrid.to_matrix(occupancy_msg)
        shape = (2*self.maxSafetyRad+self.numpyOccupancyGrid.occupancyGrid.shape[0]),  (2*self.maxSafetyRad+self.numpyOccupancyGrid.occupancyGrid.shape[1])
        tempNumpyOccupancyGrid = np.zeros(shape, dtype=np.int8)
        tempNumpyOccupancyGrid[self.maxSafetyRad:self.maxSafetyRad+self.numpyOccupancyGrid.occupancyGrid.shape[0], self.maxSafetyRad:self.maxSafetyRad+self.numpyOccupancyGrid.occupancyGrid.shape[1]] = self.numpyOccupancyGrid.occupancyGrid
        it = np.nditer(self.numpyOccupancyGrid.occupancyGrid, flags=['multi_index'])
        for x in it:
            if (x == 100):
                rw_point = self.numpyOccupancyGrid.from_matrix_to_real_world(it.multi_index)
                decision = self.criterialMap.get_multicriterial_coef(rw_point)
                index_w_offset = it.multi_index[0]+self.maxSafetyRad, it.multi_index[1]+self.maxSafetyRad
                safetyRadius = self.get_safety_radius(decision)
                tempNumpyOccupancyGrid[index_w_offset[0]-safetyRadius:index_w_offset[0]+safetyRadius+1, index_w_offset[1]-safetyRadius:index_w_offset[1]+safetyRadius+1] = 100
        self.numpyOccupancyGrid.occupancyGrid = tempNumpyOccupancyGrid[self.maxSafetyRad:self.maxSafetyRad+self.numpyOccupancyGrid.occupancyGrid.shape[0], self.maxSafetyRad:self.maxSafetyRad+self.numpyOccupancyGrid.occupancyGrid.shape[1]]
        self.numpyOccupancyGrid.to_occupancy_grid()

    def get_safety_radius(self, decision_):
        return self.minSafetyRad + int(decision_*(self.maxSafetyRad-self.minSafetyRad))