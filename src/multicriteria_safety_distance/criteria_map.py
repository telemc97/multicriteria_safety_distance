import numpy as np
import rospy
import tf
import math
import sys

from geometry_msgs.msg import PointStamped

from multicriteria_safety_distance.points import Points

class CriteriaMap:
    def __init__(self):
        self.res = rospy.get_param('~res', 1)

        self.detsWeight = rospy.get_param('~detections_weight', 0.4)
        self.avgSlopeWeight = rospy.get_param('~average_slope_weight', 0.3)
        self.densityWeight = rospy.get_param('~average_density', 0.5)
        
        self.maxDecisionCoef = 0
        self.minDecisionCoef = 100000

        self.multisampleRes = rospy.get_param('~multisample_resolution', 2) #Multiplies resolution to get more data for slope, avg conf, etc
        
        #Dictionary with Key: coord (converted to one int with cantor pairing function) and Value: amount of total detections (int)
        self.det_sum = {}

        #Dictionary with Key: coord (converted to one int with cantor pairing function) and Value: avg confidence (float)
        self.avg_conf = {}

        #Dictionary with Key: coord (converted to one int with cantor pairing function) and Value: detection density (float)
        self.density = {}

        #Dictionary with Key: coord (converted to one int with cantor pairing function) and Value: coordinates of detections (object coords)
        self.points = {}


        self.map = np.zeros(shape=(0,0), dtype=bool)

        self.origin = np.zeros(shape=(2), dtype=np.int8)
        self.world_coords = np.zeros(shape=(3), dtype=np.float16)

        self.listener = tf.TransformListener()


    def insert_to_map(self, point_with_conf_msg):

        point_msg = PointStamped()
        point_msg.header.stamp = point_with_conf_msg.header.stamp
        point_msg.header.frame_id = point_with_conf_msg.header.frame_id
        point_msg.point.x = point_with_conf_msg.point.x
        point_msg.point.y = point_with_conf_msg.point.y
        point_msg.point.z = point_with_conf_msg.point.z

        transform_ok = False
        while not transform_ok and not rospy.is_shutdown():
            try:
                point_in_world = self.listener.transformPoint('map', point_msg)
                transform_ok = True
            except tf.ExtrapolationException as e:
                rospy.logwarn("Exception on transforming pose... trying again \n(" + str(e) + ")")
                rospy.sleep(0.1)
                point_msg.header.stamp = self.listener.getLatestCommonTime('stereo_cam', 'map')
        
        self.world_coords[0:] = point_in_world.point.x, point_in_world.point.y, point_in_world.point.z

        # rospy.loginfo('Point coordinates: %f, %f' %(self.world_coords[0], self.world_coords[1]))
        
        x,y = self.point_to_index(self.world_coords)
        newPoint = self.new_point(x,y, self.world_coords)

        if (newPoint):
            ##add to detection sum
            self.det_sum[x,y]+=1

            ##add to points
            if (self.det_sum[x,y]==0):
                self.points[x,y]=Points()
            self.points[x,y].insertPoint(point_with_conf_msg.point.x, point_with_conf_msg.point.y, point_with_conf_msg.point.z)

            ##add to average conf ToDo check again
            self.avg_conf[x,y] = ((self.det_sum[x,y] - 1) * (self.avg_conf[x,y]) + point_with_conf_msg.confidence)/(self.det_sum[x,y])

            ##add density
            self.density[x,y] = self.det_sum[x,y]/self.res


    def point_to_index(self, point):
        """
        Takes the Point and converts it to matrix index. 
        It also updates the origin if the value is negative 
        or extends it if the value is greater than the matrix.
        The function returns the (x,y).
        """
        if (point[0]/self.res*self.multisampleRes<0):
            x = 0
            self.origin[0] = -int(round(point[0]/self.res*self.multisampleRes)<0)
        else:
            x = int(point[0]/(self.res*self.multisampleRes))

        if (point[1]/self.res*self.multisampleRes<0):
            y = 0
            self.origin[1] = -int(round(point[1]/self.res*self.multisampleRes)<0)
        else:
            y = int(point[1]/(self.res*self.multisampleRes))

        if (x>=self.map.shape[0]):
            newWidth = (x+1)
        else:
            newWidth = self.map.shape[0]
        if (y>=self.map.shape[1]):
            newHeight = (y+1)
        else:
            newHeight = self.map.shape[1]
        temp_map = self.map
        self.map = np.zeros(shape=(newWidth,newHeight), dtype=temp_map.dtype)
        self.map[:temp_map.shape[0], :temp_map.shape[1]] = temp_map
        return x, y
        

    def new_point(self, x, y, point):
        if (self.map[x, y]['det_sum']==0):
            new_point = True
        else:
            cantorPoint = self.get_cantor_hash(point)
            it = np.nditer(self.map[x, y]['points'], op_flags=['readonly'])
            new_point = True
            for k in it:
                if (k == cantorPoint): new_point = False
                rospy.loginfo('Point: %f, %f, %f already detected' % (point[0], point[1], point[2]))
        return new_point
    

    def coord_in_map(self, point):
        """
        Converts a real world coordinate (pipeline's resolution) in a coordinate in the criteria matrix
        """
        map_x = int(point[0]/(self.res*self.multisampleRes)) - self.origin[0]
        map_y = int(point[1]/(self.res*self.multisampleRes)) - self.origin[1]
        if (map_x>=self.map.shape[0] or map_y>=self.map.shape[1]):
            rospy.logwarn('Index: %i, %i is not in the matrix' % (map_x, map_y))
            return -1
        else:
            return map_x, map_y


    def get_multicriterial_coef(self, occupancy_point):
        """
        Point in real world's coordinates (pipeline's resolution)
        """
        point = self.coord_in_map(occupancy_point)
        if (point!=-1):
            detSum = self.det_sum[point[0],point[1]]
            avgSlope = self.points[point[0],point[1]].getAvgSlope()
            avgConf = self.avg_conf[point[0],point[1]]
            density = self.density[point[0],point[1]]
            decisionCoef = self.detsWeight*(detSum*avgConf) + self.avgSlopeWeight*avgSlope + self.densityWeight*density
            # rospy.loginfo('decision coef: %f' % decisionCoef)
            if (decisionCoef>self.maxDecisionCoef):
                self.maxDecisionCoef = decisionCoef
            elif (decisionCoef<self.minDecisionCoef):
                self.minDecisionCoef = decisionCoef
            if ((self.maxDecisionCoef-self.minDecisionCoef) != 0):
                normalizedValue = (decisionCoef-self.minDecisionCoef)/(self.maxDecisionCoef-self.minDecisionCoef)
            else:
                normalizedValue = 0
        else:
            normalizedValue = 0

        return normalizedValue