import numpy as np
import rospy
import tf
import math
import sys

from geometry_msgs.msg import PointStamped

class criteria_map:
    def __init__(self):
        self.res = rospy.get_param('~res', 1)

        self.detsWeight = rospy.get_param('~detections_weight', 0.4)
        self.avgSlopeWeight = rospy.get_param('~average_slope_weight', 0.3)
        self.densityWeight = rospy.get_param('~average_density', 0.5)
        
        self.maxDecisionCoef = 0
        self.minDecisionCoef = 100000

        self.comparRes = rospy.get_param('~comparison_resolution', 100)

        self.multisampleRes = rospy.get_param('~multisample_resolution', 2) #Multiplies resolution to get more data for slope, avg conf, etc

        criteriadt = np.dtype([('det_sum', np.int16), ('avg_conf', np.float16), ('points', object), ('density', np.float16)])

        self.map = np.zeros(shape=(0,0), dtype=criteriadt)

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
            ##add to points
            if (self.map[x,y]['det_sum']==0):
                self.map[x,y]['points'] = np.asarray(self.get_cantor_hash(self.world_coords), dtype=np.int64)
            else:    
                self.map[x,y]['points'] = np.hstack((self.map[x,y]['points'], np.asarray(self.get_cantor_hash(self.world_coords), dtype=np.int64)))

            ##add to detection sum
            self.map[x,y]['det_sum'] +=1

            ##add to average conf
            if (self.map[x,y]['det_sum']>1):
                self.map[x,y]['avg_conf'] = ((self.map[x,y]['det_sum'] - 1) * (self.map[x,y]['avg_conf']) + point_with_conf_msg.confidence)/(self.map[x,y]['det_sum'])
            else:
                self.map[x,y]['avg_conf'] = point_with_conf_msg.confidence

            ##add density
            self.map[x,y]['density'] = self.map[x,y]['det_sum']/self.res


    def get_average_slope(self, x,y):
        if (self.map[x,y]['det_sum']>3):
            rospy.loginfo(self.map[x,y]['det_sum'])
            sum_set = int((self.map[x,y]['points'].shape[0]//3))
            det_sum = 0
            det_array = np.zeros(shape=(sum_set,3), dtype=np.float16)
            for i in range(sum_set):
                for j in range(3):
                    pointxyz = self.get_points_from_cantor(self.map[x,y]['points'][i*3+j])
                    det_array[j,:] = pointxyz
                det = self.det_calc(det_array)
                det_sum += det
            avg_det = det_sum/sum_set
            return avg_det
        else:
            avg_det = 0
            return avg_det


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
        new_point = True
        cantorPoint = self.get_cantor_hash(point)
        detectedPoints = self.map[x, y]['points']
        it = np.nditer(detectedPoints, op_flags=['readonly'])
        for x in it:
            if (self.get_cantor_hash(x)==cantorPoint): new_point = False
            rospy.loginfo('Point: %f, %f, %f already detected' % (point[0], point[1], point[2]))
        return new_point

    
    def get_cantor_hash(self, point) -> np.uint64:
        newPoints = np.zeros(shape=(3), dtype=np.int64)
        for i in range(3):
            if (0>point[i]):
                negative=1
                point[i] = abs(point[i])
            else:
                negative=0
            meters = np.uint64(point[i] - point[i]%1)
            centimeters = np.uint64(point[i]%1*10)
            newPoints[i] = np.uint64((negative*10000) + (meters*10) + centimeters)
        print(newPoints)
        a = np.uint64(((newPoints[0] * newPoints[0]) + newPoints[0] + 2*newPoints[0]*newPoints[1] + 3*newPoints[1] + (newPoints[1] * newPoints[1]))/2)
        cHash = np.uint64(((a * a) + a + 2*a*newPoints[2] + 3*newPoints[2] + (newPoints[2] * newPoints[2]))/2)
        return cHash


    def get_points_from_cantor(self, cantor: np.uint64):
        w = np.uint64(math.floor((math.sqrt(8*cantor+1)-1)/2))
        t = np.uint64(((w * w)+w)/2)
        z = np.uint64(cantor - t)
        xy = np.uint64(w - z)
        xyw = np.uint64(math.floor((math.sqrt(8*xy+1)-1)/2))
        xyt = np.uint64(((xyw * xyw)+xyw)/2)
        y = np.uint64(xy - xyt)
        x = np.uint64(xyw - y)
        newPoints = np.array([x,y,z], dtype=np.int64)
        point = np.zeros(shape=(3), dtype=np.float32)
        for i in range(3):
            if ((newPoints[i]//10000) == 0):
                meters = np.uint64((newPoints[i]/10)//1)
                centimeteres = np.uint64(newPoints[i]%10)
                point[i] = meters + centimeteres/10
            else:
                meters = np.uint64(((newPoints[i]-10000)/10)//1)
                centimeteres = np.uint64(newPoints[i]%10)
                point[i] = (meters + centimeteres/10)*-1
        return point
    

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
            detSum = self.map[point[0],point[1]]['det_sum']
            avgSlope = self.get_average_slope(point[0],point[1])
            avgConf = self.map[point[0],point[1]]['avg_conf']
            density = self.map[point[0],point[1]]['density']
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
    
    def det_calc(self, array):
        array = array*100
        array = np.asarray(array, dtype=np.int32)
        det_ = np.linalg.det(array)
        return det_