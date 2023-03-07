import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import rospy

class numpy_occupancyGrid:
    def __init__(self):

        self.occupancyGrid = np.zeros(shape=(0,0), dtype=np.int8)

        self.origin = np.zeros(shape=(3), dtype=np.int8)

        self.gridPub = rospy.Publisher('projected_map_with_safety_dist', OccupancyGrid, queue_size=10)

        self.header = Header()
        self.mapInfo = MapMetaData()


    def to_matrix(self, occupancy_msg):
        cols = occupancy_msg.info.height
        rows = occupancy_msg.info.width
        self.header = occupancy_msg.header
        self.mapInfo = occupancy_msg.info
        self.origin = occupancy_msg.info.origin.x, occupancy_msg.info.origin.y, occupancy_msg.info.origin.z
        self.occupancyGrid = np.resize(self.occupancyGrid, (rows, cols))
        for i in range(len(occupancy_msg.data)):
            index = self.get_index_from_linearIndex(rows, i)
            self.occupancyGrid[index[0],index[1]] = occupancy_msg.data[i]


    def to_occupancy_grid(self):
        occupancyGridOutput = OccupancyGrid()
        occupancyGridOutput.header = self.header
        occupancyGridOutput.info = self.mapInfo
        occupancyGridOutput.info.width = self.occupancyGrid.shape[0]
        occupancyGridOutput.info.height =self.occupancyGrid.shape[1]
        it = np.nditer(self.occupancyGrid, flags=['readonly'], order='F')
        for x in it:
            occupancyGridOutput.data.append(x)
        self.gridPub.publish(occupancyGridOutput)

    
    def from_matrix_to_real_world(self, point):
        rw_point = np.zeros(shape=3, dtype=np.int8)
        rw_point[0] = point[0]+self.origin[0]
        rw_point[1] = point[1]+self.origin[1]
        rw_point[2] = point[2]+self.origin[2]
        return rw_point


    def from_real_world_to_matrix(self, rw_point):
        point = np.zeros(shape=3, dtype=np.int8)
        point[0] = rw_point[0]-self.origin[0]
        point[1] = rw_point[1]-self.origin[1]
        point[2] = rw_point[2]-self.origin[2]
        return point


    def get_index_from_linearIndex(self, rows, index):
        y = index/rows
        assert(type(y)==int)
        x = index-(y*rows)
        assert(type(x)==int)
        return x,y
    

    def get_linearIndex_from_index(self, height, x, y):
        linearIndex = x * height + y
        return linearIndex
