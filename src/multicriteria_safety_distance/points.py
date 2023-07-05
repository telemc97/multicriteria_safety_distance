import numpy as np
import math

class Coord:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class Points:
    def __init__(self):
        self.last_idx = 0
        self.looped = False
        self.points = [Coord(0,0,0)]*18

    def insertPoint(self, x, y, z):
        coord = Coord(x,y,z)
        self.points[self.getIdx()] = Coord(x,y,z)
    
    def getIdx(self):
        idx=0
        if (self.last_idx<17):
            self.last_idx+=1
            idx = self.last_idx
        elif (self.last_idx==17):
            self.looped = True
        return idx

    def getAvgSlope(self):
        avg_angle = 0.0
        if (self.last_idx>3 or not self.looped):
            if (not self.looped):
                sum_set = self.last_idx//3
            else:
                sum_set=0
            vec_ref = np.array([0.0,1.0,0.0],dtype=np.float32)
            for i in range(sum_set):
                vec_pq = np.array([self.points[i*3+0].x-self.points[i*3+1].x, self.points[i*3+0].y-self.points[i*3+1].y, self.points[i*3+0].z-self.points[i*3+1].z])
                vec_pr = np.array([self.points[i*3+0].x-self.points[i*3+2].x, self.points[i*3+0].y-self.points[i*3+2].y, self.points[i*3+0].z-self.points[i*3+2].z])
                cross = np.cross(vec_pq, vec_pr)
                magnitude = math.sqrt(math.pow(cross[0],2) + math.pow(cross[1],2) + math.pow(cross[2],2))
                cross = np.array([cross[0]/magnitude, cross[1]/magnitude, cross[2]/magnitude])
                dot = np.dot(cross,vec_ref)
                angle = math.acos(dot)
                avg_angle = (i*avg_angle + angle)/(i+1)
        return avg_angle

