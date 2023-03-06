import numpy as np

class occupancyGrid_to_numpy:
    def __init__(self):

        self.occupancyGrid = np.zeros(shape=(0,0), dtype=np.int8)

        self.offest = np.zeros(shape=(2), dtype=np.int8)


    def to_matrix(self, )



    def get_index_from_linearIndex(self, rows, index):
        y = index/rows
        assert(type(y)==int)
        x = index-(y*rows)
        assert(type(x)==int)
        return x,y
    
    def get_linearIndex_from_index(self, height, x, y):
        linearIndex = x * height + y
        return linearIndex
