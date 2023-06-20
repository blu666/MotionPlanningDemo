from map import Map
import numpy as np
import matplotlib.pyplot as plt

map = Map('map2.txt', (0, 0), (19, 19))
print(map.xlim)
print(map.ylim)
print(map.checkState((6, 8)))