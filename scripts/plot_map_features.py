#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

import sys
DATA_PATH = '/home/maestro/catkin_ws/src/pf_localization/Data'
MODULES_PATH = '/home/maestro/catkin_ws/src/pf_localization/scripts'
sys.path.insert(0, MODULES_PATH)

from pf_classes import *

//TODO: Proveriti zasto nece da se pokrene mapa stage_1
MAP_NAME = DATA_PATH + '/stage_1_test'

map = Map(MAP_NAME)
map.extract_objects()
map.save_objects()

print('*************')
print('Map: ' + map.name)
print('*************')
print('Objects: ')

plt.figure(1)

for w in map.walls:
    plt.plot(w.x, w.y, label = w.name, lw = 2)
    print(w.name)
    print('   (x,y) : (%.3f,%.3f) => (%.3f,%.3f) [m]' % (np.min(w.x), np.min(w.y), np.max(w.x), np.max(w.y)))

plt.title('Map "' + map.name + '" in X-Y plane')
plt.xlabel('x[m]')
plt.ylabel('y[m]')
plt.xlim(-3,3)
plt.ylim(-3,3)
plt.legend()

plt.show()
