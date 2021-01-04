#!/usr/bin/env python

from pf_classes import *

import numpy as np
import matplotlib.pyplot as plt

MAP_NAME = 'stage_1'

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
