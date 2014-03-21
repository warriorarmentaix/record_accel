#!/usr/bin/env/python

import argparse
import rosbag
from pylab import *
import math

parser = argparse.ArgumentParser()
parser.add_argument("demo_name", type=str)
args = parser.parse_args()


bag = rosbag.Bag('data/'+args.demo_name+'.bag')


x = []
y = []
z = []
mag = []
for topic, msg, t in bag.read_messages(topics=['accel']):
    x.append(-msg.x)
    y.append(msg.y)
    z.append(msg.z)
    mag.append(math.sqrt(math.pow(msg.x,2) + math.pow(msg.y,2) + math.pow(msg.z,2)))

axis = range(len(x))

title(args.demo_name)
xlabel("Time")
ylabel("Amplitude(g)")
plot(axis, x, label='x')
plot(axis, y, label='y')
plot(axis, z, label='z')
plot(axis, mag, label='mag')
legend()
show()

bag.close()
