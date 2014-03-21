#!/usr/bin/env python
# Plot a graph of Data which is comming in on the fly
# uses pylab
# Author: Norbert Feurle
# Date: 12.1.2012
# License: if you get any profit from this then please share it with me
import pylab
from pylab import *
import roslib; roslib.load_manifest('tf')
import tf
import rospy
import math

rospy.init_node('plotter')
listener = tf.TransformListener()


xAchse=pylab.arange(0,100,1)
yAchse=pylab.array([0]*100)

fig = pylab.figure(1)
ax = fig.add_subplot(111)
ax.grid(True)
ax.set_title("Acceleration")
ax.set_xlabel("Time")
ax.set_ylabel("Amplitude(g)")
ax.axis([0,100,-1.0,1.0])
#line1=ax.plot(xAchse,yAchse,'-', label='x')
#line2=ax.plot(xAchse, yAchse, '-', label='y')
#line3=ax.plot(xAchse, yAchse, '-', label='z')
line4=ax.plot(xAchse, yAchse, '-', label='mag')
legend()
manager = pylab.get_current_fig_manager()

valuesx=[]
valuesx= [0 for x in range(100)]
valuesy=[]
valuesy = [0 for x in range(100)]
valuesz=[]
valuesz = [0 for x in range(100)]
valuesmag=[]
valuesmag = [0 for x in range(100)]
def UpdateValue(arg):
  global valuesx, valuesy
  (trans, rot) = listener.lookupTransform("accelb", "accela", rospy.Time(0))
  valuesx.append(trans[0])
  valuesy.append(trans[1])
  valuesz.append(trans[2])
  valuesmag.append(math.sqrt(math.pow(trans[0],2)+math.pow(trans[1],2)+math.pow(trans[2],2)))

def RealtimePloter(arg):
  global valuesy
  CurrentXAxis=pylab.arange(len(valuesx)-100,len(valuesx),1)
  #line1[0].set_data(CurrentXAxis,pylab.array(valuesx[-100:]))
  #line2[0].set_data(CurrentXAxis,pylab.array(valuesy[-100:]))
  #line3[0].set_data(CurrentXAxis,pylab.array(valuesz[-100:]))
  line4[0].set_data(CurrentXAxis,pylab.array(valuesmag[-100:]))
  ax.axis([CurrentXAxis.min(),CurrentXAxis.max(),0.5,1.2])
  manager.canvas.draw()

timer = fig.canvas.new_timer(interval=200)
timer.add_callback(RealtimePloter, ())
timer2 = fig.canvas.new_timer(interval=200)
timer2.add_callback(UpdateValue, ())
timer.start()
timer2.start()

pylab.show()
