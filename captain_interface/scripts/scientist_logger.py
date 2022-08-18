import rospy
from std_msgs.msg import String
import numpy as np
import time
import os


ssdPath ='/tmp/' #'/home/mrl/Documents/python_logger/Test_logs/'

counterDataFile = 0
fileMaxSizeReached = False

dayFolder = time.strftime("%Y%m%d")
print(dayFolder)
actualDayFolder = ssdPath+dayFolder+'/'
if not os.path.exists(actualDayFolder):
    os.mkdir(os.path.abspath(actualDayFolder))
    print("Created day folder")


timeNow = time.gmtime()
timeFolder = time.strftime("%H%M%S",timeNow)

actualFolder = actualDayFolder+timeFolder+'/'
if not os.path.exists(actualFolder):
    os.mkdir(os.path.abspath(actualFolder))
    print("Created Time folder")

if not os.path.exists(actualFolder+'MISSION'):
    fidMission = open(os.path.abspath(actualFolder+'MISSION'),'a')
    fidMission.close()
    print("Created mission log file")

if not os.path.exists(actualFolder+'DATA'+str(counterDataFile)):
    fidData = open(os.path.abspath(actualFolder+'DATA'+str(counterDataFile)),'a')
    fidData.close()
    print("Created new data log file, number =" + str(counterDataFile))

def logMissionCallback(msg):
    fidMission = open(os.path.abspath(actualFolder+'MISSION'),'a')
    fidMission.write(msg.data)
    fidMission.close()

def logDataCallback(data):
    global counterDataFile
    global fileMaxSizeReached
    if os.path.exists(os.path.abspath(actualFolder+'DATA'+str(counterDataFile))):
        if os.path.getsize(os.path.abspath(actualFolder+'DATA'+str(counterDataFile)))>(16 * 1024 * 1024) :
            fileMaxSizeReached = True
    fidData = open(os.path.abspath(actualFolder+'DATA'+str(counterDataFile)),'a')
    fidData.write(data.data)
    fidData.close()
    if fileMaxSizeReached:
        if data.data[-1] == '\n':
            counterDataFile +=1
            fileMaxSizeReached = False
            print("Created new data log file, number =" + str(counterDataFile))
    


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('logger_node', anonymous=True)
    subMissionLog = rospy.Subscriber('lolo/log/mission', String, logMissionCallback)
    subDataLog = rospy.Subscriber('lolo/log/data', String, logDataCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
	listener()
