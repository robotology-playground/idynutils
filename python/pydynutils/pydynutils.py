import yarp
import numpy as np
import PyKDL as kdl

def numPyToFrame(mat):
    v = kdl.Vector(mat[0,3],mat[1,3],mat[2,3])
    r = kdl.Rotation()
    frame = kdl.Frame(r, v)
    for i in range(3):
        for j in range(3):
            frame.M[i,j] = mat[i,j]

    return frame

def yarpPose2KDLPose(poseBottle):
    l = poseBottle.get(1).asList()
    v = kdl.Vector(l.get(1).asDouble(), l.get(2).asDouble(), l.get(3).asDouble())
    #r = kdl.Rotation.RPY(l.get(4).asDouble(), l.get(5).asDouble(), l.get(6).asDouble())
    r = kdl.Rotation()
    return kdl.Frame(r, v)

def KDLPose2yarpPose(kdlPose, bottle, frame="world"):
    bottle.clear()

    tmp0 = bottle.addList()
    tmp0.addString("frame")
    tmp0.addString(frame)
    p = kdlPose.p
    r = kdlPose.M.GetQuaternion()
    tmp = bottle.addList()
    tmp.addString("data")
    tmp.addDouble(p[0])
    tmp.addDouble(p[1])
    tmp.addDouble(p[2])
    tmp.addDouble(r[0])
    tmp.addDouble(r[1])
    tmp.addDouble(r[2])
    tmp.addDouble(r[3])

    return bottle


def yarpListToTuple(listBottle):
    tuple = []
    for i in range(listBottle.size()):
        tuple += [listBottle.get(i).asDouble()]
    return tuple

__author__ = ('Alessio Rocchi', 'Enrico Mingo')

if __name__ == '__main__':

    print('Welcome to pydynutils!')
