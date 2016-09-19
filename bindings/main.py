import yarp
import pydynutils

if __name__ == "__main__":

    robot = pydynutils.iDynUtils("coman",
                                 "../tests/robots/coman/coman.urdf",
                                 "../tests/robots/coman/coman.srdf")
    q = robot.getAng()
