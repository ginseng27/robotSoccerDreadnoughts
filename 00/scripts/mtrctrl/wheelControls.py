import roboclaw
import numpy

class WheelControls:

    def __init__(self, conversionFactor=[8192*2.2/6.28, 8192*2.2/6.28, 8192*2.2/6.28],
                 pConst = 100000, iConst=50000, dConst=16351, qppsConst=180000,
                 dribbleSpeed=5,
                 ports=["/dev/ttyPCH1", "/dev/ttyPCH2"], baud=38400):
        '''
        note:
            this assumes that left,right ctrls are on ttyPCH1 m1, m2 respectively
            this assumes that back,misc ctrls are on ttyPCH2 m1, m2 respectively
            all speeds inputted into functions MUST be 0<speed<100

        '''
        self.port1 = ports[0]
        self.port2 = ports[1]
        self.baud = baud
        self.leftFactor = conversionFactor[0]
        self.rightFactor  = conversionFactor[1]
        self.backFactor = conversionFactor[2]
        self.leftRightCtrl = roboclaw.Roboclaw(self.leftFactor, self.rightFactor, self.port1, self.baud)
        self.backnMiscCtrl = roboclaw.Roboclaw(self.backFactor, self.backFactor, self.port2, self.baud)
        self.dribbleSpeed = dribbleSpeed

        self.conversionMatrix = numpy.zeros(shape=(3,3))
        self.conversionMatrix[0] = [-19.685, 34.0955, 1.375]
        self.conversionMatrix[1] = [-19.685, -34.0955, 1.375]
        self.conversionMatrix[2] = [39.3701, 0, 6.25]
        print "conversionmatrix being used:"
        print self.conversionMatrix
        self.inversionMatrix = numpy.zeros(shape=(3,3))
        self.inversionMatrix[0] = [-0.017639, -0.017639, 0.0077611]
        self.inversionMatrix[1] = [.014665, -0.014665, 0]
        self.inversionMatrix[2] = [.11111, .11111, .11111]

        print "setting pid values of: %s, %s, %s, %s" % (pConst, iConst, dConst, qppsConst)
        self.setBackPID(p=pConst, i=iConst, d=dConst, qpps=qppsConst)
        self.setRightPID(p=pConst, i=iConst, d=dConst, qpps=qppsConst)
        self.setLeftPID(p=pConst, i=iConst, d=dConst, qpps=qppsConst)
#        self.setDribblerPID(p=pConst, i=iConst, d=dConst, qpps=qppsConst)

    def actualRobotSpeed(self):
        wheelSpeeds = self.readAll()
        wheelSpeedsMatrix = []
        wheelSpeedsMatrix.append(wheelSpeeds["right"][0])
        wheelSpeedsMatrix.append(wheelSpeeds["left"][0])
        wheelSpeedsMatrix.append(wheelSpeeds["back"][0])
        robospeed = self.inversionMatrix.dot(wheelSpeedsMatrix)
        return robospeed


    def readAll(self):
        rightSpeed = self.leftRightCtrl.m1SpeedRead()
        leftSpeed = self.leftRightCtrl.m2SpeedRead()
        backSpeed = self.backnMiscCtrl.m2SpeedRead()
        return {"right": rightSpeed, "left": leftSpeed, "back": backSpeed}

    def setDribblerPID(self, p, i, d, qpps):
        self.backnMiscCtrl.setm1PIDConstants(p=p, i=i, d=d, qpps=qpps)


    def setLeftPID(self, p, i, d, qpps):
        '''
        set the left PID values.
        p, i, d, and qpps are ints
        '''
        self.leftRightCtrl.setm1PIDConstants(p=p, i=i, d=d, qpps=qpps)

    def setRightPID(self, p, i, d, qpps):
        '''
        set the right PID values
        p, i, d, and qpps are ints
        '''
        self.leftRightCtrl.setm2PIDConstants(p=p, i=i, d=d, qpps=qpps)

    def setBackPID(self, p, i, d, qpps):
        '''
        set the back PID values
        p, i, d, and qpps are ints
        '''
        self.backnMiscCtrl.setm2PIDConstants(p=p, i=i, d=d, qpps=qpps)

    def readLeftPID(self):
        '''
        read the left PID values
        doesn't do any conversion, just spits out the data
        '''
        return self.leftRightCtrl.readm1PIDSettings()

    def readRightPID(self):
        '''
        read the right PID values
        doesn't do any conversion, just spits out the data
        '''
        return self.leftRightCtrl.readm2PIDSettings()

    def readBackPID(self):
        '''
        read the back PID values
        doesn't do any conversion, just spits out the data
        '''
        return self.backnMiscCtrl.readm2PIDSettings()

    def readLeftSpeed(self):
        return self.leftRightCtrl.m1SpeedRead()

    def readRightSpeed(self):
        return self.leftRightCtrl.m2SpeedRead()

    def readBackSpeed(self):
        return self.backnMiscCtrl.m2SpeedRead()

    def readMiscSpeed(self):
        return self.backnMiscCtrl.m1SpeedRead()

    def dribbleOn(self, on):
        if on is True:
            self.backnMiscCtrl.m1AngularSpeed(angularSpeed=self.dribbleSpeed)
        else:
            self.backnMiscCtrl.m1AngularSpeed(angularSpeed=0)

    def moveGeneric(self, speedsMatrix):
        '''
            speedsMatrix = [x, y, w] in m/s
            note: this uses command 37 (mix mode drive m1/m2 w/ signed speeds)
                    for left and right wheels.
        '''
        wheelSpeeds = self.conversionMatrix.dot(speedsMatrix)
        #self.leftRightCtrl.drivem1m2SignedSpeed(m1angularSpeed=wheelSpeeds[0], m2angularSpeed=-wheelSpeeds[1])
        self.leftRightCtrl.m1AngularSpeed(angularSpeed=wheelSpeeds[0])
        self.leftRightCtrl.m2AngularSpeed(angularSpeed=-wheelSpeeds[1])
        self.backnMiscCtrl.m2AngularSpeed(angularSpeed=-wheelSpeeds[2])
        return wheelSpeeds

    def moveBackward(self, angularSpeed):
        backwardMatrix = [0, -angularSpeed, 0]
        return self.moveGeneric(backwardMatrix)

    def moveForward(self, angularSpeed):
        forwardMatrix = [0, angularSpeed, 0]
        return self.moveGeneric(forwardMatrix)

    def rotateCW(self, angularSpeed):
        rotateMatrix = [0, 0, angularSpeed]
        return self.moveGeneric(rotateMatrix)

    def rotateCCW(self, percentageSpeed):
        rotateMatrix = [0, 0, -angularSpeed]
        return self.moveGeneric(rotateMatrix)

    def moveLeft(self, angularSpeed):
        leftMatrix = [-angularSpeed, 0, 0]
        return self.moveGeneric(leftMatrix)

    def moveRight(self, angularSpeed):
        rightMatrix = [angularSpeed, 0, 0]
        return self.moveGeneric(rightMatrix)

    # all 45 degree angle functions:
#    def moveNW(self, percentageSpeed):
#        NWMatrix = [-percentageSpeed, -percentageSpeed, 0]
#        wheelSpeedPercentage = self.conversionMatrix.dot(NWMatrix)
#        wheelSpeeds = self.changeMatrixTo127(wheelSpeedPercentage)
#        return self.wheelControls(wheelSpeeds)
#
#    def moveNE(self, percentageSpeed):
#        NEMatrix = [-percentageSpeed, percentageSpeed, 0]
#        wheelSpeedPercentage = self.conversionMatrix.dot(NEMatrix)
#        wheelSpeeds = self.changeMatrixTo127(wheelSpeedPercentage)
#        return self.wheelControls(wheelSpeeds)
#
#    def moveSW(self, percentageSpeed):
#        SWMatrix = [percentageSpeed, -percentageSpeed, 0]
#        wheelSpeedPercentage = self.conversionMatrix.dot(SWMatrix)
#        wheelSpeeds = self.changeMatrixTo127(wheelSpeedPercentage)
#        return self.wheelControls(wheelSpeeds)
#
#    def moveSE(self, percentageSpeed):
#        SEMatrix = [-percentageSpeed, percentageSpeed, 0]
#        wheelSpeedPercentage = self.conversionMatrix.dot(SEMatrix)
#        wheelSpeeds = self.changeMatrixTo127(wheelSpeedPercentage)
#        return self.wheelControls(wheelSpeeds)

    # all stop functions follow::
    def fullStop(self):
        self.leftRightCtrl.stopAllMotors()
        self.backnMiscCtrl.stopAllMotors()

    def movementStop(self):
        self.leftRightCtrl.stopAllMotors()
        self.backnMiscCtrl.stopm1()

    def miscStop(self):
        self.backnMiscCtrl.stopm2()

