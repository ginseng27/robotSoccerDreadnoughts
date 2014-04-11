import serial
import time

class Roboclaw:

    def __init__(self, m1Factor, m2Factor, port, baud=38400):
        self.port = port
        self.baud = baud
        self.ser = serial.Serial(self.port, int(self.baud))
        self.m1TickConversionFactor = m1Factor
        self.m2TickConversionFactor = m2Factor

    def closeSerial(self, cmd):
        self.ser.close()

    def serialSend(self, cmd):
        try:
            for i in cmd:
                self.ser.write(chr(i))
        except ValueError, e:
            print "value error while opening %s @ %s" % (self.port, self.baud)
            print "e mssage: %s" % e
        except SerialTimeoutException, e:
            print "serial port timed out while writing %s @ %s" % (self.port, self.baud)
            print "e message: %s" % e
        except Exception, e:
            print "something happened while opening serial port!"
            print "e message: %s" % e
            print "port @ baud: %s, %s" % (self.port, self.baud)

    def serialRead(self, cmd, length):
        try:
            for i in cmd:
                self.ser.write(chr(i))
            data = []
            for i in range(length):
                data.append(self.ser.read())
            return data
        except Exception, e:
            print "error while doing serial read %s @%s" % (self.port, self.baud)
            print "e msg: %s" % e

    def m1SpeedRead(self, address=128):
        '''
        returns rotations per second and direction
        '''
        cmd = [address, 30]
        data = self.serialRead(cmd, length=6)
        speed = (data[0].encode("hex")) + (data[1].encode("hex")) + (data[2].encode("hex")) + (data[3].encode("hex"))
        speed = int(speed, 16)
        if ((ord(data[4]) == 1) and (speed != 0)):
            speed = ~(0xffffffff - speed) + 1
        # *125/8192 --> resolution in 125ths of a second, and then (apparently) 8192 ticks per rotation.
        rps = abs(float(speed) * 125 / 8192) 
        return (rps, ord(data[4]))

    def m2SpeedRead(self, address=128):
        cmd = [address, 31]
        data = self.serialRead(cmd, length=6)
        speed = (data[0].encode("hex")) + (data[1].encode("hex")) + (data[2].encode("hex")) + (data[3].encode("hex"))
        speed = int(speed, 16)
        if ((ord(data[4]) == 1) and (speed != 0)):
            speed = ~(0xffffffff - speed) + 1
        rps = abs(float(speed) * 125 / 8192)
        return (rps, ord(data[4]))

    def convertPercentageSpeed(self, percentageSpeed, max_ticks):
        speedDec = float(float(percentageSpeed)/float(100))
        normalizedSpeed = int(speedDec * max_ticks)
        if normalizedSpeed >= max_ticks:
            normalizedSpeed = max_ticks
        return normalizedSpeed

    def m1AngularSpeed(self, address=128, angularSpeed=0):
        '''
            angularSpeed is in rad/s. does the conversion to ticks
        '''
        #ticks = angularSpeed * 8192 * 2.2 / 6.28
        ticks = angularSpeed * self.m1TickConversionFactor
        all_speeds = [(int(ticks) >> i &0xff) for i in (24,16,8,0)]
        checksum = ((address + 35 + all_speeds[0] + all_speeds[1] + all_speeds[2] + all_speeds[3]) & 0x7f)
        cmd = [address, 35, all_speeds[0], all_speeds[1], all_speeds[2], all_speeds[3], checksum]
        self.serialSend(cmd)

    def m2AngularSpeed(self, address=128, angularSpeed=0):
        '''
            angularSpeed is in rad/s. does the conversion to ticks
        '''
        #ticks = angularSpeed * 8192 * 2.2 /  6.28
        ticks = angularSpeed * self.m2TickConversionFactor
        all_speeds = [(int(ticks) >> i &0xff) for i in (24,16,8,0)]
        checksum = ((address + 36 + all_speeds[0] + all_speeds[1] + all_speeds[2] + all_speeds[3]) & 0x7f)
        cmd = [address, 36, all_speeds[0], all_speeds[1], all_speeds[2], all_speeds[3], checksum]
        self.serialSend(cmd)

    def drivem1m2SignedSpeed(self, address=128, m1angularSpeed=0, m2angularSpeed=0):
        #ticks = m1angularSpeed * 8192 * 2.2 /  6.28
        ticks = m1angularSpeed * self.m1TickConversionFactor
        m1all_speeds = [(int(ticks) >> i &0xff) for i in (24,16,8,0)]
        #ticks = m2angularSpeed * 8192 * 2.2 / 6.28
        ticks = m2angularSpeed * self.m2TickConversionFactor
        m2all_speeds = [(int(ticks) >> i &0xff) for i in (24,16,8,0)]
        cmd = [address, 37]
        checksum = (address + 37)
        for i in m1all_speeds:
            cmd.append(i)
            checksum += i
        for i in m2all_speeds:
            cmd.append(i)
            checksum += i
        checksum = checksum & 0x7f
        cmd.append(checksum)
        self.serialSend(cmd)

#    def m1Control(self, address=128, speed=0, max_ticks=250000):
#        '''
#            speed is in percentage
#        '''
#        convertedSpeed = self.convertPercentageSpeed(speed, max_ticks)
#        all_speeds = [int(convertedSpeed >> i & 0xff) for i in (24,16,8,0)]
#        checksum = ((address + 35 + all_speeds[0] + all_speeds[1] + all_speeds[2] + all_speeds[3]) & 0x7f)
#        cmd = [address, 35, all_speeds[0], all_speeds[1], all_speeds[2], all_speeds[3], checksum]
#        self.serialSend(cmd)
#        #if speed > 0:
#        #    self.m1Forward(address=address, speed=speed)
#        #else:
#        #    self.m1Backward(address=address,speed=abs(speed))
#
#    def m2Control(self, address=128, speed=0, max_ticks=250000):
#        convertedSpeed = self.convertPercentageSpeed(speed, max_ticks)
#        all_speeds = [int(convertedSpeed >> i & 0xff) for i in (24,16,8,0)]
#        checksum = ((address + 36 + all_speeds[0] + all_speeds[1] + all_speeds[2] + all_speeds[3]) & 0x7f)
#        cmd = [address, 36, all_speeds[0], all_speeds[1], all_speeds[2], all_speeds[3], checksum]
#        self.serialSend(cmd)
#        #if speed > 0:
#        #    self.m2Forward(address=address, speed=speed)
#        #else:
#        #    self.m2Backward(address=address,speed=abs(speed))

    def drivem1Forward(self, address=128, speed=0):
        cmd = [address, 0, speed, ((address+speed) & 0x7f)]
        self.serialSend(cmd)

    def drivem1Backward(self, address=128, speed=0):
        cmd = [address, 1, speed, ((address + 1 + speed) & 0x7f)]
        self.serialSend(cmd)

    def drivem2Forward(self, address=128, speed=0):
        cmd = [address, 4, speed, ((address + 4 + speed) & 0x7f)]
        self.serialSend(cmd)

    def drivem2Backward(self, address=128, speed=0):
        cmd = [address, 5, speed, ((address + 5 + speed) & 0x7f)]
        self.serialSend(cmd)

    def stopAllMotors(self, address=128):
        self.drivem1Forward(address)
        self.drivem2Forward(address)

    def readm1PIDSettings(self, address=128):
        cmd = [address, 55]
        return self.serialRead(cmd, 17)

    def readm2PIDSettings(self, address=128):
        cmd = [address, 56]
        return self.serialRead(cmd, 17)

    def setm1PIDConstants(self, p, i, d, qpps, address=128):
        '''
        p, i, d, and qpps are inputted as ints.
        this method will split and convert as needed
        '''
        cmd = [address, 28]
        pnew = [(int(p) >> j & 0xff) for j in (24, 16, 8, 0)]
        inew = [(int(i) >> j & 0xff) for j in (24, 16, 8, 0)]
        dnew = [(int(d) >> j & 0xff) for j in (24, 16, 8, 0)]
        qppsnew = [(int(qpps) >> j & 0xff) for j in (24, 16, 8, 0)]
        for j in dnew:
            cmd.append(j)
        for j in pnew:
            cmd.append(j)
        for j in inew:
            cmd.append(j)
        for j in qppsnew:
            cmd.append(j)

        checksum = 0
        for i in cmd:
            checksum += i
        checksum = checksum & 0x7f
        cmd.append(checksum)
        self.serialSend(cmd)

    def setm2PIDConstants(self, p, i, d, qpps, address=128):
        '''
        p, i, d, and qpps are inputted as ints.
        this method will split and convert as needed
        '''
        cmd = [address, 29]
        pnew = [(int(p) >> j & 0xff) for j in (24, 16, 8, 0)]
        inew = [(int(i) >> j & 0xff) for j in (24, 16, 8, 0)]
        dnew = [(int(d) >> j & 0xff) for j in (24, 16, 8, 0)]
        qppsnew = [(int(qpps) >> j & 0xff) for j in (24, 16, 8, 0)]
        for j in dnew:
            cmd.append(j)
        for j in pnew:
            cmd.append(j)
        for j in inew:
            cmd.append(j)
        for j in qppsnew:
            cmd.append(j)

        checksum = 0
        for i in cmd:
            checksum += i
        checksum = checksum & 0x7f
        cmd.append(checksum)
        self.serialSend(cmd)

