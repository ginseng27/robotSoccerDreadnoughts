import subprocess
import os
import time

class Shooter:
    def __init__(self):
        print "Shooter control on gpio247"
        (filepath,filename) = os.path.split(os.path.realpath(__file__))
        self.kick_out = ["%s/kick_out.sh" % filepath]
        self.kick_in = ["%s/kick_in.sh" % filepath]
        setup = ["%s/setup_gpio.sh" % filepath]
        print "filepath: %s" % filepath
        self.execute_command(setup)

    def execute_command(self, cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True):
        print "executing: %s" % cmd
        subprocess.call(cmd, stdout=stdout, stderr=stderr, shell=shell)
        print "done"

    def kickOut(self):
        self.execute_command(self.kick_out)

    def kickIn(self):
        self.execute_command(self.kick_in)

    def kickNow(self):
        self.kickOut()
        time.sleep(0.5)
        self.kickIn()
