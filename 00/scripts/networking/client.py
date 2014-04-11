import socket
import sys

class Client:
    def __init__(self,host,port):
        self.host = host
        self.port = port
        self.size = 1024
        self.open_socket()

    def open_socket(self):
        try:
            self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server.connect((self.host,self.port))
        except socket.error, (value,message):
            if self.server:
                self.server.close()
            print "Could not open socket: " + message
            sys.exit(1)
    
    def run(self):
        while True:
            print "type somethign to send"
            line = sys.stdin.readline()
            self.sendInfo(line)

    def sendInfo(self, info):
        print "Sending: %s" % info
        self.server.send(info)

