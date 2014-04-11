#!/usr/bin/env python
import errno
import select
import socket
import sys
import traceback
import rospy
from std_msgs.msg import String
import json

class Server:
    def __init__(self, port, ref, teammate, logger):
        self.host = ""
        self.port = port
        #self.logger = logger
        self.open_socket()
        self.clients = {}
        self.addresses = {}
        self.size = 1024
        self.ref = ref
        self.teammate = teammate
        #self.logger.info("Server")
        #self.logger.debug("ref is at: %s" % self.ref)
        #self.logger.debug("teammate is at: %s" % self.teammate)
        rospy.init_node('robotServer', anonymous=False)
        self.pub = rospy.Publisher('serverCommChatter', String)

    def open_socket(self):
        try:
            self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
            self.server.bind((self.host,self.port))
            self.server.listen(5)
            self.server.setblocking(0)
        except socket.error, (value,message):
            if self.server:
                self.server.close()
            print "Could not open socket: " + message
            sys.exit(1)

    def run(self):
        self.poller = select.epoll()
        self.pollmask = select.EPOLLIN | select.EPOLLHUP | select.EPOLLERR
        self.poller.register(self.server,self.pollmask)
        while True:
            try:
                fds = self.poller.poll(timeout=1)
            except:
                return
            for (fd,event) in fds:
                #self.logger.debug("handling: (%s,%s)" % (fd, event))
                # handle errors
                if event & (select.POLLHUP | select.POLLERR):
                    self.handleError(fd)
                    continue
                # handle the server socket
                if fd == self.server.fileno():
                    self.handleServer()
                    continue
                # handle client socket
                result = self.handleClient(fd)

    def handleError(self,fd):
        #self.logger.error("something happened at: %s" % fd)
        self.poller.unregister(fd)
        if fd == self.server.fileno():
            # recreate server socket
            self.server.close()
            self.open_socket()
            self.poller.register(self.server,self.pollmask)
        else:
            # close the socket
            self.clients[fd].close()
            del self.clients[fd]
            del self.addresses[fd]

    def handleServer(self):
        # accept as many clients are possible
        while True:
            try:
                (client,address) = self.server.accept()
            except socket.error, (value,message):
                # if socket blocks because no clients are available,
                # then return
                if value == errno.EAGAIN or errno.EWOULDBLOCK:
                    return
                print traceback.format_exc()
                sys.exit()
            # set client socket to be non blocking
            client.setblocking(0)
            self.clients[client.fileno()] = client
            self.addresses[client.fileno()] = address[0]
            self.poller.register(client.fileno(),self.pollmask)

    def handleClient(self,fd):
        try:
            data = self.clients[fd].recv(self.size)
            address = self.addresses[fd]
        except socket.error, (value,message):
            # if no data is available, move on to another client
            if value == errno.EAGAIN or errno.EWOULDBLOCK:
                return
            #self.logger.error("error handling client: %s" % fd)
            #self.logger.exception(traceback.format_exc())
            print traceback.format_exc()
            sys.exit()

        if data:
            self.handleData(data, address)
        else:
            self.poller.unregister(fd)
            self.clients[fd].close()
            del self.clients[fd]
            del self.addresses[fd]

    def handleData(self, data, address):
        '''
        this is where your code goes
        '''
        #self.logger.info("received %s from: %s" % (data, address))
        msg = {}
        msg["data"] = data
        msg["address"] = address
        rospy.loginfo(json.dumps(msg))
        self.pub.publish(json.dumps(msg))

if __name__ == "__main__":
    try:
        s = Server(1337, "192.168.1.117", "192.168.1.117", "logger")
        s.run()
    except rospy.ROSInterruptException:
        pass
