#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This module implements a server that reads UDP packets from a chosen port.

Class UDPServer is a daemon thread listening on a specified port.
Class UDPServerReader is the copy of PyGameReader class, whith same variables and methods, and communicates with
UDPServer.

Packet format: S[TART] LEN TYPE ROLL PITCH YAW THRUST VAL7 TIMESTAMP CRC E[ND]
"""

__author__ = "Nicolo' Boninsegna"
__all__ = ['UDPServerReader']

import socket
import threading
import Queue
import logging
import math

logger = logging.getLogger(__name__)


class UDPServer(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self._initialStoredData = "S 0 0 0 0 0 0 0 0 0 E"
        self._storedData = self._initialStoredData
        self._ready = False

    def prepare(self, addr, port):
        self._UDPSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._UDPSock.bind((addr, port))
        self._UDPSock.settimeout(0.5)
        self._port = port
        self._queue = Queue.Queue()
        self._ready = True

    def isReady(self):
        return self._ready

    def getData(self):
        try:
            return self._queue.get(False)
        except Exception:
            return self._storedData

    def run(self):
        logger.info("Listening on port %d", self._port)
        while True:
            try:
                # data, addr = self._UDPSock.recvfrom(64)
                data, addr = self._UDPSock.recvfrom(90)
                self._storedData = data
                self._queue.put(data)
            except Exception:
                self._storedData = self._initialStoredData


class UDPServerReader():
    contatore = 0

    def __init__(self):
        self.name = "UDP Server Reader"
        self.server = UDPServer()
        self.i = 0
        self.data = {}

    def getAvailableDevices(self):
        """List all the available devices"""
        dev = []
        dev.append({"id": 0, "name": self.name})
        return dev

    def start_input(self, deviceId, inputMap, port):
        """Initalize the reading and set the mapping for axis/buttons using the	inputMap"""

        self.data = {"value": 0.0, "pitch": 0.0, "yaw": 0.0, "yaw_Opti": 0.0, "thrust": 0.0, "pitchcal": 0.0,
                     "rollcal": 0.0, "estop": False, "exit": False, "althold": False}
        self.inputMap = inputMap
        if not self.server.isReady():
            self.server.prepare("", port)
        if not self.server.isAlive() and self.server.isReady():
            self.server.start()

    # self.contatore = 0

    def read_input(self):
        """Read input"""
        packet = self.server.getData()
        """
        self.i += 1
        if (self.i > 100):
            print packet
            self.i = 0
        """
        splitPacket = packet.split()
        roll, pitch, yaw, thrust = splitPacket[3:7]
        yaw_Opti = splitPacket[2]  ## Test in corso
        """
        roll = (float(roll) - 512) / 512      # Input range [0, 1024] converted to [-1, 1]
        pitch = (float(pitch) - 512) / 512    # Input range [0, 1024] converted to [-1, 1]
        yaw = (float(yaw) - 512) / 512        # Input range [0, 1024] converted to [-1, 1]
        thrust = (float(thrust) - 512) / 512  # Input range [0, 1024] converted to [-1, 1]
        yaw_Opti = (float(yaw_Opti))          # Input range [-pi/pi] no conversion
        """
        roll = float(roll) / (math.pi)  # Input range [-pi, pi] converted to [-1, 1]
        pitch = float(pitch) / (math.pi)  # Input range [-pi, pi] converted to [-1, 1]
        yaw = float(yaw) / (math.pi)  # Input range [-pi, pi]   converted to [-1, 1]
        yaw_Opti = float(yaw_Opti) * 180 / math.pi  # Crazyflie mantiene yaw tra -90 e 90
        thrust = float(thrust) / 100  # Input range [0, 100]      converted to [0, 1]

        self.data["roll"], self.data["pitch"], self.data["yaw"], self.data["thrust"], self.data[
            "yaw_Opti"] = roll, pitch, -yaw, thrust, -yaw_Opti
        """
        self.i += 1
        if (self.i > 100):
            print self.data
            self.i = 0
        """
        return self.data

    def enableRawReading(self, deviceId):
        """Enable reading of raw values (without mapping)"""
        pass

    def disableRawReading(self):
        """Disable raw reading"""
        # No need to de-init since there's no good support for multiple input devices
        pass

    def readRawValues(self):
        """Read out the raw values from the device"""
        rawaxis = {}
        rawbutton = {}
        return [rawaxis, rawbutton]


server_reader = UDPServerReader()
server_reader.start_input(00000, None, 9030)
