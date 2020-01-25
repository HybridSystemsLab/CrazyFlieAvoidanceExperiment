
import scipy
from scipy import signal
import numpy as np
import time
import math
from feedback.transformations import euler_from_quaternion

# todo: make this more extensible
def butter_lowpass(cutoff, fs, order=3):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = signal.butter(order, normal_cutoff, btype='low', analog=True)
    return b, a

def butter_lowpass_filter(b, a, data):
    y = signal.lfilter(b, a, data)
    return y

def quat2euler(q):
    """

    Function for returning a set of Euler angles from a given quaternion. Uses a fixed rotation sequence.

    :param q:
    :return:

    """
    qx, qy, qz, qw = q
    sqx, sqy, sqz, sqw = q ** 2
    invs = 1.0 / (sqx + sqy + sqz + sqw)

    yaw = np.arctan2(2.0 * (qx * qz + qy * qw) * invs, (sqx - sqy - sqz + sqw) * invs)
    pitch = -np.arcsin(2.0 * (qx * qy - qz * qw) * invs)
    roll = np.arctan2(2.0 * (qy * qz + qx * qw) * invs, (-sqx + sqy - sqz + sqw) * invs)

    return np.array((yaw, pitch, roll))


class Frame(object):

    def __init__(self, value=None):
        self._frame_data = np.array(value)
        self.time_stamp = time.time()

    @property
    def frame_data(self):
        if self._frame_data is not None:
            return self._frame_data
        else:
            return None

    @frame_data.setter
    def frame_data(self, value):
        self._frame_data = np.array(value)
        self.time_stamp = time.time()

    @property
    def detected(self):
        if self._frame_data is not None:
            return self._frame_data[-1]
        else:
            return None

    @property
    def state(self):
        if self._frame_data is not None:
            return self._frame_data[:6]
        else:
            return None


class FrameHistory(object):

    def __init__(self, extrapolating=False, filtering=False, **kwargs):
        self.extrapolation_max = kwargs.get('extrapolation_max', 5)
        self._cutoff, self._fs, self._order = kwargs.get('cutoff', 20), kwargs.get('fs', 120), kwargs.get('order', 4)
        self.smooth_operator = np.array([0, 0, 0, 0, 0, 0])  # no need to ask...
        self.current_frame = None
        self.l_frame = None
        self.ll_frame = None
        self._filtered_frame = None
        self.extrapolation_count = 0
        self.prev_time = time.time()

        self.filtering = filtering
        self.extrapolating = extrapolating
        if filtering:
            self._b, self._a = butter_lowpass(self._cutoff, self._fs, order=self._order)

    @property
    def cutoff(self):
        return self._cutoff

    @cutoff.setter
    def cutoff(self, value):
        if value > 0:
            self._cutoff = value
        else:
            self._cutoff = 8  # default
    @property
    def filtered_frame(self):
        return self._filtered_frame

    @filtered_frame.setter
    def filtered_frame(self, value):
        self.ll_frame = self.l_frame
        self.l_frame = self.filtered_frame
        self._filtered_frame = value

    @property
    def can_extrapolate(self):
        """

        Determine if extrapolation is valid based on given constraints

        :return: True if we can extrapolate position, false if we cannot
        """
        return self.l_frame is not None and self.ll_frame is not None and self.extrapolation_count \
                                                                          < self.extrapolation_max

    def extrapolate(self):
        """

        Used to estimate the position of a body during occlusion, or missed frame events.
        Based on the previous two valid frames, estimate velocity, and therefore position of the current dropped frame.
        :return: the estimated state of the body, or none is extrapolation cannot be completed

        """
        if self.can_extrapolate:
            self.extrapolation_count += 1
            dt = time.time() - self.prev_time # current time - last time we added a state to smooth operator
            frame_velocity = (self.l_frame.state - self.ll_frame.state)/(self.l_frame.time_stamp - self. ll_frame.time_stamp)
            state = self.l_frame.state + frame_velocity*dt
            return state
        else:
            self.extrapolation_count = 0
            return None

    def filter(self, state):
        """

        Apply a Butterworth filter to input states

        :param state: state array consisting of [x, y, z, yaw, roll, pitch]
        :return:

        """
        if state is not None:  # not None if frame is valid, or we could extrapolate
            self.smooth_operator = np.vstack((self.smooth_operator, state))
            self.prev_time = time.time()
            filtered = []
            if self.smooth_operator.shape[0] > 250:
                self.smooth_operator = np.delete(self.smooth_operator, 0, 0)
                # Filter
                for column in range(len(self.current_frame.state + 1)):
                    filt = butter_lowpass_filter(self._b, self._a, self.smooth_operator[:, column])
                    #print("Last", filt[-1])
                    filtered.append(filt[-1])
                return filtered
        return None

    def decode_packet(self, packet):
        """

        Read in a packed message from the ZMQ stream, deserialize using msgpack.
        :param packet: Raw, serialized packet from ZMQ stream
        :return: The state of the vehicle is given in an array of the form [x, y, z, yaw, roll, pitch, detected(bool)]
        """

        packet  = packet[0]
        detected = packet[-1]
        delta = packet[-2]
        x, y, z = packet[0], packet[1], packet[2]
        x, y, z = -x, z, y
        q = np.array([packet[3], packet[4], packet[5], packet[6]])
        #np.linalg.norm(q)  # (qx, qy, qz, qw)
        # print("X:{}, Y:{}, Z:{}".format(x, y, z))
        orientation = [elem * (180 / math.pi) for elem in euler_from_quaternion(q, axes='syxz')]
        yaw, roll, pitch = orientation[0], orientation[1], orientation[2]
        return [x, y, z, yaw, roll, pitch, detected]

    def update(self, packet):
        """

        Update the frame history by reading in new packets, constructing frame objects, and performing filtering and
        extrapolation as desired. The packet is a serialized chunk of prepackaged data coming from a ZMQ REQ server
        running on a machine running Motive, Optitracks GUI that doubles as a UDP packet server

        :param packet: Serialized data from ZMQ stream
        :return: A frame of data, appropriately extrapolated and or filtered if specified in __init__

        """
        # Check if body is being tracked by cameras
        self.current_frame = Frame(self.decode_packet(packet))
        if self.current_frame.detected:  # Unpack position, orientation add it to current frame, update frame history
            state = self.current_frame.state
            self.extrapolation_count = 0
        else:  # extrapolate
            if self.extrapolating:
                state = self.extrapolate()
            else:
                state = None

        if state is not None:
            if self.filtering:
                state = self.filter(state)
                print(state)
            else:
                pass

            if state is not None:
                self.filtered_frame = Frame(state)
                # print self.filtered_frame.frame_data
                return self.filtered_frame  # indicate success in updating

        else:
            return None  # id frame was not valid, and we cannot extrapolate