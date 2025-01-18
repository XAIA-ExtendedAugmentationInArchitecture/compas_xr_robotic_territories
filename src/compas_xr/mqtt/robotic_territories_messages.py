import sys
import threading
import uuid
from datetime import datetime

from compas.geometry import Frame, Point, Vector
from compas_eve import Message

class Header(Message):
    """
    The header class is responsible for coordinating and understanding messages between users.

    The Header class provides methods for parsing, updating, and accessing the header fields of a message,
    and provides a means of defining attributes of the message in order to accept or ignore specific messages.

    Parameters
    ----------
    increment_response_ID : bool, optional
        Whether to increment the response ID when creating a new instance of Header.
    sequence_id : int, optional
        The sequence ID of the message. Optional for parsing.
    response_id : int, optional
        The response ID of the message. Optional for parsing.
    device_id : str, optional
        The device ID of the message. Optional for parsing.
    time_stamp : str, optional
        The timestamp of the message. Optional for parsing.

    Attributes
    ----------
    increment_response_ID : bool
        Whether to increment the response ID when creating a new instance of Header.
    sequence_id : int
        Sequence ID is an atomic counter that increments with each message.
    response_id : int
        Response ID is an int that increments with request routine.
    device_id : str
        Device ID coresponds to the unique system identifier that send the message.
    time_stamp : str
        Timestamp is the time in which the message was sent.
    """
    _device_id = None

    def __init__(self, device_id=None, time_stamp=None):
        super(Header, self).__init__()
        self["device_id"] = device_id or self._get_device_id()
        self["time_stamp"] = time_stamp or self._get_time_stamp()

    @classmethod
    def parse(cls, value):
        """Parse the header information
        from the input value
        """
        # instance = cls(value["device_id"], value["time_stamp"])
        instance = cls("device_id", "time_stamp")
        return instance

    def _get_device_id(self):
        """Ensure device ID is set and return it.
        If not set, generate a new device ID.
        """
        if not Header._device_id:
            Header._device_id = str(uuid.uuid4())
            self.device_id = Header._device_id
        else:
            self.device_id = Header._device_id
        return self.device_id

    def _get_time_stamp(self):
        """Generate timestamp and return it."""
        self.time_stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        return self.time_stamp

class MimicTrajectoryRequestMessage(Message):
    """
    The MimicTrajectoryRequestMessage class is responsible for requesting a robot to mimic a trajectory.

    The MimicTrajectoryRequestMessage class provides methods for parsing, updating, and accessing the fields of a message,
    and provides a means of defining attributes of the message in order to accept or ignore specific messages.

    Parameters
    ----------
    header : Header
        The header of the message.
    """
    @classmethod
    def parse(cls, data):
        """Parse the message information
        from the input data
        """
        # header = Header.parse(data["header"])
        header = Header("device_id", "time_stamp")
        # human_frames = data["human_frames"] #cls._parse_frames_list_from_data(data["human_frames"])
        # robot_frames = data["robot_frames"]#cls._parse_frames_list_from_data(data["robot_frames"])
        # robot_name = data["robot_name"]
        test_frames_list = [Frame.worldXY(), Frame.worldXY(), Frame.worldXY()]
        instance = cls(test_frames_list, test_frames_list, "UR20", header)
        return instance
        # return cls(human_frames, robot_frames, robot_name, header)

    def __init__(self, human_frames, robot_frames, robot_name, header=None):
        super(MimicTrajectoryRequestMessage, self).__init__()
        self["header"] = header or Header()
        self["human_frames"] = human_frames
        self["robot_frames"] = robot_frames
        self["robot_name"] = robot_name

    def _parse_frames_list_from_data(self, data):
        """Parse the list of frames from the input data."""
        return [Frame.__from_data__(frame) for frame in data] #TODO: CHECK IF THIS WORKS

    @classmethod
    def parse(cls, value):
        """Parse the message information
        from the input value
        """
        header = Header.parse(value["header"])
        trajectory = [Frame.from_data(frame) for frame in value["trajectory"]]
        return cls(header, trajectory)