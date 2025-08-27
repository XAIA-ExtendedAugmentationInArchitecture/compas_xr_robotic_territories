import sys
import threading
import uuid
from datetime import datetime

from compas.geometry import Frame, Point, Vector
from compas_fab.robots import JointTrajectory, JointTrajectoryPoint
from compas_eve import Message


class MessageHandelingExtensions(object):

    @staticmethod
    def _parse_frames_list_from_data(data):
        """Parse the list of frames from the input data."""
        return [Frame.__from_data__(frame) for frame in data] #TODO: CHECK IF THIS WORKS... it does :)
    
    @staticmethod
    def _parse_trajectory_list(data):
        """Parse the list of trajectories from the input data."""
        #TODO: Check this and throw an error?
        for trajectory in data:
            # print(trajectory)
            traj = JointTrajectory.__from_data__(trajectory)

        # return [JointTrajectory.__from_data__(trajectory) for trajectory in data]
    
    @staticmethod #TODO: I am not sure why __from_data__ did not work...
    def _parse_trajectory_point_list(data):
        """Parse the list of trajectory points from the input data."""
        for point in data:
            pt = JointTrajectoryPoint(point["joint_values"], point["joint_types"], point["joint_names"])
            pt.accelerations = point["accelerations"]
            pt.effort = point["effort"]
            pt.velocities = point["velocities"]
        return pt


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
        return cls(value["device_id"], value["time_stamp"])

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

    def __init__(self, human_frames, robot_frames, robot_name, header=None):
        super(MimicTrajectoryRequestMessage, self).__init__()
        self["header"] = header or Header()
        self["human_frames"] = human_frames
        self["robot_frames"] = robot_frames
        self["robot_name"] = robot_name

    @classmethod
    def _parse_frames_list_from_data(self, data):
        """Parse the list of frames from the input data."""
        return [Frame.__from_data__(frame) for frame in data] #TODO: CHECK IF THIS WORKS... it does :) 

    @classmethod
    def parse(cls, value):
        """Parse the message information
        from the input value
        """
        header = Header.parse(value["header"])
        human_frames = cls._parse_frames_list_from_data(value["human_frames"])
        robot_frames = cls._parse_frames_list_from_data(value["robot_frames"])
        robot_name = value["robot_name"]
        return cls(human_frames, robot_frames, robot_name, header)
    
class MimicTrajectoryResultMessage(Message):
    """
    The MimicTrajectoryResultMessage class is responsible for sending the result of a robot mimicking a trajectory.
    """

    def __init__(self, trajectories, robot_base_frame, robot_name, header=None):
        super(MimicTrajectoryResultMessage, self).__init__()
        self["header"] = header or Header()
        self["trajectories"] = trajectories
        self["combined_trajectory_points"] = self._combine_trajectories_points(trajectories)
        self["robot_base_frame"] = robot_base_frame
        self["robot_name"] = robot_name

    def _combine_trajectories_points(self, trajectories):
        """Combine the trajectories into a single trajectory."""
        combined_trajectory = []
        for trajectory in trajectories:
            combined_trajectory.extend(trajectory.points)
        print(type(combined_trajectory))
        return combined_trajectory
    
    @classmethod
    def _parse_trajectory_list(self, data):
        """Parse the list of trajectories from the input data."""
        #TODO: Check this and throw an error?
        return [JointTrajectory.__from_data__(trajectory) for trajectory in data]

    @classmethod
    def parse(cls, value):
        """Parse the message information
        from the input value
        """
        header = Header.parse(value["header"])
        trajectories = cls._parse_trajectory_list(value["trajectories"])
        robot_base_frame = Frame.__from_data__(value["robot_base_frame"])
        robot_name = value["robot_name"]
        return cls(trajectories, robot_base_frame, robot_name, header)

class ExecuteMimicTrajectoryRequestMessage(Message):

    """
    The ExecuteMimicTrajectoryRequestMessage class is responsible for requesting a robot to execute a trajectory.

    The ExecuteMimicTrajectoryRequestMessage class provides methods for parsing, updating, and accessing the fields of a message,
    and provides a means of defining attributes of the message in order to accept or ignore specific messages.

    Parameters
    ----------
    header : Header
        The header of the message.
    """

    def __init__(self, trajectories, combined_trajectory_points_list, robot_name, robot_base_frame, header=None):
        super(ExecuteMimicTrajectoryRequestMessage, self).__init__()
        self["header"] = header or Header()
        self["trajectories"] = trajectories #TODO: THIS NEEDS TO BE PARSE TRAJECTORY LIST
        self["robot_name"] = robot_name
        self["robot_base_frame"] = robot_base_frame
        self["combined_trajectory_points"] = combined_trajectory_points_list

    @classmethod
    def parse(cls, value):
        """Parse the message information
        from the input value
        """
        header = Header.parse(value["header"])
        robot_name = value["robot_name"]
        robot_base_frame = Frame.__from_data__(value["robot_base_frame"])
        trajectories = value["trajectories"]
        combined_trajectory_points_list = MessageHandelingExtensions._parse_trajectory_point_list(value["combined_trajectory_points"])
        return cls(trajectories, combined_trajectory_points_list, robot_name, robot_base_frame, header)
        
class RealtimeMimicRequestMessage(Message):
    """
    The RealtimeMimicRequestMessage class is responsible for requesting a robot to mimic a trajectory in real-time.

    The RealtimeMimicRequestMessage class provides methods for parsing, updating, and accessing the fields of a message,
    and provides a means of defining attributes of the message in order to accept or ignore specific messages.

    Parameters
    ----------
    header : Header
        The header of the message.
    """

    def __init__(self, requested_robot_frame, robot_name, message, header=None, intial_request=False):
        super(RealtimeMimicRequestMessage, self).__init__()
        self["header"] = header or Header()
        # self["human_frames"] = human_frames
        # self["robot_frames"] = robot_frames
        self["robot_name"] = robot_name
        self["requested_robot_frame"] = requested_robot_frame
        self["message"] = message
        self["initial_request"] = intial_request
    
    @classmethod
    def parse(cls, value):
        """Parse the message information
        from the input value
        """
        header = Header.parse(value["header"])
        requested_robot_frame = Frame.__from_data__(value["requested_robot_frame"])
        robot_name = value["robot_name"]
        message = value["message"]
        initial_request = value["initial_request"]
        return cls(requested_robot_frame, robot_name, message, header, initial_request)

class RealtimeMimicResultMessage(Message):
    """
    The RealtimeMimicResultMessage class is responsible for sending the result of a robot mimicking a trajectory in real-time.

    The RealtimeMimicResultMessage class provides methods for parsing, updating, and accessing the fields of a message,
    and provides a means of defining attributes of the message in order to accept or ignore specific messages.

    Parameters
    ----------
    header : Header
        The header of the message.
    """

    def __init__(self, robot_name, return_message, header=None):
        super(RealtimeMimicResultMessage, self).__init__()
        self["header"] = header or Header()
        # self["human_frames"] = human_frames
        # self["robot_frames"] = robot_frames
        self["robot_name"] = robot_name
        self["return_message"] = return_message

    @classmethod
    def parse(cls, value):
        """Parse the message information
        from the input value
        """
        header = Header.parse(value["header"])
        robot_name = value["robot_name"]
        return_message = value["return_message"]
        return cls(robot_name, return_message, header)
    
class RealtimeMimicIOToggleRequestMessage(Message):
    """
    The RealtimeMimicRequestMessage class is responsible for requesting a robot to mimic a trajectory in real-time.

    The RealtimeMimicRequestMessage class provides methods for parsing, updating, and accessing the fields of a message,
    and provides a means of defining attributes of the message in order to accept or ignore specific messages.

    Parameters
    ----------
    header : Header
        The header of the message.
    """

    def __init__(self, signal, gripper_toggle_bool, header=None):
        super(RealtimeMimicIOToggleRequestMessage, self).__init__()
        self["header"] = header or Header()
        self["signal"] = signal

        if gripper_toggle_bool:
            self["value"] = 1
        else:
            self["value"] = 0

    @classmethod
    def parse(cls, value):
        """Parse the message information
        from the input value
        """
        header = Header.parse(value["header"])
        signal = value["signal"]
        io_value = value["value"]
        if io_value == 0:
            gripper_toggle = False
        elif io_value == 1:
            gripper_toggle = True
        else:
            raise ValueError("Invalid value for IO toggle. Expected 0 or 1.")

        return cls(signal, gripper_toggle, header)

#TODO: Testing Classes..... for inference request and result messages.
class InferenceRequestMessage(Message):
    """
    Request sent from Unity to CAD: current geometry + robot name for inference.
    """

    def __init__(self, current_geometry_frames, robot_name, header=None):
        super(InferenceRequestMessage, self).__init__()
        self["header"] = header or Header()
        # now a dict of name -> Frame, not a list
        self["geometry_frames"] = current_geometry_frames or {}
        self["robot_name"] = robot_name

    # --- helpers ---
    @classmethod
    def _parse_frames_dict_from_data(cls, data):
        """Parse a dict of name->frameData into a dict of name->Frame."""
        if not data:
            return {}
        frames = {}
        for name, frame_data in data.items():
            try:
                frames[name] = Frame.__from_data__(frame_data)
            except Exception as e:
                print(f"InferenceRequestMessage: skipping frame '{name}': {e}")
        return frames

    # --- API ---
    @classmethod
    def parse(cls, value):
        """
        Parse from a dict-like value (already JSON-decoded).
        Missing fields are tolerated and defaulted.
        """
        header_data = value.get("header")
        header = Header.parse(header_data) if header_data else Header()

        frames_data = value.get("geometry_frames", {})
        geometry_frames = cls._parse_frames_dict_from_data(frames_data)

        robot_name = value.get("robot_name")

        if robot_name is None:
            raise ValueError("InferenceRequestMessage missing required field: robot_name")
        if len(geometry_frames) == 0:
            raise ValueError("InferenceRequestMessage missing required field: geometry_frames")

        return cls(geometry_frames, robot_name, header)

class InferenceResultMessage(Message):
    """
    Result returned from CAD to Unity: trajectories, combined points, base frame, robot name, and the inference guess string.
    """

    def __init__(self, inference_guess, trajectories=None, robot_base_frame=None, robot_name=None, header=None):
        super(InferenceResultMessage, self).__init__()
        trajectories = trajectories or []

        self["header"] = header or Header()
        self["trajectories"] = trajectories                           # list[Trajectory]
        self["combined_trajectory_points"] = self._combine_points(trajectories)
        self["robot_base_frame"] = robot_base_frame                   # Frame or None
        self["robot_name"] = robot_name                               # str or None
        self["inference_guess"] = inference_guess                     # str or None

    # --- helpers ---
    def _combine_points(self, trajectories):
        """
        Flattens points from all trajectories. If a trajectory has no points, it contributes nothing.
        Returns a Python list of JointTrajectoryPoint (as your system represents them).
        """
        combined = []
        for traj in (trajectories or []):
            # assuming Trajectory exposes .points (list[JointTrajectoryPoint])
            pts = getattr(traj, "points", None)
            if pts:
                combined.extend(pts)
        return combined

    @classmethod
    def _parse_trajectory_list(cls, data):
        """
        Parse list of trajectory dicts -> List[Trajectory].
        """
        if not data:
            return []
        return [JointTrajectory.__from_data__(t) for t in data]

    # --- API ---
    @classmethod
    def parse(cls, value):
        """
        Parse from a dict-like value (already JSON-decoded).
        All fields are optional; defaults applied if missing.
        """
        header = Header.parse(value.get("header"))

        trajectories_data = value.get("trajectories", [])
        trajectories = cls._parse_trajectory_list(trajectories_data)

        rbf_data = value.get("robot_base_frame")
        robot_base_frame = Frame.__from_data__(rbf_data)

        robot_name = value.get("robot_name")
        inference_guess = value.get("inference_guess")

        return cls(
            inference_guess=inference_guess,
            trajectories=trajectories,
            robot_base_frame=robot_base_frame,
            robot_name=robot_name,
            header=header,
        )

class InferenceReplyMessage(Message):
    """
    Reply from Unity back to CAD acknowledging/accepting/rejecting the inferred goal.
    """

    def __init__(self, goal_status_reply, header=None):
        super(InferenceReplyMessage, self).__init__()
        self["header"] = header or Header()
        self["goal_status_reply"] = int(goal_status_reply) if goal_status_reply is not None else 0

    @classmethod
    def parse(cls, value):
        """
        Parse from dict-like value (already JSON-decoded).
        Missing fields default to safe values.
        """
        header = Header.parse(value.get("header"))
        gsr_raw = value.get("goal_status_reply", 0)
        try:
            goal_status_reply = int(gsr_raw)
        except Exception:
            raise ValueError(f"Invalid goal_status_reply: {gsr_raw!r}")

        return cls(goal_status_reply, header)
