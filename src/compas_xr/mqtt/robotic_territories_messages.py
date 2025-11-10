import sys
import threading
import uuid
from datetime import datetime

from compas.geometry import Frame, Point, Vector
from compas_fab.robots import JointTrajectory, JointTrajectoryPoint
from compas_robots import Configuration
from compas_eve import Message
from enum import IntEnum


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

    #TODO : ADD PICK AND PLACE INDEXES????
    def __init__(self, human_frames, robot_frames, robot_name, io_control_indexes, header=None):
        super(MimicTrajectoryRequestMessage, self).__init__()
        self["header"] = header or Header()
        self["human_frames"] = human_frames
        self["robot_frames"] = robot_frames
        self["robot_name"] = robot_name
        self["io_control_indexes"] = io_control_indexes

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
        io_control_indexes = value.get("io_control_indexes", [])
        if len(io_control_indexes) == 0:
            raise ValueError("MimicTrajectoryRequestMessage missing required field: io_control_indexes")
        return cls(human_frames, robot_frames, robot_name, io_control_indexes, header)
    
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

    def __init__(self, requested_robot_frame, robot_name, point_index, header=None, intial_request=False, is_pick=False, is_place=False, geometry_frame=None):
        super(RealtimeMimicRequestMessage, self).__init__()
        self["header"] = header or Header()
        self["robot_name"] = robot_name
        self["requested_robot_frame"] = requested_robot_frame
        self["point_index"] = point_index
        self["initial_request"] = intial_request
        self["is_pick"] = is_pick
        self["is_place"] = is_place
        self["geometry_frame"] = geometry_frame
    
    @classmethod
    def parse(cls, value):
        """Parse the message information
        from the input value
        """
        header = Header.parse(value["header"])
        requested_robot_frame = Frame.__from_data__(value["requested_robot_frame"])
        robot_name = value["robot_name"]
        pt_index = value["point_index"]
        initial_request = value["initial_request"]
        
        # Optional fields for pick and place
        is_pick = value.get("is_pick", False)
        is_place = value.get("is_place", False)
        geometry_frame_data = value.get("geometry_frame", None)
        if geometry_frame_data:
            geometry_frame = Frame.__from_data__(geometry_frame_data)
        else:
            geometry_frame = None

        return cls(requested_robot_frame, robot_name, pt_index, header, initial_request, is_pick, is_place, geometry_frame)

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

    def __init__(self, robot_name, pt_index, return_message, correct_backend, configuration=None, was_pick_request=False, was_place_request=False, pick_or_place_planning_succeeded=False, header=None):
        super(RealtimeMimicResultMessage, self).__init__()
        self["header"] = header or Header()
        self["point_index"] = pt_index
        self["configuration"] = configuration
        self["robot_name"] = robot_name
        self["return_message"] = return_message
        self["correct_backend"] = correct_backend
        self["was_pick_request"] = was_pick_request
        self["was_place_request"] = was_place_request
        self["pick_or_place_planning_succeeded"] = pick_or_place_planning_succeeded

    @classmethod
    def parse(cls, value):
        """Parse the message information
        from the input value
        """
        header = Header.parse(value["header"])
        robot_name = value["robot_name"]
        return_message = value["return_message"]
        pt_index = value["point_index"]
        correct_backend = value.get("correct_backend", None)
        if correct_backend is None:
            raise ValueError("RealtimeMimicResultMessage missing required field: correct_backend")

        configuration = value.get("configuration", None)
        if configuration is not None:
            configuration = Configuration.__from_data__(configuration)
        else:
            configuration = None
        
        was_pick_or_place = value.get("was_pick_request", False)
        was_place_request = value.get("was_place_request", False)
        pick_or_place_planning_succeeded = value.get("pick_or_place_planning_succeeded", False)

        return cls(robot_name, return_message, correct_backend, configuration, was_pick_or_place, pick_or_place_planning_succeeded, header)
    
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

    def __init__(self, current_geometry_frames, initial_request, robot_name, header=None):
        super(InferenceRequestMessage, self).__init__()
        self["header"] = header or Header()
        # now a dict of name -> Frame, not a list
        self["geometry_frames"] = current_geometry_frames or {}
        self["initial_request"] = initial_request
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
        initial_request = value.get("initial_request", None)
        if initial_request is None:
            raise ValueError("InferenceRequestMessage missing required field: initial_request")

        if robot_name is None:
            raise ValueError("InferenceRequestMessage missing required field: robot_name")
        if len(geometry_frames) == 0:
            raise ValueError("InferenceRequestMessage missing required field: geometry_frames")

        return cls(geometry_frames, initial_request, robot_name, header)

class InferenceResultMessage(Message):
    """
    Result returned from CAD to Unity: trajectories, combined points, base frame, robot name, and the inference guess string.
    """

    def __init__(self, inference_guess=None, suggested_target_name=None, completed_goals_list=[], trajectories=[], robot_base_frame=None, robot_name=None, header=None):
        super(InferenceResultMessage, self).__init__()
        trajectories = trajectories or []

        self["header"] = header or Header()
        self["completed_goals"] = completed_goals_list                     # list[str]
        self["trajectories"] = trajectories                           # list[Trajectory]
        self["combined_trajectory_points"] = self._combine_points(trajectories)
        self["robot_base_frame"] = robot_base_frame                   # Frame or None
        self["robot_name"] = robot_name                               # str or None
        self["inference_guess"] = inference_guess                     # str or None
        self["suggested_target_name"] = suggested_target_name                   # Frame or None

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

        robot_name = value.get("robot_name", None)
        inference_guess = value.get("inference_guess", None)
        completed_goals = value.get("completed_goals", [])

        return cls(
            inference_guess=inference_guess,
            trajectories=trajectories,
            robot_base_frame=robot_base_frame,
            robot_name=robot_name,
            completed_goals_list=completed_goals,
            header=header
        )


class GoalStatusReply(IntEnum):
    REJECT_TARGET_AND_GOAL = 0
    ACCEPT_TARGET_REJECT_GOAL = 1
    ACCEPT_TARGET_AND_GOAL = 2

class InferenceReplyMessage(Message):
    """
    Reply from Unity back to CAD acknowledging/accepting/rejecting the inferred goal.
    """

    def __init__(self, goal_status_reply: GoalStatusReply, current_goal_name:str, suggested_target_name:str, includes_executable_trajectory: bool, header=None):
        super(InferenceReplyMessage, self).__init__()
        self["header"] = header or Header()
        # always store as int for transport
        self["goal_status_reply"] = int(goal_status_reply) if goal_status_reply is not None else 0
        self["includes_executable_trajectory"] = bool(includes_executable_trajectory)
        self["suggested_target_name"] = suggested_target_name
        self["current_goal_name"] = current_goal_name

    @classmethod
    def parse(cls, value: dict):
        """
        Parse from dict-like value (already JSON-decoded).
        Missing fields default to safe values.
        """
        header = Header.parse(value.get("header"))
        gsr_raw = value.get("goal_status_reply", 0)
        includes_executable_trajectory = bool(value.get("includes_executable_trajectory", False))
        suggested_target_name = value.get("suggested_target_name", None)
        current_goal_name = value.get("current_goal_name", None)

        if suggested_target_name is None:
            raise ValueError("InferenceReplyMessage missing required field: suggested_target_name")
        if current_goal_name is None:
            raise ValueError("InferenceReplyMessage missing required field: current_goal_name")

        try:
            goal_status_reply = GoalStatusReply(int(gsr_raw))
        except Exception:
            raise ValueError(f"Invalid goal_status_reply: {gsr_raw!r}")

        return cls(goal_status_reply, current_goal_name, suggested_target_name, includes_executable_trajectory, header)

class PostInferenceTargetRequestMessage(Message):
    def __init__(self, completed_goals_list=None, completed_object_names=None, inference_goal_name=None, target_name=None, robot_name=None, geometry_frames_dict=None, header=None):
        super(PostInferenceTargetRequestMessage, self).__init__()
        self["header"] = header or Header()
        self["completed_goals"] = list(completed_goals_list or [])
        self["completed_object_names"] = list(completed_object_names or [])
        self["inference_goal_name"] = inference_goal_name
        self["target_name"] = target_name
        self["geometry_frames"] = dict(geometry_frames_dict or {})
        self["robot_name"] = robot_name

    @classmethod
    def parse(cls, value):
        header = Header.parse(value.get("header"))

        completed_goals = value.get("completed_goals") or []
        if not isinstance(completed_goals, list):
            completed_goals = []

        completed_object_names = value.get("completed_object_names") or []
        print (f"PostInferenceTargetRequestMessage: Raw Completed Object Names: {completed_object_names}")
        if not isinstance(completed_object_names, list):
            completed_object_names = []
        print (f"PostInferenceTargetRequestMessage: Completed Object Names: {completed_object_names}")

        gf_raw = value.get("geometry_frames") or {}
        geometry_frames = {}
        if isinstance(gf_raw, dict):
            for name, fd in gf_raw.items():
                try:
                    geometry_frames[name] = Frame.__from_data__(fd)
                except Exception:
                    continue

        return cls(
            completed_goals_list=completed_goals,
            completed_object_names=completed_object_names,
            inference_goal_name=value.get("inference_goal_name"),
            target_name=value.get("target_name"),
            robot_name=value.get("robot_name"),
            geometry_frames_dict=geometry_frames,
            header=header,
        )

#TODO: Written by GPT NEEDS FIXED....
class PostInferenceTrajectoryResultMessage(Message):
    def __init__(self, trajectories=None, inference_goal_name=None, target_name=None, robot_base_frame=None, robot_name=None, header=None):
        super(PostInferenceTrajectoryResultMessage, self).__init__()
        trajectories = trajectories or []
        self["header"] = header or Header()
        self["trajectories"] = trajectories
        self["combined_trajectory_points"] = self._combine_points(trajectories)
        self["robot_base_frame"] = robot_base_frame
        self["robot_name"] = robot_name
        self["inference_goal_name"] = inference_goal_name
        self["target_name"] = target_name

    def _combine_points(self, trajectories):
        combined = []
        for traj in (trajectories or []):
            pts = getattr(traj, "points", None)
            if pts:
                combined.extend(pts)
        return combined

    @classmethod
    def _parse_trajectory_list(cls, data):
        if not data:
            return []
        return [JointTrajectory.__from_data__(t) for t in data]

    @classmethod
    def parse(cls, value):
        header = Header.parse(value.get("header"))

        trajectories_data = value.get("trajectories", [])
        trajectories = cls._parse_trajectory_list(trajectories_data)

        rbf_data = value.get("robot_base_frame")
        robot_base_frame = Frame.__from_data__(rbf_data)

        robot_name = value.get("robot_name")
        inference_goal_name = value.get("inference_goal_name")
        target_name = value.get("target_name")

        return cls(
            trajectories=trajectories,
            inference_goal_name=inference_goal_name,
            target_name=target_name,
            robot_base_frame=robot_base_frame,
            robot_name=robot_name,
            header=header,
        )


class PostInferenceExecuteTrajectoryMessage(Message):
    def __init__(self, inference_goal_name=None, target_name=None, robot_name=None, header=None):
        super(PostInferenceExecuteTrajectoryMessage, self).__init__()
        self["header"] = header or Header()
        self["robot_name"] = robot_name
        self["inference_goal_name"] = inference_goal_name
        self["target_name"] = target_name

    @classmethod
    def parse(cls, value):
        header = Header.parse(value.get("header"))
        robot_name = value.get("robot_name")
        inference_goal_name = value.get("inference_goal_name")
        target_name = value.get("target_name")

        return cls(
            inference_goal_name=inference_goal_name,
            target_name=target_name,
            robot_name=robot_name,
            header=header,
        )
