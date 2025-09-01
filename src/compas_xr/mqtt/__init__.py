"""
********************************************************************************
compas_xr.mqtt
********************************************************************************

This package contains classes for interfacing with MQTT protocol.

.. currentmodule:: compas_xr.mqtt

MQTT Messages
-------------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    GetTrajectoryRequest
    GetTrajectoryResult
    ApproveTrajectory
    SendTrajectory

"""

from .messages import ApproveTrajectory, GetTrajectoryRequest, GetTrajectoryResult, SendTrajectory
from.robotic_territories_messages import MimicTrajectoryRequestMessage, MimicTrajectoryResultMessage, ExecuteMimicTrajectoryRequestMessage, RealtimeMimicRequestMessage, RealtimeMimicResultMessage, RealtimeMimicIOToggleRequestMessage, InferenceReplyMessage, InferenceRequestMessage, InferenceResultMessage, PostInferenceExecuteTrajectoryMessage, PostInferenceTargetRequestMessage, PostInferenceTrajectoryResultMessage

__all__ = ["GetTrajectoryRequest", "GetTrajectoryResult", "ApproveTrajectory", "SendTrajectory", "MimicTrajectoryRequestMessage", "MimicTrajectoryResultMessage", "ExecuteMimicTrajectoryRequestMessage", "RealtimeMimicRequestMessage", "RealtimeMimicResultMessage", "RealtimeMimicIOToggleRequestMessage", "InferenceReplyMessage", "InferenceRequestMessage", 'InferenceResultMessage', "PostInferenceExecuteTrajectoryMessage", "PostInferenceTargetRequestMessage", "PostInferenceTrajectoryResultMessage"]