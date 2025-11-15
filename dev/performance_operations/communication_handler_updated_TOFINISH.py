from compas_eve import Subscriber, Publisher, Topic
from compas_eve.mqtt import MqttTransport
from compas_xr.mqtt import RealtimeMimicRequestMessage, RealtimeMimicResultMessage, MimicTrajectoryRequestMessage, MimicTrajectoryResultMessage, ExecuteMimicTrajectoryRequestMessage, RealtimeMimicIOToggleRequestMessage
from compas_xr.mqtt import InferenceRequestMessage, InferenceResultMessage, InferenceReplyMessage, PostInferenceTargetRequestMessage, PostInferenceTrajectoryResultMessage, PostInferenceExecuteTrajectoryMessage

# from robots.com_handlers.realtime_mimic_roshandler import URRealtimeMimicHandler
from robots.com_handlers.realtime_mimic_pbhandler import URMimicHandlerPyB, ABBMimicHandlerPyB #TODO: This needs to be wrapped into one handler for both Mimics
from robots.com_handlers.realtime_mimic_roshandler import URMimicHandlerROS, ABBMimicHandlerROS #TODO: This needs to be wrapped into one handler for both Mimics
from robots.com_handlers.robot_handler_combined_rospy import URMimicHandlerCombined, ABBMimicHandlerCombined
from compas.geometry import Rotation, Translation

from inference.inference_manager import InferenceManager
from robots.planning.play import ScriptedPolicy

from compas.data import json_load, json_dump, json_loads
from compas_fab.robots import JointTrajectory
import os
import math
import time

from compas_xr.realtime_database import RealtimeDatabase
from compas.geometry import Frame, Transformation
from compas_robots import Configuration
import logging
from logger import SimpleLogger

class CommunicationManager:

    def __init__(self, project_name, robot_name, project_config_dict, pick_and_place_xaxis_tolerance, pick_and_place_zaxis_tolerance, participant_name, pybullet_raj_or_joseph="JOSEPH", broker='localhost', mqtt_port=1883, backend_type='PyBullet'):
        __dir_path = os.path.dirname(os.path.realpath(__file__))

        self.mqtt = MqttTransport(broker, mqtt_port)
        self.project_name = project_name
        self.robot_name = robot_name
        self.participant_name = participant_name

        #TODO: Transformation Tolerances to avoid markers and pick and place issues if needed.
        self._PICK_AND_PLACE_XAXIS_TOLERANCE = pick_and_place_xaxis_tolerance
        self._PICK_AND_PLACE_ZAXIS_TOLERANCE = pick_and_place_zaxis_tolerance

        #TODO: This is a bit hacky, but just to create seperate ones.
        if pybullet_raj_or_joseph == "JOSEPH":
            self.connect_joe_to_pybullet = True
            self.connect_raj_to_pybullet = False
        elif pybullet_raj_or_joseph == "RAJ":
            self.connect_joe_to_pybullet = False
            self.connect_raj_to_pybullet = True
        else:
            raise ValueError("Invalid value for pybullet_raj_or_joseph. Use 'JOSEPH' or 'RAJ'.")

        #Robot Loading & Handleing
        _urdf_filepath = os.path.join(__dir_path, project_config_dict["urdf_fps"][robot_name]["urdf"])
        _srdf_filepath = os.path.join(__dir_path, project_config_dict["urdf_fps"][robot_name]["srdf"])
        _robot_hardware_info = project_config_dict["robot_hardware_info"][robot_name]
        _sim_test_start_config = project_config_dict["sim_testing_start_configurations"][robot_name]
        if _sim_test_start_config:
            self.sim_test_start_config = Configuration.__from_data__(_sim_test_start_config)
            print (f"CommunicationManager : Loaded sim test start config for robot {robot_name} of type {type(self.sim_test_start_config)} {_sim_test_start_config}.")
        else:
            self.sim_test_start_config = None
            print (f"CommunicationManager : No sim test start config found for robot {robot_name}.")
        self.handler = self._load_handler(robot_name, _urdf_filepath, _srdf_filepath, _robot_hardware_info, pybullet_connect=self.connect_joe_to_pybullet, backend_type=backend_type)

        #Setting Publishers and Subscriber
        self._set_mimic_publishers_and_subscribers(self.project_name)
        self._set_inference_publishers_and_subscribers(self.project_name)

        #Message Helpers
        self._user_initiated_mimic_trajectories_to_execute = []
        self._user_initiated_io_control_indexes_to_execute = []
        self._post_inference_exacutable_trajectories = []

        #TODO: Testing Simple Logger
        log_folder_fp = os.path.join(__dir_path, project_config_dict["logging_folder_path"], participant_name)
        self._LOGGER = SimpleLogger(dir_path=log_folder_fp, sub_folder_name="communication", participant_name=participant_name)

        #Inference Manager
        goals_folder_fp = os.path.join(__dir_path, project_config_dict["goals_folder_file_path"])
        inference_state_file_path = os.path.join(__dir_path, project_config_dict["inference_state_file_path"], f"{participant_name}_inference_state.json")
        self.inference_manager = InferenceManager(goals_folder_fp, participant_name=participant_name, state_file_path=inference_state_file_path)

        #Scripted Policy for Raj
        if self.connect_raj_to_pybullet:
            self.scripted_policy = ScriptedPolicy(render=True)
        else:
            self.scripted_policy = None

        #Realtime Database for Transformations
        firebase_fp = os.path.join(__dir_path, project_config_dict["firebase_config_fp"])
        print (f"JOE FIREBASE FP: {firebase_fp}")
        self._RTDB_REFERENCE = RealtimeDatabase(firebase_fp)
        self._RTDB_Project_Name = project_config_dict["project_name"]
        self.transformations_reference_list = [self._RTDB_Project_Name, "robot_transformations", robot_name]
        #TODO: This needs to change...
        #Frame Transformations #TODO: I THINK THIS NEEDS TO CHANGE. This strategy only runs once at the beginning... which is not correct if the robot moves.
        self._transformations_file_path = os.path.join(__dir_path, project_config_dict["robot_transformations_fp"])
        (
            self.transformation_ar_space_to_robot_space,
            self.transformations_robot_space_to_ar_space,
            self._urdf_baseframe,
            self._observed_urdf_baseframe
        ) = self._load_transformations(file_path=self._transformations_file_path, robot_name=self.robot_name, transformations_reference_list=self.transformations_reference_list)

    def _load_handler(self, robot_name, urdf_filepath, srdf_filepath, robot_hardware_info_dict, pybullet_connect, backend_type='PyBullet'):
        if robot_name == "UR20" or robot_name == "UR31" or robot_name == "UR32":
            if backend_type == 'COMBINED':
                return URMimicHandlerCombined(robot_name, 
                                                 robot_ip=robot_hardware_info_dict["robot_ip"], 
                                                 urdf_path=urdf_filepath, 
                                                #  pybullet_connect=False, #TODO: RECOMMENT TO RUN JOE BACKEND ONLY ROS.
                                                 pybullet_connect=pybullet_connect,
                                                 participant_name=self.participant_name,
                                                 srdf_path=srdf_filepath,
                                                 ros_ip=robot_hardware_info_dict["ros_ip"],
                                                 ros_port=robot_hardware_info_dict["ros_port"],
                                                 group=robot_hardware_info_dict["group"],
                                                 speed=robot_hardware_info_dict["speed"],
                                                 acceleration=robot_hardware_info_dict["acceleration"],
                                                 radius=robot_hardware_info_dict["radius"],
                                                 nowait=robot_hardware_info_dict["nowait"],
                                                 io=robot_hardware_info_dict["vacum_io"],
                                                 tool_info_fp=robot_hardware_info_dict.get("tool_info_fp"),
                                                 additional_static_collision_meshes_fp=robot_hardware_info_dict.get("additional_collison_meshes_fp"),
                                                 sim_test_start_coifig=self.sim_test_start_config)                
            else:
                raise ValueError(f"Unsupported backend type: {backend_type} for robot {robot_name}")
        elif robot_name == "ABB1" or robot_name == "ABB2" or robot_name == "ABB_IRB4600LL" or robot_name == "ABB_IRB4600LL":
            if backend_type == 'COMBINED':
                return ABBMimicHandlerCombined(robot_name, 
                                                  robot_ip=robot_hardware_info_dict["robot_ip"], 
                                                  urdf_path=urdf_filepath, 
                                                  srdf_path=srdf_filepath,
                                                  pybullet_connect=pybullet_connect,
                                                  ros_ip=robot_hardware_info_dict["ros_ip"],
                                                  ros_port=robot_hardware_info_dict["ros_port"],
                                                  speed=robot_hardware_info_dict["speed"],
                                                  nowait=robot_hardware_info_dict["nowait"],
                                                  tool_info_fp=robot_hardware_info_dict.get("tool_info_fp"),
                                                  additional_static_collision_meshes_fp=robot_hardware_info_dict.get("additional_attached_collison_meshes_fp"))
            else:
                raise ValueError(f"Unsupported backend type: {backend_type} for robot {robot_name}")
        else:
            raise ValueError(f"Unsupported robot name: {robot_name}")

    def _load_transformations(self, file_path, robot_name, transformations_reference_list):
        try:
            transformations_reference = self._RTDB_REFERENCE.construct_reference_from_list(transformations_reference_list)
            print(f"CommunicationManager : [CommunicationManager] Loading Transformatoins From RTDB.")
            (
                transformation_ar_space_to_robot_space,
                transformations_robot_space_to_ar_space,
                _urdf_baseframe,
                _observed_urdf_baseframe
            ) = self._get_robot_transformations_from_rtdb(transformations_reference, robot_name)
            return transformation_ar_space_to_robot_space, transformations_robot_space_to_ar_space, _urdf_baseframe, _observed_urdf_baseframe
        except Exception as e:
            logging.warning(
                f"CommunicationManager: Failed to load transformations from RTDB ({e}). "
                "Loading from file path instead, but the data may be stale..."
            )            
            (
                transformation_ar_space_to_robot_space,
                transformations_robot_space_to_ar_space,
                _urdf_baseframe,
                _observed_urdf_baseframe
            ) = self._load_transformations_from_file_and_baseframes_from_file(file_path=file_path, robot_name=robot_name)
            return transformation_ar_space_to_robot_space, transformations_robot_space_to_ar_space, _urdf_baseframe, _observed_urdf_baseframe

    def _load_transformations_from_file_and_baseframes_from_file(self, file_path, robot_name):
        # Load the transformations from the JSON file
        all_robot_transforms = json_load(file_path)
        if robot_name not in all_robot_transforms:
            raise ValueError(f"CommunicationManager : Robot name '{robot_name}' not found in transformations file.")
        
        robot_transformation = all_robot_transforms[robot_name]["observed"] #TODO: CHECK THIS
        if "inverse_transform_to_observed" not in robot_transformation or "transformation_to_urdf" not in robot_transformation:
            raise ValueError(f"CommunicationManager : Transformations for robot '{robot_name}' are incomplete in the file.")

        # Extract robot_base_frame for transformations back to the real world
        observed_robot_base_frame = robot_transformation["urdf_base_frame"]
        print(f"CommunicationManager : URDF BASEFRAME TYPE: {type(observed_robot_base_frame)}")
        static_urdf_base_frame = all_robot_transforms[robot_name]["static"]["urdf_base_frame"]

        if not static_urdf_base_frame:
            raise ValueError(f"CommunicationManager : Static URDF base frame for '{robot_name}' is not defined in the transformations file.")
        else:
            print(f"CommunicationManager : [CommunicationManager] Loaded static URDF base frame for '{robot_name}': {static_urdf_base_frame}")

        if not observed_robot_base_frame:
            raise ValueError(f"CommunicationManager : Robot base frame for '{robot_name}' is not defined in the transformations file.")
        else:
            print(f"CommunicationManager : [CommunicationManager] Loaded robot base frame for '{robot_name}': {observed_robot_base_frame}")

        inverse_transform = robot_transformation["inverse_transform_to_observed"]
        transform = robot_transformation["transformation_to_urdf"]
        print (f"CommunicationManager : [CommunicationManager] Loaded transformations for robot '{robot_name}' from {file_path}, types: {type(inverse_transform)}, {type(transform)}")
        return inverse_transform, transform, static_urdf_base_frame, observed_robot_base_frame

    def _get_robot_transformations_from_rtdb(self, transformations_reference, robot_name):
        transformatins_dict = self._RTDB_REFERENCE.get_data_from_reference(transformations_reference)
        if not transformatins_dict:
            raise ValueError(f"CommunicationManager : No transformations found in RTDB for robot '{robot_name}' at reference '{transformations_reference}'")
        else:
            print(f"CommunicationManager : [CommunicationManager] Retrieved transformations from RTDB for robot '{robot_name}': {transformatins_dict}")
            (
                transformation_ar_space_to_robot_space,
                transformations_robot_space_to_ar_space,
                _urdf_baseframe,
                _observed_urdf_baseframe
            ) = self._deserialize_transformations_from_database(transformatins_dict, robot_name)
        return transformation_ar_space_to_robot_space, transformations_robot_space_to_ar_space, _urdf_baseframe, _observed_urdf_baseframe

    def _deserialize_transformations_from_database(self, transformation_dict, robot_name):
        robot_transformation = transformation_dict["observed"] #TODO: CHECK THIS
        if "inverse_transform_to_observed" not in robot_transformation or "transformation_to_urdf" not in robot_transformation:
            raise ValueError(f"CommunicationManager : Transformations for robot '{robot_name}' are incomplete in the file.")

        # Extract robot_base_frame for transformations back to the real world
        observed_robot_base_frame = Frame.__from_data__(robot_transformation["urdf_base_frame"]["data"])
        print(f"CommunicationManager : URDF BASEFRAME TYPE: {type(observed_robot_base_frame)}")
        static_urdf_base_frame = Frame.__from_data__(transformation_dict["static"]["urdf_base_frame"]["data"])

        if not static_urdf_base_frame:
            raise ValueError(f"CommunicationManager : Static URDF base frame for '{robot_name}' is not defined in the transformations file.")
        else:
            print(f"CommunicationManager : [CommunicationManager] Loaded static URDF base frame for '{robot_name}': {static_urdf_base_frame}")

        if not observed_robot_base_frame:
            raise ValueError(f"CommunicationManager : Robot base frame for '{robot_name}' is not defined in the transformations file.")
        else:
            print(f"CommunicationManager : [CommunicationManager] Loaded robot base frame for '{robot_name}': {observed_robot_base_frame}")

        inverse_transform = Transformation.__from_data__(robot_transformation["inverse_transform_to_observed"]["data"])
        transform = Transformation.__from_data__(robot_transformation["transformation_to_urdf"]["data"])
        print (f"CommunicationManager : [CommunicationManager] Loaded transformations for robot '{robot_name}', types: {type(inverse_transform)}, {type(transform)}")
        return inverse_transform, transform, static_urdf_base_frame, observed_robot_base_frame

    def __update_robot_transformations_from_rtdb__(self):
        (
            self.transformation_ar_space_to_robot_space,
            self.transformations_robot_space_to_ar_space,
            self._urdf_baseframe,
            self._observed_urdf_baseframe
        ) = self._load_transformations(file_path=self._transformations_file_path, robot_name=self.robot_name, transformations_reference_list=self.transformations_reference_list)

    ######################################################################################################
    # Set Publisers and Subscribers for Inference
    ####################################################################################################

    def _set_inference_publishers_and_subscribers(self, project_name):

        #User Initiated Mimic Request and Result Handlers
        # user_initiated_mimic_result_topic = Topic(f"robotic_territories/mimic_result/{project_name}", MimicTrajectoryResultMessage)
        # self.user_initiated_publisher = Publisher(user_initiated_mimic_result_topic, transport=self.mqtt)

        self.inference_request_topic = Topic(f"robotic_territories/inference_request/{project_name}", InferenceRequestMessage)
        self.inference_request_subscriber = Subscriber(self.inference_request_topic, callback=self._on_handle_inference_request, transport=self.mqtt)
        self.inference_request_subscriber.subscribe()

        self.inference_result_topic = Topic(f"robotic_territories/inference_result/{project_name}", InferenceResultMessage)
        self.inference_result_publisher = Publisher(self.inference_result_topic, transport=self.mqtt)

        self.inference_user_reply_topic = Topic(f"robotic_territories/inference_user_reply/{project_name}", InferenceReplyMessage)
        self.inference_user_reply_subscriber = Subscriber(self.inference_user_reply_topic, callback=self._on_handle_inference_user_reply, transport=self.mqtt)
        self.inference_user_reply_subscriber.subscribe()

        # Topics for sending information after infernce has been compelted
        self.inference_post_inference_request_target_topic = Topic(f"robotic_territories/post_inference_request_target/{project_name}", PostInferenceTargetRequestMessage)
        self.inferenc_post_inference_request_target_subscriber = Subscriber(self.inference_post_inference_request_target_topic, callback=self._on_handle_post_inference_request_target, transport=self.mqtt)
        self.inferenc_post_inference_request_target_subscriber.subscribe()

        self.inference_post_inference_target_request_result_topic = Topic(f"robotic_territories/post_inference_target_result/{project_name}", PostInferenceTrajectoryResultMessage)
        self.inference_post_inference_target_result_publisher = Publisher(self.inference_post_inference_target_request_result_topic, transport=self.mqtt)

        self.inference_post_inference_execute_target_topic = Topic(f"robotic_territories/post_inference_execute_target/{project_name}", PostInferenceExecuteTrajectoryMessage)
        self.inference_post_inference_execute_target_subscriber = Subscriber(self.inference_post_inference_execute_target_topic, callback=self._on_handle_post_inference_execute_target, transport=self.mqtt)
        self.inference_post_inference_execute_target_subscriber.subscribe()

        # inferencePostInferenceRequestTarget = $"robotic_territories/post_inference_request_target/{projectName}";
        # inferencePostInferenceExecuteTargetTopic = $"robotic_territories/post_inference_execute_target/{projectName}";
        # inferencePostInferenceTargetTrajectoryResultTopic = $"robotic_territories/post_inference_target_result/{projectName}";


        print(f"CommunicationManager : [CommunicationManager] Subscribed to: robotic_territories inference topics for project '{project_name}' and robot '{self.robot_name}'")

    def _set_mimic_publishers_and_subscribers(self, project_name):
        #Realtime Mimic Request and Result Handlers
        realtime_mimic_result_topic = Topic(f"robotic_territories/real_time_mimic_result/{project_name}", RealtimeMimicResultMessage)
        self.realtime_publisher = Publisher(realtime_mimic_result_topic, transport=self.mqtt)

        realtime_mimic_request_topic = Topic(f"robotic_territories/real_time_mimic_request/{project_name}", RealtimeMimicRequestMessage)
        self.realtime_subscriber = Subscriber(realtime_mimic_request_topic, callback=self._on_message_realtime_mimic, transport=self.mqtt)
        self.realtime_subscriber.subscribe()

        # Adding IO toggle for Realtime Mimic
        realtime_mimic_io_toggle_request = Topic(f"robotic_territories/real_time_mimic_io_toggle_request/{project_name}", RealtimeMimicIOToggleRequestMessage)
        self.realtime_mimic_io_toggle_request = Subscriber(realtime_mimic_io_toggle_request, callback=self._on_message_realtime_mimic_io_toggle, transport=self.mqtt)
        self.realtime_mimic_io_toggle_request.subscribe()

        #User Initiated Mimic Request and Result Handlers
        user_initiated_mimic_result_topic = Topic(f"robotic_territories/mimic_result/{project_name}", MimicTrajectoryResultMessage)
        self.user_initiated_publisher = Publisher(user_initiated_mimic_result_topic, transport=self.mqtt)

        user_initiated_mimic_request_topic = Topic(f"robotic_territories/mimic_request/{project_name}", MimicTrajectoryRequestMessage)
        self.user_initiated_subscriber = Subscriber(user_initiated_mimic_request_topic, callback=self._on_message_user_initiated_mimic, transport=self.mqtt)
        self.user_initiated_subscriber.subscribe()

        user_initiated_mimic_execution_topic = Topic(f"robotic_territories/mimic_execute_trajectory/{project_name}", ExecuteMimicTrajectoryRequestMessage)
        self.user_initiated_execution_subscriber = Subscriber(user_initiated_mimic_execution_topic, callback=self._on_message_user_initiated_mimic_execution, transport=self.mqtt)
        self.user_initiated_execution_subscriber.subscribe()

        print(f"CommunicationManager : [CommunicationManager] Subscribed to: robotic_territories mimic topics for project '{project_name}' and robot '{self.robot_name}'")

    ######################################################################################################
    # Frame Transformations to Robot Space & AR Space
    ####################################################################################################

    # IN TRANSFORMATIONS ##################################################################################

    def _transform_requested_frame_from_ar_space_to_robot_space(self, frame):
        tx_frame = frame.transformed(self.transformation_ar_space_to_robot_space)
        return tx_frame

    def _transform_requested_frames_list_from_robot_space_to_ar_space(self, frames_list):
        transformed_frames = []
        for frame in frames_list:
            transformed_frame = self._transform_requested_frame_from_ar_space_to_robot_space(frame)
            transformed_frames.append(transformed_frame)
        return transformed_frames

    # OUT TRANSFORMATIONS ##################################################################################

    def _transform_result_frame_from_robot_space_to_ar_space(self, frame):
        tx_frame = frame.transformed(self.transformations_robot_space_to_ar_space)
        return tx_frame

    def _transform_result_frames_list_from_robot_space_to_ar_space(self, frames_list):
        transformed_frames = []
        for frame in frames_list:
            transformed_frame = self._transform_result_frame_from_robot_space_to_ar_space(frame)
            transformed_frames.append(transformed_frame)
        return transformed_frames

    def _transform_inference_information(self, geometry_frames_dict, incompleted_items_names, completed_items_names, target_frame):
        # print(f"CommunicationManager : geometry_frames_dict {geometry_frames_dict}")
        transformed_incompleted_items_dict = {}
        for item_name in incompleted_items_names:
            if item_name in geometry_frames_dict:
                original_frame = geometry_frames_dict[item_name]
                #TODO: I THINK THIS IS THE INVERSE TRANSFORMATION THAT I WANT....
                transformed_frame = self._transform_requested_frame_from_ar_space_to_robot_space(original_frame)
                transformed_incompleted_items_dict[item_name] = transformed_frame
            else:
                print(f"CommunicationManager : [CommunicationManager] Warning: Incompleted item '{item_name}' not found in geometry frames dictionary.")

        transformed_completed_items_dict = {}
        for item_name in completed_items_names:
            if item_name in geometry_frames_dict:
                original_frame = geometry_frames_dict[item_name]
                #TODO: I THINK THIS IS THE INVERSE TRANSFORMATION THAT I WANT....
                transformed_frame = self._transform_requested_frame_from_ar_space_to_robot_space(original_frame)
                transformed_completed_items_dict[item_name] = transformed_frame
            else:
                print(f"CommunicationManager : [CommunicationManager] Warning: Completed item '{item_name}' not found in geometry frames dictionary.")

        #TODO: I THINK THIS IS THE INVERSE TRANSFORMATION THAT I WANT....
        transformed_target_frame = self._transform_requested_frame_from_ar_space_to_robot_space(target_frame)

        return transformed_incompleted_items_dict, transformed_completed_items_dict, transformed_target_frame

    def _transform_post_inference_geometry_frames_dict_to_robot_space(self, geometry_frames_dict, target_frame, completed_goals, completed_object_names):
        incompleted_goals = []
        transformed_incomplete_items_dict = {}
        transformed_completed_items_dict = {}
        print(f"CommunicationManaager : [CommunicatoinManager] Completed Goals: {completed_object_names}")
        for item_name, original_frame in geometry_frames_dict.items():
            print(f"CommunicationManager : [CommunicationManager] Transforming frame for item '{item_name}'")
            transformed_frame = self._transform_requested_frame_from_ar_space_to_robot_space(original_frame)
            if item_name not in completed_object_names:
                print (f"CommunicationManager : [CommunicationManager] Item '{item_name}' is incomplete. Adding to incompleted items.")
                incompleted_goals.append(item_name)
                transformed_incomplete_items_dict[item_name] = transformed_frame
            else:
                transformed_completed_items_dict[item_name] = transformed_frame
                print (f"CommunicationManager : [CommunicationManager] Item '{item_name}' is completed. Adding to completed items.")

        transformed_target_frame = self._transform_requested_frame_from_ar_space_to_robot_space(target_frame)
        return transformed_completed_items_dict, transformed_incomplete_items_dict, transformed_target_frame, incompleted_goals

    # HELPER TRANSFORMATIONS ##################################################################################

    def _offset_frame_along_vector(self, frame, vector, distance):
        translation_vector = vector.unitized() * distance
        translation = Translation.from_vector(translation_vector)
        offsetted_frame = frame.transformed(translation)
        return offsetted_frame

    def _accomodate_pick_and_place_tolerances(self, pick_frame, place_frame):
        # Accommodate X-Axis Tolerance
        if self._PICK_AND_PLACE_XAXIS_TOLERANCE is not None:
            pick_x_axis = pick_frame.xaxis
            place_x_axis = place_frame.xaxis
            pick_frame = self._offset_frame_along_vector(pick_frame, pick_x_axis, self._PICK_AND_PLACE_XAXIS_TOLERANCE)
            place_frame = self._offset_frame_along_vector(place_frame, place_x_axis, self._PICK_AND_PLACE_XAXIS_TOLERANCE)

        # Accommodate Z-Axis Tolerance
        if self._PICK_AND_PLACE_ZAXIS_TOLERANCE is not None:
            pick_z_axis = pick_frame.zaxis
            place_z_axis = place_frame.zaxis
            pick_frame = self._offset_frame_along_vector(pick_frame, pick_z_axis, self._PICK_AND_PLACE_ZAXIS_TOLERANCE)
            place_frame = self._offset_frame_along_vector(place_frame, place_z_axis, self._PICK_AND_PLACE_ZAXIS_TOLERANCE)

        return pick_frame, place_frame

    ###################################################################################################
    # Inference Target Finding Helpers
    ###################################################################################################

    def _find_closest_incomplete_target_for_inference(self, incompleted_items_dict, target_frame): #TODO: This should find the closest incomplete to the robot actually.
        if not incompleted_items_dict:
            print("CommunicationManager : [CommunicationManager] No incompleted items provided for finding closest target.")
            return None, None

        closest_item_name = None
        closest_item_frame = None
        min_distance = float('inf')

        for item_name, item_frame in incompleted_items_dict.items():
            distance = item_frame.point.distance_to_point(target_frame.point)
            if distance < min_distance:
                min_distance = distance
                closest_item_name = item_name
                closest_item_frame = item_frame

        if closest_item_name is None:
            print("CommunicationManager : [CommunicationManager] No closest target found among incompleted items.")
            return None, None

        print(f"CommunicationManager : [CommunicationManager] Closest target found: {closest_item_name} at distance {min_distance}")
        return closest_item_name, closest_item_frame

    ###################################################################################################
    # Inference RL Trajectory HELPERS #TODO: This subsamples the trajectory to not be so massive...
    ###################################################################################################

    def _subsample_trajectory(self, trajectory, modulus=5):
        if not trajectory or not trajectory.points:
            return []
        points = trajectory.points
        subsampled = points[::modulus]
        if subsampled[-1] is not points[-1]:
            subsampled.append(points[-1])
        joint_trajectory = JointTrajectory(trajectory_points=subsampled, start_configuration=trajectory.start_configuration, attached_collision_meshes=trajectory.attached_collision_meshes)
        return joint_trajectory

    ######################################################################################################
    # Message Handlers for Realtime Mimic and User Initiated Mimic
    ####################################################################################################

    def _on_message_realtime_mimic(self, msg: RealtimeMimicRequestMessage):
        robot_name = msg.robot_name

        print(f"CommunicationManager : [CommunicationManager] Received Relatime Mimic request for robot '{robot_name}': {msg}")

        #TODO: Logging Test
        self._LOGGER.log_message(message=msg)

        handler = self.handler
        # self._save_requested_frame(msg)

        if self.connect_raj_to_pybullet == True:
            result = RealtimeMimicResultMessage(
                robot_name=robot_name,
                return_message=f"IK cannot be computed because it is the incorrect backend for {msg.point_index} with {robot_name}",
                configuration=None,
                pt_index=msg.point_index,
                correct_backend=False
            )
            print(f" CommunicationManager : [CommunicationManager] Signaling Application that Pybullet backend needs to change")

        else:
            #TODO: Testing Updating the base frame dynamically....
            if msg.point_index == 0:
                self.__update_robot_transformations_from_rtdb__()

            msg.requested_robot_frame = self._transform_requested_frame_from_ar_space_to_robot_space(msg.requested_robot_frame)
            # ik_config = self.handler.handle_realtime_msg_request_ik_target(msg)
            # ik_config = self.handler.handle_realtime_msg_request_compas_fab_itter(msg)
            # ik_config = self.handler.handle_realtime_msg_request_recursive_solver(msg)
            # ik_config = self.handler.handle_realtime_msg_request(msg)
            # ik_config = handler.handle_realtime_msg_request_fastest_ik(msg)
            if not (msg.is_pick or msg.is_place):

                ik_config = handler.handle_realtime_msg_request_servoj_gate(msg)

                if ik_config:
                    result = RealtimeMimicResultMessage(
                        robot_name=robot_name,
                        return_message=f"IK computed for {msg.point_index} with {robot_name} and an ik solution of {ik_config}",
                        configuration=ik_config,
                        pt_index=msg.point_index,
                        correct_backend=True
                    )
                    print(f"CommunicationManager : [CommunicationManager] Published IK result for robot {robot_name}")
                else:
                    result = RealtimeMimicResultMessage(
                        robot_name=robot_name,
                        return_message=f"IK computed for {msg.point_index} with {robot_name} and an ik solution of {ik_config}",
                        configuration=None,
                        pt_index=msg.point_index,
                        correct_backend=True
                    )
                    print(f" CommunicationManager : [CommunicationManager] No result to publish for robot {robot_name}")

            elif msg.is_pick:
                print(f"CommunicationManager : [CommunicationManager] PICK operation detected. Attemptign PICK Planning")
                msg.geometry_frame = self._transform_requested_frame_from_ar_space_to_robot_space(msg.geometry_frame)
                trajectories = handler.handle_realtime_mimic_request_pick(msg)
                if len(trajectories) > 0:
                    print(f"CommunicationManager : [CommunicationManager] PICK operation PLANNING SUCCEEDED.")
                    last_trajectory_point = trajectories[-1].points[-1] #TODO: I think I will need to turn this into a Configuration...
                    ik_config = Configuration(last_trajectory_point.joint_values, last_trajectory_point.joint_types, last_trajectory_point.joint_names)
                    result = RealtimeMimicResultMessage(
                        robot_name=robot_name,
                        return_message=f"IK computed for {msg.point_index} with {robot_name} and an ik solution of {ik_config}",
                        configuration=ik_config,
                        pt_index=msg.point_index,
                        correct_backend=True,
                        was_pick_request=True,
                        was_place_request=False,
                        pick_or_place_planning_succeeded=True
                    )
                else:
                    print(f"CommunicationManager : [CommunicationManager] PICK operation PLANNING FAILED.")
                    ik_config = None
                    result = RealtimeMimicResultMessage(
                        robot_name=robot_name,
                        return_message=f"IK computed for {msg.point_index} with {robot_name} and an ik solution of {ik_config}",
                        configuration=ik_config,
                        pt_index=msg.point_index,
                        correct_backend=True,
                        was_pick_request=True,
                        was_place_request=False,
                        pick_or_place_planning_succeeded=False
                    )
            elif msg.is_place:
                print(f"CommunicationManager : [CommunicationManager] PLACE operation detected. Attemptign PLACE Planning.")
                msg.geometry_frame = self._transform_requested_frame_from_ar_space_to_robot_space(msg.geometry_frame)
                trajectories = handler.handle_realtime_mimic_request_place(msg)
                if len(trajectories) > 0:
                    print(f"CommunicationManager : [CommunicationManager] PLACE operation PLANNING SUCCESS.")
                    last_trajectory_point = trajectories[-1].points[-1] #TODO: I think I will need to turn this into a Configuration...
                    ik_config = Configuration(last_trajectory_point.joint_values, last_trajectory_point.joint_types, last_trajectory_point.joint_names)
                    result = RealtimeMimicResultMessage(
                        robot_name=robot_name,
                        return_message=f"IK computed for {msg.point_index} with {robot_name} and an ik solution of {ik_config}",
                        configuration=ik_config,
                        pt_index=msg.point_index,
                        correct_backend=True,
                        was_pick_request=False,
                        was_place_request=True,
                        pick_or_place_planning_succeeded=True
                    )
                else:
                    print(f"CommunicationManager : [CommunicationManager] PLACE operation PLANNING FAILED.")
                    ik_config = None
                    result = RealtimeMimicResultMessage(
                        robot_name=robot_name,
                        return_message=f"IK computed for {msg.point_index} with {robot_name} and an ik solution of {ik_config}",
                        configuration=ik_config,
                        pt_index=msg.point_index,
                        correct_backend=True,
                        was_pick_request=True,
                        was_place_request=True,
                        pick_or_place_planning_succeeded=False
                    )
        #TODO: Logging Test
        self._LOGGER.log_message(message=result)
        self.realtime_publisher.publish(result)

    def _on_message_user_initiated_mimic(self, msg: MimicTrajectoryRequestMessage):
        #TODO: Testing updating base frames dynamically
        self.__update_robot_transformations_from_rtdb__()

        # TODO: Set this to an empty list just to be safe... This will be my attribute for storing the trajectories to execute.
        self._user_initiated_mimic_trajectories_to_execute = []
        self._user_initiated_io_control_indexes_to_execute = []

        robot_name = msg.robot_name
        requested_human_frames = msg.human_frames
        requested_robot_frames = msg.robot_frames
        requested_io_signals = msg.io_control_indexes

        print(f"CommunicationManager : [CommunicationManager] Received User Controled Mimic request for robot '{robot_name}': Requesting : {len(requested_robot_frames)} frames")
        # print(f"CommunicationManager : [CommunicationManager] Received User Controled Mimic request for robot '{robot_name}': Requesting : {len(requested_robot_frames)} frames : msg : {msg}")

        # TODO: Remove this logging afer running some tests...
        data={}
        data["robot_name"] = robot_name
        data["requested_robot_frames"] = msg.robot_frames
        data["requested_human_frames"] = msg.human_frames
        data["io_signals"] = msg.io_control_indexes
        json_dump(data=data, fp=r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\random_data_saves\test_requested_robot_frames.json", pretty=True)

        #Transform the frames and reassign them to the message.
        transformed_requested_robot_frames = self._transform_requested_frames_list_from_robot_space_to_ar_space(requested_robot_frames)

        #TODO: Accomodate pick and place tolerances here if needed.
        #TODO: JOSEPH - OFFSETTING FRAMES HERE IF TOLERANCE ISSUES ARISE.
        offset_distance = -0.012
        offset_frames = []
        for frame in transformed_requested_robot_frames:
            print(f"CommunicationManager : Offsetting by {offset_distance} cm for : {frame}")
            offset_frame = self._offset_frame_along_vector(frame, frame.zaxis, offset_distance)
            offset_frames.append(offset_frame)
        transformed_requested_robot_frames = offset_frames
        #TODO: Accomodate pick and place tolerances here if needed.

        msg.robot_frames = transformed_requested_robot_frames

        #TODO: Logging Test
        self._LOGGER.log_message(message=msg)

        handler = self.handler
        # trajectories_list = handler.handle_user_iniated_msg_request(msg)
        trajectories_list = handler.handle_user_iniated_msg_request_ROS(msg)

        if trajectories_list:
            trajectories_to_publsih = trajectories_list
            print(f"CommunicationManager : [CommunicationManager] Received trajectories for robot '{robot_name}': {trajectories_to_publsih}")
        else:
            trajectories_to_publsih = []
            print(f"CommunicationManager : [CommunicationManager] No trajectories received for robot '{robot_name}'.")

        # Robot base frame transformation
        #TODO: Tranformation is from the URDF baseframe to make sure that everything is correct with the urdf baseframe to the real world. (also where I can add extra transformatoin if needed because of the poor structure of some URDFs)
        #TODO: TESTING THIS...
        # robot_base_frame = self._transform_result_frame_from_robot_space_to_ar_space(self._urdf_baseframe)
        _urdf_baseframe = self._urdf_baseframe
        rotation = Rotation.from_axis_and_angle(_urdf_baseframe.zaxis, math.radians(180), _urdf_baseframe.point)
        rotated_frame = _urdf_baseframe.transformed(rotation)
        robot_base_frame = rotated_frame.transformed(self.transformations_robot_space_to_ar_space)
        #TODO: Added Additional Transformation Rotation to account for poor URDF baseframe placement.

        self._user_initiated_mimic_trajectories_to_execute = trajectories_to_publsih
        self._user_initiated_io_control_indexes_to_execute = requested_io_signals

        result = MimicTrajectoryResultMessage(
            robot_name=self.robot_name,
            trajectories=trajectories_to_publsih,
            robot_base_frame=robot_base_frame,
        )

        # random_fp_save = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\random_trajectory_for_testing.json"
        # json_dump(data=result, fp=random_fp_save, pretty=True)
        self._LOGGER.log_message(message=result)
        self.user_initiated_publisher.publish(result)
        print(f"CommunicationManager : [CommunicationManager] Published result with {len(trajectories_to_publsih)} trajectories for robot {robot_name}")

    def _on_message_user_initiated_mimic_execution(self, msg: ExecuteMimicTrajectoryRequestMessage):
        robot_name = msg.robot_name
        self._LOGGER.log_message(message=msg)
        print(f"CommunicationManager : [CommunicationManager] Received User Controled Mimic execution request for robot '{robot_name}' with {len(self._user_initiated_mimic_trajectories_to_execute)} trajectories to execute.")

        handler = self.handler
        success = handler.handle_user_initiated_mimic_execution(msg, self._user_initiated_mimic_trajectories_to_execute, self._user_initiated_io_control_indexes_to_execute)

        if success:
            print(f"CommunicationManager : [CommunicationManager] Successfully executed mimic trajectory for robot {robot_name}")
        else:
            print(f"CommunicationManager : [CommunicationManager] Failed to execute mimic trajectory for robot {robot_name}")

    def _on_message_realtime_mimic_io_toggle(self, msg: RealtimeMimicIOToggleRequestMessage):
        # robot_name = msg.robot_name
        signal = msg.signal
        value = msg.value
        handler = self.handler
        handler.handle_realtime_mimic_io_toggle_request(msg=msg)

        self._LOGGER.log_message(message=msg)
        print(f"CommunicationManager : [CommunicationManager] Processed IO toggle for signal '{signal}' with value '{value}'")

    ######################################################################################################
    # Message Handlers for Inference Requests and Results
    ####################################################################################################
    
    def _on_handle_inference_request(self, msg: InferenceRequestMessage):
        #TODO: testing updating base frames dynamically
        self.__update_robot_transformations_from_rtdb__()

        self._LOGGER.log_message(message=msg)

        # Clear any previous executable trajectories (just to be safe)
        self._INFERENCE_EXACUTABLE_TRAJECTORIES = []

        robot_name = msg.robot_name
        geometry_frames_for_inference = msg.geometry_frames
        print(f"CommunicationManager : [CommunicationManager] Received Inference request for robot '{robot_name}': Requesting : {len(geometry_frames_for_inference)} frames. Initial request: {msg.initial_request}")
        if len(geometry_frames_for_inference) == 0:
            #TODO: This should actually return a none messgage.
            raise ValueError("Inference request contains no geometry frames.")
        
        inference_result_dict = self.inference_manager.handle_inference_request(geometry_frames_for_inference, initial_request=msg.initial_request)
        self._LOGGER.log_message(message=inference_result_dict)

        if inference_result_dict == None:
            print(f"CommunicationManager : [CommunicationManager] Inference manager returned no result for robot '{robot_name}'.")
            self.inference_result_publisher.publish(InferenceResultMessage(
                inference_guess=None,
                suggested_target_name=None,
                completed_goals_list=[],
                trajectories=[],
                robot_base_frame=[],
                robot_name=robot_name
            ))
            return

        print (f"Inference Result Dict: {inference_result_dict}")
        suggested_goal = inference_result_dict["suggested_goal"]
        completed_goal_names = inference_result_dict["completed_goals"]
        suggested_target_frame = inference_result_dict["suggested_target"]
        completed_items_names = inference_result_dict["completed_items"]
        incompleted_items_names = inference_result_dict["incompleted_items"]
        suggested_target_name = inference_result_dict["suggested_target_name"]
        print(f"CommunicationManager : [CommunicationManager] Inference suggested goal: {suggested_goal}, completed goals: {completed_goal_names}, target frame: {suggested_target_frame}, completed items: {completed_items_names}, incompleted items: {incompleted_items_names}")

        transformed_incompleted_items_dict, transformed_completed_items_dict, transformed_target = self._transform_inference_information(geometry_frames_for_inference, incompleted_items_names, completed_items_names, suggested_target_frame)
        closest_target_name, closest_target_frame = self._find_closest_incomplete_target_for_inference(transformed_incompleted_items_dict, transformed_target)

        if self._PICK_AND_PLACE_XAXIS_TOLERANCE is not None or self._PICK_AND_PLACE_ZAXIS_TOLERANCE is not None:
            closest_target_frame, transformed_target = self._accomodate_pick_and_place_tolerances(closest_target_frame, transformed_target)

        #TODO: UPDATED BY JOSEPH
        handler = self.handler
        if self.connect_joe_to_pybullet:
            trajectories = handler.handle_planning_for_inference(closest_target_frame, transformed_target, transformed_completed_items_dict, transformed_incompleted_items_dict, closest_target_name)
            traj_len = len(trajectories)
        elif self.connect_raj_to_pybullet:
            attached_collision_meshes_list = handler.ros_robot.get_attached_tool_collision_meshes()
            ee_collision_mesh = attached_collision_meshes_list[0] if attached_collision_meshes_list else None
            trajectories, pick_index = self.scripted_policy.plan_pick_and_place_joe_wrapper(closest_target_frame, transformed_target, attached_collision_mesh=ee_collision_mesh)
            traj_len = len(trajectories.points)
            if traj_len > 100:
                vis_trajectory = self._subsample_trajectory(trajectories, modulus=3)
            print (f"CommunicationManager : [CommunicationManager] Raj planned {traj_len} trajectory points for inference.")

        if traj_len == 0:
            print(f"CommunicationManager : [CommunicationManager] No trajectories computed for inference request for robot '{robot_name}'.")
            failed_message = InferenceResultMessage(
                    inference_guess=suggested_goal,
                    completed_goals_list=completed_goal_names,
                    suggested_target_name=suggested_target_name,
                    trajectories=[],
                    robot_base_frame=[],
                    robot_name=robot_name
                )
            self._LOGGER.log_message(message=failed_message)
            self.inference_result_publisher.publish(failed_message)
            return
        #TODO: Wrap this for RAJ
        elif self.connect_raj_to_pybullet:
            trajectories = [trajectories]  # Wrap single trajectory in a list for Raj
            vis_trajectory = [vis_trajectory] if traj_len > 100 else trajectories
        elif self.connect_joe_to_pybullet:
            trajectories = trajectories
            vis_trajectory = trajectories

        #TODO: Tranformation is from the URDF baseframe to make sure that everything is correct with the urdf baseframe to the real world. (also where I can add extra transformatoin if needed because of the poor structure of some URDFs)
        #TODO: TESTING THIS...
        # fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\20250830_json_testing_trajectory_dumps\inf_trajectory_dump.json"
        # json_dump(data=trajectories, fp=fp, pretty=True)

        # robot_base_frame = self._transform_result_frame_from_robot_space_to_ar_space(self._urdf_baseframe)
        _urdf_baseframe = self._urdf_baseframe
        rotation = Rotation.from_axis_and_angle(_urdf_baseframe.zaxis, math.radians(180), _urdf_baseframe.point)
        rotated_frame = _urdf_baseframe.transformed(rotation)
        robot_base_frame = rotated_frame.transformed(self.transformations_robot_space_to_ar_space)
        #TODO: Added Additional Transformation Rotation to account for poor URDF baseframe placement.

        print(f"CommunicationManager : [CommunicationManager] Computed {len(trajectories)} trajectories for inference request for robot '{robot_name}'.")
        result = InferenceResultMessage(
            inference_guess=suggested_goal,
            suggested_target_name=suggested_target_name,
            completed_goals_list=completed_goal_names,
            trajectories=vis_trajectory,
            robot_base_frame=robot_base_frame,
            robot_name=robot_name
        )

        if self.connect_raj_to_pybullet:
            self._INFERENCE_EXACUTABLE_TRAJECTORIES_RAJ = trajectories
            self._INFERENCE_PICK_INDEX_RAJ = pick_index
        elif self.connect_joe_to_pybullet:
            self._INFERENCE_EXACUTABLE_TRAJECTORIES = trajectories

        self._LOGGER.log_message(message=result)
        self.inference_result_publisher.publish(result)
        print(f"CommunicationManager : [CommunicationManager] Published inference result with {len(trajectories)} trajectories for robot {robot_name}")

    def _on_handle_inference_user_reply(self, msg: InferenceReplyMessage):
        # robot_name = msg.robot_name
        user_reply = msg.goal_status_reply
        print(f"CommunicationManager : [CommunicationManager] msg : {msg}")
        self.inference_manager._process_user_reply(goal_name=msg.current_goal_name, suggested_target_name=msg.suggested_target_name, goal_status_reply=msg.goal_status_reply, timestamp=msg.header.time_stamp)

        self._LOGGER.log_message(message=msg)

        if user_reply == 0:
            print(f"CommunicationManager : [CommunicationManager] Reject Goal & Target reply from User : {msg.header.device_id} ': Reply : {user_reply}")
        elif user_reply == 1:
            if msg.includes_executable_trajectory:
                #TODO: THIS IS FOR FIXING WITH RAJ AND JOSEPH'S CODE
                if self.connect_joe_to_pybullet:
                    if len(self._INFERENCE_EXACUTABLE_TRAJECTORIES) > 0:
                        handler = self.handler
                        handler._execute_inference_pick_and_place(self._INFERENCE_EXACUTABLE_TRAJECTORIES)
                    else:
                        raise ValueError("No executable trajectories available to execute for inference pick-and-place.")
                    print(f"CommunicationManager : [CommunicationManager] Accept Target and Reject Goal with Executable Trajectory reply from User : {msg.header.device_id} ': Reply : {user_reply}")
                elif self.connect_raj_to_pybullet:
                    if len(self._INFERENCE_EXACUTABLE_TRAJECTORIES_RAJ) > 0:
                        handler = self.handler
                        handler._execute_inference_pick_and_place_raj(self._INFERENCE_EXACUTABLE_TRAJECTORIES_RAJ, self._INFERENCE_PICK_INDEX_RAJ)
                    else:
                        raise ValueError("No executable trajectories available to execute for inference pick-and-place.")
                    print(f"CommunicationManager : [CommunicationManager] Accept Target and Reject Goal with Executable Trajectory reply from User : {msg.header.device_id} ': Reply : {user_reply}")
                else:
                    raise ValueError("Neither Joe nor Raj is connected to PyBullet for executing trajectories.")
            else:
                print(f"CommunicationManager : [CommunicationManager] Accept Target and Reject Goal without Executable Trajectory reply from User : {msg.header.device_id} ': Reply : {user_reply}")
        elif user_reply == 2:
            if msg.includes_executable_trajectory:
                #TODO: THIS IS FOR FIXTING WITH RAJ AND JOSEPH'S CODE
                if self.connect_joe_to_pybullet:
                    print(f"CommunicationManager : [CommunicationManager] Accept Target and Goal with Executable Trajectory reply from User : {msg.header.device_id} ': Reply : {user_reply}")
                    if len(self._INFERENCE_EXACUTABLE_TRAJECTORIES) > 0:
                        handler = self.handler
                        handler._execute_inference_pick_and_place(self._INFERENCE_EXACUTABLE_TRAJECTORIES)
                    else:
                        raise ValueError("No executable trajectories available to execute for inference pick-and-place.")
                elif self.connect_raj_to_pybullet:
                    print(f"CommunicationManager : [CommunicationManager] Accept Target and Goal with Executable Trajectory reply from User : {msg.header.device_id} ': Reply : {user_reply}")
                    if len(self._INFERENCE_EXACUTABLE_TRAJECTORIES_RAJ) > 0:
                        handler = self.handler
                        handler._execute_inference_pick_and_place_raj(self._INFERENCE_EXACUTABLE_TRAJECTORIES_RAJ, self._INFERENCE_PICK_INDEX_RAJ)
                    else:
                        raise ValueError("No executable trajectories available to execute for inference pick-and-place.")
                else:
                    raise ValueError("Neither Joe nor Raj is connected to PyBullet for executing trajectories.")    
            else:
                print(f"CommunicationManager : [CommunicationManager] Accept Target and Goal without Executable Trajectory reply from User : {msg.header.device_id} ': Reply : {user_reply}")

    def _on_handle_post_inference_request_target(self, msg: PostInferenceTargetRequestMessage):
        #Todo: testing updating base frames dynamically
        self.__update_robot_transformations_from_rtdb__()

        self._LOGGER.log_message(message=msg)

        if len(self._post_inference_exacutable_trajectories) > 0 :
            self._post_inference_exacutable_trajectories = []
            self._POST_INFERENCE_PICK_INDEX_RAJ = None

        robot_name = msg.robot_name
        geometry_frames_dict = msg.geometry_frames
        target_frame_name = msg.target_name
        completed_goals = msg.completed_goals
        goal_name = msg.inference_goal_name
        completed_object_names = msg.completed_object_names

        print (f"JOEEEEEEEE : Message Completed Object Names: {completed_object_names}, Message Completed Goals: {completed_goals}, Goal Name: {goal_name}, Target Frame Name: {target_frame_name}")

        print(f"CommunicationManager : [CommunicationManager] Received Post Inference Target request for robot '{robot_name}': Goal Name: {goal_name}, Target Name: {target_frame_name}, Completed Goals: {completed_goals}, Number of Geometry Frames: {len(geometry_frames_dict)}")
        goal_is_correct, transformed_target_frame = self.inference_manager._validate_and_compute_target_location(goal_name, geometry_frames_dict, target_frame_name)
        if not goal_is_correct:
            print(f"CommunicationManager : [CommunicationManager] Inference goal '{goal_name}' is not correct based on current geometry frames and completed goals.")
            self.inference_post_inference_target_result_publisher.publish(PostInferenceTrajectoryResultMessage(
                robot_name=robot_name,
                trajectories=[],
                robot_base_frame=[],
                inference_goal_name=goal_name,
                target_name=target_frame_name,
            ))
            return
        else:
            print(f"CommunicationManager : [CommunicationManager] Inference goal '{goal_name}' validated successfully.")
        print (f"Message Completed Goals: {msg.completed_goals}, Goal Name: {msg.inference_goal_name}, Target Frame: {transformed_target_frame}")
        #TODO: Kind of hacky, but just need to return an incompleted_goals list for the transformation function.
        transformed_completed_items_dict, transformed_incomplete_items_dict, transformed_target_frame, incompleted_goals = self._transform_post_inference_geometry_frames_dict_to_robot_space(geometry_frames_dict, transformed_target_frame, completed_goals, completed_object_names)
        print (f"CommunicationManager : [CommunicationManager] Completed Goals: {completed_goals}, incompleted_goals: {incompleted_goals}]")
        print (f"CommunicationManager : [CommunicationManager] Transformed Completed Items Dict Keys: {list(transformed_completed_items_dict.keys())}, Transformed Incomplete Items Dict Keys: {list(transformed_incomplete_items_dict.keys())}]")
        closest_item_name, closest_item_frame = self._find_closest_incomplete_target_for_inference(transformed_incomplete_items_dict, transformed_target_frame)

        # data = {}
        # data["transformed_completed_items_dict"] = {k: v.to_dict() for k, v in transformed_completed_items_dict.items()}
        # data["transformed_incomplete_items_dict"] = {k: v.to_dict() for k, v in transformed_incomplete_items_dict.items()}
        # data["transformed_target_frame"] = transformed_target_frame.to_dict()
        # data["closest_item_name"] = closest_item_name
        # data["closest_item_frame"] = closest_item_frame.to_dict() if closest_item_frame else None
        # self._LOGGER.log(message=data)

        #TODO: Accomodate Tolerances
        if self._PICK_AND_PLACE_XAXIS_TOLERANCE is not None or self._PICK_AND_PLACE_ZAXIS_TOLERANCE is not None:
            closest_item_frame, transformed_target_frame = self._accomodate_pick_and_place_tolerances(closest_item_frame, transformed_target_frame)

        if closest_item_name is None or closest_item_frame is None:
            print(f"CommunicationManager : [CommunicationManager] No closest target found for post-inference request for robot '{robot_name}'.")
            post_inf_no_target_result = PostInferenceTrajectoryResultMessage(
                robot_name=robot_name,
                trajectories=[],
                robot_base_frame=[],
                inference_goal_name=goal_name,
                target_name=target_frame_name,
            )
            self._LOGGER.log(message=post_inf_no_target_result)
            self.inference_post_inference_target_result_publisher.publish(post_inf_no_target_result)
            return

        print(f"CommunicationManager : [CommunicationManager] Closest target for post-inference request: {closest_item_name}")
        handler = self.handler
        if self.connect_joe_to_pybullet:
            trajectories = handler.handle_planning_for_inference(closest_item_frame, transformed_target_frame, transformed_completed_items_dict, transformed_incomplete_items_dict, closest_item_name, post_inference=True)
            traj_len = len(trajectories)
        elif self.connect_raj_to_pybullet:
            attached_collision_meshes_list = handler.ros_robot.get_attached_tool_collision_meshes()
            ee_collision_mesh = attached_collision_meshes_list[0] if attached_collision_meshes_list else None
            trajectories, pick_index = self.scripted_policy.plan_pick_and_place_joe_wrapper(closest_item_frame, transformed_target_frame, attached_collision_mesh=ee_collision_mesh)
            traj_len = len(trajectories.points)
            if traj_len > 100:
                vis_trajectory = self._subsample_trajectory(trajectories, modulus=3)
            print (f"CommunicationManager : [CommunicationManager] Raj planned {traj_len} trajectory points for inference.")

        if traj_len == 0:
            print(f"CommunicationManager : [CommunicationManager] No trajectories computed for post-inference request for robot '{robot_name}'.")
            post_inf_failed_trajectory = PostInferenceTrajectoryResultMessage(
                robot_name=robot_name,
                trajectories=[],
                robot_base_frame=[],
                inference_goal_name=goal_name,
                target_name=target_frame_name,
            )
            self._LOGGER.log_message(message=post_inf_failed_trajectory)
            self.inference_post_inference_target_result_publisher.publish(post_inf_failed_trajectory)
            return
        
        #TODO: Wrap this for RAJ
        elif self.connect_raj_to_pybullet:
            trajectories = [trajectories]  # Wrap single trajectory in a list for Raj
            vis_trajectory = [vis_trajectory] if traj_len > 100 else trajectories
        elif self.connect_joe_to_pybullet:
            trajectories = trajectories
            vis_trajectory = trajectories

        print(f"CommunicationManager : [CommunicationManager] Computed {len(trajectories)} trajectories for post-inference request for robot '{robot_name}'.")
        # Robot base frame transformation
        _urdf_baseframe = self._urdf_baseframe
        rotation = Rotation.from_axis_and_angle(_urdf_baseframe.zaxis, math.radians(180), _urdf_baseframe.point)
        rotated_frame = _urdf_baseframe.transformed(rotation)
        robot_base_frame = rotated_frame.transformed(self.transformations_robot_space_to_ar_space)

        result = PostInferenceTrajectoryResultMessage(
            robot_name=robot_name,
            trajectories=vis_trajectory,
            robot_base_frame=robot_base_frame,
            inference_goal_name=goal_name,
            target_name=target_frame_name,
        )
        self.inference_post_inference_target_result_publisher.publish(result)
        print(f"CommunicationManager : [CommunicationManager] Published Post Inference Target result with {len(trajectories)} trajectories for robot {robot_name}")

        if self.connect_raj_to_pybullet:
            self._post_inference_exacutable_trajectories = trajectories
            self._POST_INFERENCE_PICK_INDEX_RAJ = pick_index
        elif self.connect_joe_to_pybullet:
            self._post_inference_exacutable_trajectories = trajectories

    def _on_handle_post_inference_execute_target(self, msg: PostInferenceExecuteTrajectoryMessage):
        robot_name = msg.robot_name
        goal_name = msg.inference_goal_name
        target_name = msg.target_name

        self._LOGGER.log_message(message=msg)

        handler = self.handler
        if self.connect_raj_to_pybullet:
            handler._execute_inference_pick_and_place_raj(self._post_inference_exacutable_trajectories, self._POST_INFERENCE_PICK_INDEX_RAJ)
        elif self.connect_joe_to_pybullet:
            handler._execute_inference_pick_and_place(self._post_inference_exacutable_trajectories)
        print(f"CommunicationManager : [CommunicationManager] Received Post Inference Execute Target Message: {msg}")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_CONFIG_FP = os.path.join(SCRIPT_DIR, "project_config.json")
PROJECT_CONFIG_DICT = json_load(PROJECT_CONFIG_FP)

ROBOT_NAME = "UR20"

# MQTT_CONFIG = PROJECT_CONFIG_DICT["mqtt_config"]
# MQTT_CONFIG = PROJECT_CONFIG_DICT["mqtt_config_ruxin"]
# MQTT_CONFIG = PROJECT_CONFIG_DICT["mqtt_config_eduroam"]
# MQTT_CONFIG = PROJECT_CONFIG_DICT["mqtt_config_chaosnet"]
# MQTT_CONFIG = PROJECT_CONFIG_DICT["mqtt_config_local"]
MQTT_CONFIG = PROJECT_CONFIG_DICT["mqtt_config_macmini"]
BROKER = MQTT_CONFIG["broker"]
MQTT_PORT = MQTT_CONFIG["port"]

PROJECT_NAME = PROJECT_CONFIG_DICT["project_name"]

# BACKEND_TYPE = "PyBullet"  # or "ROS", depending on the backend you want to use
BACKEND_TYPE = "COMBINED"
# BACKEND_TYPE = "ROS"

#TODO: PICK & PLACE FRAMES TOLERANCE VALUES
PICK_AND_PLACE_XAXIS_TOLERANCE = 0.04  # Meters
# PICK_AND_PLACE_ZAXIS_TOLERANCE = None  # Meters
PICK_AND_PLACE_ZAXIS_TOLERANCE = 0.012  # Meters #TODO: JOSEPH IF TOLERANCE NEEDED UPDATE HERE... (INFERENCE ONLY)


#TODO: This is the quickest fix to avoid the dual pybullet issues....
# WHOSE_PYBULLET = "RAJ"
WHOSE_PYBULLET = "JOSEPH"


#TODO: Add Participant specific information
# PARTICIPANT_NAME = "test_user_2"
# PARTICIPANT_NUMBER = "00"
# PARTICIPANT_ID = "4970"

# PARTICIPANT_NUMBER = "01"
# PARTICIPANT_ID = "850029"
# PARTICIPANT_NAME = f"P{PARTICIPANT_NUMBER}_{PARTICIPANT_ID}"

# PARTICIPANT_NUMBER = "02"
# PARTICIPANT_ID = "911324"
# PARTICIPANT_NAME = f"P{PARTICIPANT_NUMBER}_{PARTICIPANT_ID}"

# PARTICIPANT_NUMBER = "03"
# PARTICIPANT_ID = "0874"
# PARTICIPANT_NAME = f"P{PARTICIPANT_NUMBER}_{PARTICIPANT_ID}"

# PARTICIPANT_NUMBER = "04"
# PARTICIPANT_ID = "2022653"
# PARTICIPANT_NAME = f"P{PARTICIPANT_NUMBER}_{PARTICIPANT_ID}"

# PARTICIPANT_NUMBER = "05"
# PARTICIPANT_ID = "123087101"
# PARTICIPANT_NAME = f"P{PARTICIPANT_NUMBER}_{PARTICIPANT_ID}"

# PARTICIPANT_NUMBER = "06"
# PARTICIPANT_ID = "332021"
# PARTICIPANT_NAME = f"P{PARTICIPANT_NUMBER}_{PARTICIPANT_ID}"

# PARTICIPANT_NUMBER = "07"
# PARTICIPANT_ID = "22745417"
# PARTICIPANT_NAME = f"P{PARTICIPANT_NUMBER}_{PARTICIPANT_ID}"

PARTICIPANT_NUMBER = "08"
PARTICIPANT_ID = "123456"
PARTICIPANT_NAME = f"P{PARTICIPANT_NUMBER}_{PARTICIPANT_ID}"

#TODO: While testing comment this in.
# PARTICIPANT_NAME = "researcher_tests_12"

requested_frames = []

if __name__ == "__main__":
    manager = CommunicationManager(
        project_name=PROJECT_NAME,
        robot_name=ROBOT_NAME,
        project_config_dict=PROJECT_CONFIG_DICT,
        pybullet_raj_or_joseph=WHOSE_PYBULLET,
        broker=BROKER,
        mqtt_port=MQTT_PORT,
        backend_type=BACKEND_TYPE,
        pick_and_place_xaxis_tolerance=PICK_AND_PLACE_XAXIS_TOLERANCE,
        pick_and_place_zaxis_tolerance=PICK_AND_PLACE_ZAXIS_TOLERANCE,
        participant_name=PARTICIPANT_NAME
    )
    print("[CommunicationManager] Listening for mimic requests... (Press Ctrl+C to exit)")
    try:
        while True:
            pass  # Keep the process alive
    except KeyboardInterrupt:
        manager._LOGGER.save_log()
        if len(manager.handler.realtime_mimic_data_storage) > 0:
            fp = os.path.join(manager.handler.logging_dir, f"{int(time.time())}_{manager.handler.participant_name}_realtime_mimic_data_storage_final_log.json")
            json_dump(manager.handler.realtime_mimic_data_storage, fp, pretty=True)
        if len(manager.handler.realtime_mimic_ik_solutions) > 0:
            fp_data = os.path.join(manager.handler.logging_dir, f"{int(time.time())}_{manager.handler.participant_name}_realtime_mimic_ik_solutions_final_log.json")
            json_dump(manager.handler.realtime_mimic_ik_solutions, fp_data, pretty=True)
        
        print("[CommunicationManager] Shutdown requested.")
