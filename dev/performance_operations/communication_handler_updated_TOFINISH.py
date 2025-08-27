from compas_eve import Subscriber, Publisher, Topic
from compas_eve.mqtt import MqttTransport
from compas_xr.mqtt import RealtimeMimicRequestMessage, RealtimeMimicResultMessage, MimicTrajectoryRequestMessage, MimicTrajectoryResultMessage, ExecuteMimicTrajectoryRequestMessage, RealtimeMimicIOToggleRequestMessage

# from robots.com_handlers.realtime_mimic_roshandler import URRealtimeMimicHandler
from robots.com_handlers.realtime_mimic_pbhandler import URMimicHandlerPyB, ABBMimicHandlerPyB #TODO: This needs to be wrapped into one handler for both Mimics
from robots.com_handlers.realtime_mimic_roshandler import URMimicHandlerROS, ABBMimicHandlerROS #TODO: This needs to be wrapped into one handler for both Mimics
from robots.com_handlers.robot_handler_combined_rospy import URMimicHandlerCombined, ABBMimicHandlerCombined
from compas.geometry import Rotation

from compas.data import json_load, json_dump
import os
import math

#TODO: FIX ME JOSEPH. START PLANNING....

class CommunicationManager:

    def __init__(self, project_name, robot_name, project_config_dict, broker='localhost', mqtt_port=1883, backend_type='PyBullet'):
        self.mqtt = MqttTransport(broker, mqtt_port)
        self.project_name = project_name
        self.robot_name = robot_name

        #Robot Loading & Handleing
        _urdf_filepath = project_config_dict["urdf_fps"][robot_name]["urdf"]
        _srdf_filepath = project_config_dict["urdf_fps"][robot_name]["srdf"]
        _robot_hardware_info = project_config_dict["robot_hardware_info"][robot_name]
        self.handler = self._load_handler(robot_name, _urdf_filepath, _srdf_filepath, _robot_hardware_info, backend_type=backend_type)

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
        self._user_initiated_mimic_trajectories_to_execute = []

        _transformations_file_path = project_config_dict["robot_transformations_fp"]
        if not _transformations_file_path:
            raise ValueError("Transformations file path is required in the project configuration.")
        (
            self.transformation_ar_space_to_robot_space,
            self.transformations_robot_space_to_ar_space,
            self._urdf_baseframe,
            self._observed_urdf_baseframe
        ) = self._load_transformations_from_file_and_baseframes_from_file(_transformations_file_path, robot_name)
        print(f"CommunicationManager : [CommunicationManager] Subscribed to: robotic_territories mimic topics for project '{project_name}' and robot '{robot_name}'")

    def _load_handler(self, robot_name, urdf_filepath, srdf_filepath, robot_hardware_info_dict, backend_type='PyBullet'):
        if robot_name == "UR20" or robot_name == "UR31" or robot_name == "UR32":
            if backend_type == 'PyBullet':
                return URMimicHandlerPyB(robot_name, 
                                                 robot_ip=robot_hardware_info_dict["robot_ip"], 
                                                 urdf_path=urdf_filepath, 
                                                 srdf_path=srdf_filepath,
                                                 group=robot_hardware_info_dict["group"],
                                                 speed=robot_hardware_info_dict["speed"],
                                                 acceleration=robot_hardware_info_dict["acceleration"],
                                                 radius=robot_hardware_info_dict["radius"],
                                                 nowait=robot_hardware_info_dict["nowait"],
                                                 io=robot_hardware_info_dict["vacum_io"],
                                                 tool_info_fp=robot_hardware_info_dict.get("tool_info_fp"),
                                                 additional_static_collision_meshes_fp=robot_hardware_info_dict.get("additional_collison_meshes_fp"))
            elif backend_type == 'ROS':
                return URMimicHandlerROS(robot_name, 
                                                 robot_ip=robot_hardware_info_dict["robot_ip"], 
                                                 ros_ip=robot_hardware_info_dict["ros_ip"],
                                                 ros_port=robot_hardware_info_dict["ros_port"],
                                                 speed=robot_hardware_info_dict["speed"],
                                                 acceleration=robot_hardware_info_dict["acceleration"],
                                                 radius=robot_hardware_info_dict["radius"],
                                                 nowait=robot_hardware_info_dict["nowait"],
                                                 tool_info_fp=robot_hardware_info_dict.get("tool_info_fp"),
                                                 additional_static_collision_meshes_fp=robot_hardware_info_dict.get("additional_collison_meshes_fp"))
            elif backend_type == 'COMBINED':
                return URMimicHandlerCombined(robot_name, 
                                                 robot_ip=robot_hardware_info_dict["robot_ip"], 
                                                 urdf_path=urdf_filepath, 
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
                                                 additional_static_collision_meshes_fp=robot_hardware_info_dict.get("additional_collison_meshes_fp"))                
            else:
                raise ValueError(f"Unsupported backend type: {backend_type} for robot {robot_name}")
        elif robot_name == "ABB1" or robot_name == "ABB2" or robot_name == "ABB_IRB4600LL" or robot_name == "ABB_IRB4600LL":
            if backend_type == 'PyBullet':
                return ABBMimicHandlerPyB(robot_name, 
                                                  robot_ip=robot_hardware_info_dict["robot_ip"], 
                                                  urdf_path=urdf_filepath, 
                                                  srdf_path=srdf_filepath,
                                                  speed=robot_hardware_info_dict["speed"],
                                                  nowait=robot_hardware_info_dict["nowait"],
                                                  tool_info_fp=robot_hardware_info_dict.get("tool_info_fp"),
                                                  additional_static_collision_meshes_fp=robot_hardware_info_dict.get("additional_attached_collison_meshes_fp"))
            elif backend_type == 'ROS':
                return ABBMimicHandlerROS(robot_name, 
                                                  robot_ip=robot_hardware_info_dict["robot_ip"], 
                                                  abb_client=robot_hardware_info_dict["robot_ip"],
                                                  ros_ip=robot_hardware_info_dict["ros_ip"],
                                                  ros_port=robot_hardware_info_dict["ros_port"],
                                                  speed=robot_hardware_info_dict["speed"],
                                                  nowait=robot_hardware_info_dict["nowait"],
                                                  tool_info_fp=robot_hardware_info_dict.get("tool_info_fp"),
                                                  additional_static_collision_meshes_fp=robot_hardware_info_dict.get("additional_attached_collison_meshes_fp"))
            elif backend_type == 'COMBINED':
                return ABBMimicHandlerCombined(robot_name, 
                                                  robot_ip=robot_hardware_info_dict["robot_ip"], 
                                                  urdf_path=urdf_filepath, 
                                                  srdf_path=srdf_filepath,
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

    ######################################################################################################
    # TODO: MESSAGE LOGGING....
    ####################################################################################################




    ######################################################################################################
    # Set Publisers and Subscribers for Inference
    ####################################################################################################

    # def _set_inference_publishers_and_subscribers(self, project_name):
    #     #User Initiated Mimic Request and Result Handlers
    #     user_initiated_mimic_result_topic = Topic(f"robotic_territories/mimic_result/{project_name}", MimicTrajectoryResultMessage)
    #     self.user_initiated_publisher = Publisher(user_initiated_mimic_result_topic, transport=self.mqtt)

    #     user_initiated_mimic_request_topic = Topic(f"robotic_territories/mimic_request/{project_name}", MimicTrajectoryRequestMessage)
    #     self.user_initiated_subscriber = Subscriber(user_initiated_mimic_request_topic, callback=self._on_message_user_initiated_mimic, transport=self.mqtt)
    #     self.user_initiated_subscriber.subscribe()


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

    ######################################################################################################
    # Message Handlers for Realtime Mimic and User Initiated Mimic
    ####################################################################################################

    def _on_message_realtime_mimic(self, msg: RealtimeMimicRequestMessage):
        robot_name = msg.robot_name

        print(f"CommunicationManager : [CommunicationManager] Received Relatime Mimic request for robot '{robot_name}': {msg}")

        handler = self.handler
        # self._save_requested_frame(msg)

        msg.requested_robot_frame = self._transform_requested_frame_from_ar_space_to_robot_space(msg.requested_robot_frame)
        # ik_config = self.handler.handle_realtime_msg_request_ik_target(msg)
        # ik_config = self.handler.handle_realtime_msg_request_compas_fab_itter(msg)
        # ik_config = self.handler.handle_realtime_msg_request_recursive_solver(msg)
        # ik_config = self.handler.handle_realtime_msg_request(msg)
        # ik_config = handler.handle_realtime_msg_request_fastest_ik(msg)
        ik_config = handler.handle_realtime_msg_request_servoj_gate(msg)

        #TODO: NEED TO TRANSFORM BACK TO ROBOT BASEFRAME, BUT JUST SEE IF IT PRINTS FIRST....

        if ik_config:
            #TODO: UPDATE THE KEEPING TRACK OF THE MESSAGES
            result = RealtimeMimicResultMessage(
                robot_name=robot_name,
                return_message=f"IK computed for {msg.message} with {robot_name} and an ik solution of {ik_config}",
            )
            self.realtime_publisher.publish(result)
            print(f"CommunicationManager : [CommunicationManager] Published IK result for robot {robot_name}")
        else:
            print(f" CommunicationManager : [CommunicationManager] No result to publish for robot {robot_name}")

    def _on_message_user_initiated_mimic(self, msg: MimicTrajectoryRequestMessage):
        # TODO: Set this to an empty list just to be safe... This will be my attribute for storing the trajectories to execute.
        self._user_initiated_mimic_trajectories_to_execute = []

        robot_name = msg.robot_name
        requested_human_frames = msg.human_frames
        requested_robot_frames = msg.robot_frames

        print(f"CommunicationManager : [CommunicationManager] Received User Controled Mimic request for robot '{robot_name}': Requesting : {len(requested_robot_frames)} frames")
        # print(f"CommunicationManager : [CommunicationManager] Received User Controled Mimic request for robot '{robot_name}': Requesting : {len(requested_robot_frames)} frames : msg : {msg}")

        # TODO: Remove this logging afer running some tests...
        data={}
        data["robot_name"] = robot_name
        data["requested_robot_frames"] = msg.robot_frames
        data["requested_human_frames"] = msg.human_frames
        json_dump(data=data, fp=r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\random_data_saves\test_requested_robot_frames.json", pretty=True)

        #Transform the frames and reassign them to the message.
        transformed_requested_robot_frames = self._transform_requested_frames_list_from_robot_space_to_ar_space(requested_robot_frames)
        msg.robot_frames = transformed_requested_robot_frames

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

        result = MimicTrajectoryResultMessage(
            robot_name=self.robot_name,
            trajectories=trajectories_to_publsih,
            robot_base_frame=robot_base_frame,
        )

        random_fp_save = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\random_trajectory_for_testing.json"
        json_dump(data=result, fp=random_fp_save, pretty=True)
        self.user_initiated_publisher.publish(result)
        print(f"CommunicationManager : [CommunicationManager] Published result with {len(trajectories_to_publsih)} trajectories for robot {robot_name}")

    def _on_message_user_initiated_mimic_execution(self, msg: ExecuteMimicTrajectoryRequestMessage):
        robot_name = msg.robot_name
        print(f"CommunicationManager : [CommunicationManager] Received User Controled Mimic execution request for robot '{robot_name}' with {len(self._user_initiated_mimic_trajectories_to_execute)} trajectories to execute.")

        handler = self.handler
        success = handler.handle_user_initiated_mimic_execution(msg, self._user_initiated_mimic_trajectories_to_execute)

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
        print(f"CommunicationManager : [CommunicationManager] Processed IO toggle for signal '{signal}' with value '{value}'")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_CONFIG_FP = os.path.join(SCRIPT_DIR, "project_config.json")
PROJECT_CONFIG_DICT = json_load(PROJECT_CONFIG_FP)

ROBOT_NAME = "UR20"

# MQTT_CONFIG = PROJECT_CONFIG_DICT["mqtt_config"]
MQTT_CONFIG = PROJECT_CONFIG_DICT["mqtt_config_local"]  # Use local MQTT config for testing
BROKER = MQTT_CONFIG["broker"]
MQTT_PORT = MQTT_CONFIG["port"]

PROJECT_NAME = PROJECT_CONFIG_DICT["project_name"]

# BACKEND_TYPE = "PyBullet"  # or "ROS", depending on the backend you want to use
BACKEND_TYPE = "COMBINED"
# BACKEND_TYPE = "ROS"
requested_frames = []

if __name__ == "__main__":
    manager = CommunicationManager(project_name=PROJECT_NAME, robot_name=ROBOT_NAME, project_config_dict=PROJECT_CONFIG_DICT, broker=BROKER, mqtt_port=MQTT_PORT, backend_type=BACKEND_TYPE)
    print("[CommunicationManager] Listening for mimic requests... (Press Ctrl+C to exit)")
    try:
        while True:
            pass  # Keep the process alive
    except KeyboardInterrupt:
        print("[CommunicationManager] Shutdown requested.")
