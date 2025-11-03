#TODO: This is a temporary file to test inference functionality. Needs to be updated by Camillas final implementation
from compas.data import json_load
from compas.geometry import Transformation, Frame
from compas.data import json_dump
import random
import os
import time
import json
import numpy as np

from .simple_inference.inference import SimpleInference
from .camilla.camilla_inference import CamillaInference

class InferenceManager:

    def __init__(self, goals_folder_path, participant_name, state_file_path=None):

        #TODO: Tune these thresholds
        self.INFERENCE_POSITIONAL_THRESHOLD = 0.05  # Meters
        self.INFERENCE_ROTATIONAL_THRESHOLD = 5.0  # Degrees

        #TODO: Load State: (make an input that is name.json file or simmilar...)

            #TODO: If the file exists, load from the previous state.

        self.goals_dict = self._load_goals(goals_folder_path)
        self.record_file_path = self._set_record_file_path(goals_folder_path, participant_name)
        self.inference_session_start = None
        self._LOADED_PREVIOUS_STATE = False
        self._state_file_path = state_file_path

        if state_file_path and os.path.exists(state_file_path):
            state_data = json_load(state_file_path)
            self.incorrect_goals = state_data.get("incorrect_goals", [])
            self.incorrect_targets = state_data.get("incorrect_targets", [])
            self.correct_targets = state_data.get("correct_targets", [])
            self.target_log = state_data.get("target_log", [])
            self.INFERED_GOAL = state_data.get("INFERED_GOAL", None)
            self.inference_session_start = state_data.get("inference_session_start", None)
            self.inference_routines_log = state_data.get("inference_routines_log", {})
            self._LOADED_PREVIOUS_STATE = True
            print(f"InferenceManager : Loaded previous state from {state_file_path} : Incorrect Goals {self.incorrect_goals}, goal_inferred : .")
        else:
            self.incorrect_goals = []
            self.incorrect_targets = []
            self.correct_targets = []
            self.target_log = []
            self.INFERED_GOAL = None
            self.inference_routines_log = {}
            print("InferenceManager : No previous state file found. Starting fresh inference session.")

        self.simple_inference = SimpleInference()
        self.camilla_inference = CamillaInference()

        self.TEMPORARY_SUB_GOALS_LIST = ['G0', 'G1', 'G2', 'G3', 'G4', 'G5', 'G6', 'G7', 'G8']

    def _load_goals(self, goals_folder_path):
        goals_dict = {}
        for filename in os.listdir(goals_folder_path):
            if filename.endswith(".json"):
                filepath = os.path.join(goals_folder_path, filename)
                try:
                    goal_data = json_load(filepath)
                    print (f"GoalManager : Loaded goal from file: {filename} with {len(goal_data['cube_locations'])} target locations.")
                    goal_name = goal_data.get("name")
                    if goal_name:
                        goals_dict[goal_name] = goal_data
                    else:
                        print(f"GoalManager : Warning: 'goal_name' missing in {filename}. Skipping this file.")
                except json.JSONDecodeError as e:
                    print(f"GoalManager : Error decoding JSON from file {filename}: {e}. Skipping this file.")
        print(f"Loaded {len(goals_dict)} goals from {goals_folder_path}.")
        return goals_dict
    
    def _set_record_file_path(self, goals_folder_path, participant_name):
        # go up one directory, then into "records"
        record_folder_path = os.path.normpath(
            os.path.join(goals_folder_path, "..", "records")
        )
        os.makedirs(record_folder_path, exist_ok=True)
        filename = f"{int(time.time())}_{participant_name}_inference_record.json"
        record_file_path = os.path.join(record_folder_path, filename)
        print(f"GoalManager : Inference record file will be saved to: {record_file_path}")
        return record_file_path

    def _record_inference_data(self, data: dict):
        """
        Append or initialize inference record file with new data.
        """
        # Ensure parent directory exists
        os.makedirs(os.path.dirname(self.record_file_path), exist_ok=True)

        # Load existing log if the file exists, else start fresh
        if os.path.exists(self.record_file_path):
            try:
                log = json_load(self.record_file_path)
            except Exception:
                print("Warning: Could not load existing record file. Starting new log.")
                log = {}
        else:
            log = {}

        # Use inference_session_start as unique key
        session_key = str(self.inference_session_start)

        # Update or create the entry for this session
        log[session_key] = data

        # Save back to file
        json_dump(fp=self.record_file_path, data=log, pretty=True)
        print(f"InferenceManager: Record updated at {self.record_file_path}")

    def _write_state_file_data(self):
        if not self._state_file_path:
            raise ValueError("InferenceManager : State file path not provided.")

        state_data = {
            "incorrect_goals": self.incorrect_goals,
            "incorrect_targets": self.incorrect_targets,
            "correct_targets": self.correct_targets,
            "target_log": self.target_log,
            "INFERED_GOAL": self.INFERED_GOAL,
            "inference_session_start": self.inference_session_start
        }

        # Save state data to file
        json_dump(fp=self._state_file_path, data=state_data, pretty=True)
        print(f"InferenceManager : State saved to {self._state_file_path}")

    def _process_user_reply(self, goal_name, suggested_target_name, goal_status_reply, timestamp):
        # Rejecting goal and target
        if goal_status_reply == 0:
            self.incorrect_goals.append(goal_name)
            data = {}
            data["timestamp"] = timestamp
            data["suggested_target_name"] = suggested_target_name
            data["goal_name"] = goal_name
            self.incorrect_targets.append(data)
            data["target_status"] = "incorrect_target"
            self.target_log.append(data)
            #TODO: WRITE THE STATE FILE
            self._write_state_file_data()

        # Rejecting goal and Accepting target
        elif goal_status_reply == 1:
            self.incorrect_goals.append(goal_name)
            data = {}
            data["timestamp"] = timestamp
            data["suggested_target_name"] = suggested_target_name
            data["goal_name"] = goal_name
            self.correct_targets.append(data)
            data["target_status"] = "correct_target"
            self.target_log.append(data)
            #TODO: WRITE THE STATE FILE
            self._write_state_file_data()

        # Accepting Goal and target
        elif goal_status_reply == 2:
            # self.correct_targets.append(suggested_target_name)
            self.INFERED_GOAL = goal_name
            self.incorrect_goals.append(goal_name)
            data = {}
            data["timestamp"] = timestamp
            data["suggested_target_name"] = suggested_target_name
            data["goal_name"] = goal_name
            self.correct_targets.append(data)
            data["target_status"] = "correct_target"
            data["goal_inferred"] = True
            data["inference_timestamp"] = time.time()
            self.target_log.append(data)
            self.inference_routines_log[str(self.inference_session_start)] = {
                "incorrect_goals": self.incorrect_goals,
                "incorrect_targets": self.incorrect_targets,
                "correct_targets": self.correct_targets,
                "target_log": self.target_log,
                "INFERED_GOAL": self.INFERED_GOAL,
                "GOAL_INFERRED_AT": time.time(),
                "inference_session_start": self.inference_session_start,
                "inference_request_time": time.time(),
                "inference_session_duration": time.time() - self.inference_session_start
            }
            self._record_inference_data(self.inference_routines_log)
            #TODO: WRITE THE STATE FILE
            self._write_state_file_data()

        # Unknown reply
        else:
            print(f"GoalManager : Warning: Unknown goal_status_reply '{goal_status_reply}' received.")

    def _construct_transformation_matrices(self, anchor_cube_frame):
        # Placeholder for constructing transformation matrices
        # In a real implementation, this would involve more complex logic
        T_anchor_to_world = Transformation.from_frame_to_frame(anchor_cube_frame, Frame.worldXY())
        inverse_T = T_anchor_to_world.inverse()
        return T_anchor_to_world, inverse_T

    def _transform_geometry_dict_for_inference(self, geometry_frames, transformation):
        anchor_cube_frame = geometry_frames.get("AnchorCube")
        if not anchor_cube_frame:
            raise ValueError("GoalManager : AnchorCube frame is missing from geometry_frames.")
        
        #Construct transformation from AnchorCube to world
        transformed_frames_dict = {}
        for name, frame in geometry_frames.items():
            transformed_frame = frame.transformed(transformation)
            transformed_frames_dict[name] = transformed_frame
        return transformed_frames_dict

    def handle_inference_request(self, geometry_frames_dict, initial_request):

        #If it is the first request, reset the lists
        if initial_request and not self._LOADED_PREVIOUS_STATE:
            self.inference_session_start = time.time()
            self.incorrect_goals = []
            self.incorrect_targets = []
            self.correct_targets = []
            self.INFERED_GOAL = None
            print("InferenceManager: Starting a new inference routine.")
        else:
            self.inference_routines_log[str(self.inference_session_start)] = {
                "incorrect_goals": self.incorrect_goals,
                "incorrect_targets": self.incorrect_targets,
                "correct_targets": self.correct_targets,
                "target_log": self.target_log,
                "INFERED_GOAL": self.INFERED_GOAL,
                "inference_session_start": self.inference_session_start,
                "inference_request_time": time.time(),
                "inference_session_duration": time.time() - self.inference_session_start
            }
            self._record_inference_data(self.inference_routines_log)
            print(f"Previous inference results so far: {len(self.incorrect_goals)} incorrect goals, {len(self.incorrect_targets)} incorrect targets, {len(self.correct_targets)} correct targets.")

        #Get the anchor cube frame
        anchor_cube_frame = geometry_frames_dict.get("AnchorCube")
        if not anchor_cube_frame:
            raise ValueError("AnchorCube frame is missing from geometry_frames.")

        data = {}
        data["geometry_frames_input_before_tx"] = geometry_frames_dict

        # #TODO: TESTINGGGGGG Camilla inference###########################################################################################
        # input_dat_array = self.create_input_dat_file(geometry_frames_dict)
        # inf_result = self.camilla_inference.perform_inference(input_dat_array, initial_request)
        # print(f"Camilla inference result: {inf_result}")
        # print(f"Input .dat array for inference:\n{input_dat_array}")
        # #TODO: TESTINGGGGGG Camilla inference###########################################################################################

        #Transform geometry frames to world frame for comparision wiht goals
        T_anchor_to_world, inverse_T = self._construct_transformation_matrices(anchor_cube_frame)
        transformed_current_geometry_frames = self._transform_geometry_dict_for_inference(geometry_frames_dict, T_anchor_to_world)

        # #Perform inference on transformed frames
        # suggested_goal, suggested_target_name, completed_target_names, suggested_target_frame, completed_items_names, incompleted_items_names = self.perform_inference(transformed_current_geometry_frames)
        # fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\inference\inference_debug.json"
        # data["suggested_goal"] = suggested_goal
        # data["suggested_target_name"] = suggested_target_name
        # data["completed_goal_indexes"] = completed_target_names
        # data["geometry_frames_input_after_tx"] = transformed_current_geometry_frames
        # data["target_frame"] = suggested_target_frame
        # data["suggested_target_frame_transformed"] = suggested_target_frame.transformed(inverse_T) if suggested_target_frame else None
        # data["transformed_geometry_frames"] = transformed_current_geometry_frames

        #TODO: TESTINGGGGGG Simple inference###########################################################################################
        inference_results_dict = self.simple_inference.perform_inference(self.goals_dict, transformed_current_geometry_frames, self.incorrect_goals, self.INFERENCE_POSITIONAL_THRESHOLD, self.INFERENCE_ROTATIONAL_THRESHOLD)
        print (f"Simple inference results: {inference_results_dict}")
        suggested_goal = inference_results_dict.get("suggested_goal")
        suggested_target_name = inference_results_dict.get("suggested_target_name")
        completed_target_names = inference_results_dict.get("completed_target_names", [])
        suggested_target_frame = inference_results_dict.get("suggested_target_frame")
        completed_items_names = inference_results_dict.get("completed_items_names", [])
        incompleted_items_names = inference_results_dict.get("incompleted_items_names", [])

        # json_dump(fp=fp, data=data, pretty=True)
        print(f"INFERENCE MANAGER: Suggested goal: {suggested_goal}, completed_goal_indexes: {completed_target_names}, target_frame: {suggested_target_frame}, completed_items_names: {completed_items_names}, incompleted_items_names: {incompleted_items_names}")

        if suggested_goal == None:
            return None
        if suggested_target_frame == None or len(completed_target_names) <= 0 or len(completed_items_names) <= 0 or len(incompleted_items_names) <= 0:
            raise ValueError("InferenceManager : Incomplete inference result. One of the required fields is None or empty.")


        #Package in a dictionary to return
        suggested_target_transformed = suggested_target_frame.transformed(inverse_T)
        inference_result = {
            "suggested_goal": suggested_goal,
            "completed_goals": completed_target_names,
            "suggested_target": suggested_target_transformed,
            "suggested_target_name": suggested_target_name,
            "completed_items": completed_items_names,
            "incompleted_items": incompleted_items_names
        }
        return inference_result

    #TODO CHECK THIS VALIDATION PLEASE.... SOMETHING IS NOT WORKING VERY WELL...

    def _validate_and_compute_target_location(self, goal_name, geometry_frames_dict, target_frame):
        if goal_name != self.INFERED_GOAL:
            goal_is_correct = False
        else:
            goal_is_correct = True

        #Get the anchor cube frame
        anchor_cube_frame = geometry_frames_dict.get("AnchorCube")
        if not anchor_cube_frame:
            raise ValueError("AnchorCube frame is missing from geometry_frames.")

        #Get the target_frame from the goal data
        goal_data = self.goals_dict.get(goal_name)
        if not goal_data:
            raise ValueError(f"GoalManager : Goal '{goal_name}' not found in goals_dict.")
        target_goal_entry = goal_data["cube_locations"].get(target_frame)
        if not target_goal_entry:
            raise ValueError(f"GoalManager : Target '{target_frame}' not found in goal '{goal_name}' cube_locations.")
        else:
            print(f"GoalManager : Found target '{target_frame}' in goal '{goal_name}' cube_locations.")
        #Transform geometry frames to world frame for comparision wiht goals
        T_anchor_to_world, inverse_T = self._construct_transformation_matrices(anchor_cube_frame)
        transformed_target_frame = target_goal_entry.frame.transformed(inverse_T)

        return goal_is_correct, transformed_target_frame
    
    def create_input_dat_file(self, current_geometry_frames_dict):
        expected_cubes = ["AnchorCube", "Cube01", "Cube02", "Cube03", "Cube04", "Cube05", "Cube06", "Cube07", "Cube08"]
        cube_data_list = []

        for cube_name in expected_cubes:
            if cube_name not in current_geometry_frames_dict:
                raise ValueError(f"create_input_dat_file: Missing expected cube '{cube_name}' in geometry frames.")
            
            current_frame = current_geometry_frames_dict[cube_name]
            if not isinstance(current_frame, Frame):
                raise ValueError(f"create_input_dat_file: Frame for '{cube_name}' is not a valid Frame object.")

            x = current_frame.point.x
            y = current_frame.point.y
            eualer_angles = current_frame.euler_angles()
            x_rot = eualer_angles[0]
            y_rot = eualer_angles[1]
            z_rot = eualer_angles[2]
            data_dict = {"x": x, "y": y, "theta": x_rot}

            array = np.array([x, y, x_rot])
            cube_data_list.append(array)
        
        return np.array(cube_data_list)