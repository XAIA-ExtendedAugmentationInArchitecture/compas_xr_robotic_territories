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

    def __init__(self, goals_folder_path):

        self.goals_dict = self._load_goals(goals_folder_path)
        self.record_file_path = self._set_record_file_path(goals_folder_path)
        self.runntime_start = time.time()
        self.inference_session_start = None
        self.inference_routines_log = {}

        self.incorrect_goals = []
        self.incorrect_targets = []
        self.correct_targets = []
        self.target_log = []
        self.INFERED_GOAL = None

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
                    goal_name = goal_data.get("name")
                    if goal_name:
                        goals_dict[goal_name] = goal_data
                    else:
                        print(f"GoalManager : Warning: 'goal_name' missing in {filename}. Skipping this file.")
                except json.JSONDecodeError as e:
                    print(f"GoalManager : Error decoding JSON from file {filename}: {e}. Skipping this file.")
        print(f"Loaded {len(goals_dict)} goals from {goals_folder_path}.")
        return goals_dict
    
    def _set_record_file_path(self, goals_folder_path):
        # go up one directory, then into "records"
        record_folder_path = os.path.normpath(
            os.path.join(goals_folder_path, "..", "records")
        )
        os.makedirs(record_folder_path, exist_ok=True)
        filename = f"{int(time.time())}_inference_record.json"
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

    def perform_inference(self, transformed_frames):
        """
        Returns:
        suggested_goal: str
        completed_goal_indexes: List[int]          # 0..k, k <= 5
        target_frame: Optional[Frame]              # frame of a randomly chosen incompleted goal
        completed_items_names: List[str]           # |completed_items| == |completed_goal_indexes|
        incompleted_items_names: List[str]         # |incompleted_items| == |incompleted_goal_indexes|
        """
        # ---- Safety
        if not isinstance(self.goals_dict, dict) or not self.goals_dict:
            raise ValueError("GoalManager : perform_inference: no goals available in self.goals_dict.")
        if not isinstance(self.TEMPORARY_SUB_GOALS_LIST, list) or not self.TEMPORARY_SUB_GOALS_LIST:
            raise ValueError("GoalManager : perform_inference: TEMPORARY_SUB_GOALS_LIST is empty.")

        # ---- 1) Random goal key (e.g., "Goal09")
        suggested_goal = random.choice(list(self.goals_dict.keys()))
        goal_data = self.goals_dict[suggested_goal]

        # ---- 2) Candidate goal names and helper
        candidate_names = list(self.TEMPORARY_SUB_GOALS_LIST)  # ['G0'..'G8']

        def _as_frame(v):
            if isinstance(v, Frame):
                return v
            if isinstance(v, dict):
                try:
                    return Frame.from_data(v)
                except Exception:
                    return None
            return None

        # ---- 3) Completed goal range "0..k" with k ≤ 5
        max_idx = len(candidate_names) - 1
        if max_idx < 1:
            completed_goal_indexes = []
        else:
            k_cap = min(5, max_idx)     # enforce cap at 5
            k = random.randint(1, k_cap)
            completed_goal_indexes = list(range(0, k + 1))

        incompleted_goal_indexes = [i for i in range(0, max_idx + 1) if i not in completed_goal_indexes]
        completed_goal_names = [candidate_names[i] for i in completed_goal_indexes]
        incompleted_goal_names = [candidate_names[i] for i in incompleted_goal_indexes]

        # ---- 4) Pick target frame from an incompleted goal
        suggested_target_name = random.choice(incompleted_goal_names) if incompleted_goal_names else None
        target_frame = None
        if suggested_target_name:
            # Your schema: goal["cube_locations"][Gx]["data"]["frame"] (often a Box with .frame via compas json_load)
            try:
                entry = goal_data["cube_locations"][suggested_target_name]
                # If json_load reconstructs a Box, `.frame` works; else try dict->Frame
                if hasattr(entry, "frame"):
                    target_frame = entry.frame
                else:
                    frame_dict = None
                    if isinstance(entry, dict):
                        if isinstance(entry.get("data"), dict) and isinstance(entry["data"].get("frame"), dict):
                            frame_dict = entry["data"]["frame"]
                        elif isinstance(entry.get("frame"), dict):
                            frame_dict = entry["frame"]
                    target_frame = _as_frame(frame_dict)
            except Exception:
                target_frame = None  # stay robust

        # ---- 5) Build items lists with counts matching index counts
        item_names = list(transformed_frames.keys()) if isinstance(transformed_frames, dict) else []
        anchor_present = "AnchorCube" in item_names

        # Prefer goal-like names in the scene to satisfy counts
        goal_like_in_scene = [n for n in candidate_names if n in item_names]
        # Non-goal names as fallback pool
        non_goal_pool = [n for n in item_names if n not in set(goal_like_in_scene + (["AnchorCube"] if anchor_present else []))]

        C = len(completed_goal_indexes)
        U = len(incompleted_goal_indexes)

        # Completed items
        completed_items_names = []
        if anchor_present and C > 0:
            completed_items_names.append("AnchorCube")
            need_completed_from_goals = C - 1
        else:
            need_completed_from_goals = C

        # sample completed from goal-like pool
        if need_completed_from_goals > 0:
            take = min(need_completed_from_goals, len(goal_like_in_scene))
            chosen = random.sample(goal_like_in_scene, k=take)
            completed_items_names.extend(chosen)
            # remove chosen from the goal pool
            goal_like_in_scene = [n for n in goal_like_in_scene if n not in chosen]

        # if still short, use non-goal pool
        short = C - len(completed_items_names)
        if short > 0 and non_goal_pool:
            take = min(short, len(non_goal_pool))
            chosen = random.sample(non_goal_pool, k=take)
            completed_items_names.extend(chosen)
            non_goal_pool = [n for n in non_goal_pool if n not in chosen]

        # Incompleted items: draw from remaining goal-like first
        incompleted_items_names = []
        if U > 0:
            take = min(U, len(goal_like_in_scene))
            chosen = random.sample(goal_like_in_scene, k=take)
            incompleted_items_names.extend(chosen)
            goal_like_in_scene = [n for n in goal_like_in_scene if n not in chosen]

        # if still short, fill from remaining non-goal pool
        short = U - len(incompleted_items_names)
        if short > 0 and non_goal_pool:
            take = min(short, len(non_goal_pool))
            chosen = random.sample(non_goal_pool, k=take)
            incompleted_items_names.extend(chosen)
            non_goal_pool = [n for n in non_goal_pool if n not in chosen]

        # Final guard: if we still couldn't reach exact counts (not enough scene items), clip and warn
        if len(completed_items_names) != C or len(incompleted_items_names) != U:
            print(f"[perform_inference] Warning: scene items ({len(item_names)}) insufficient to match desired counts "
                f"(completed={C}, incompleted={U}). "
                f"Got completed={len(completed_items_names)}, incompleted={len(incompleted_items_names)}.")

        # ---- Debug (optional)
        print(f"Performing inference : PLACE HOLDER : on {len(transformed_frames)} frames.")
        print(f"Selected goal: {suggested_goal}")
        print(f"Completed goal idx: {completed_goal_indexes} | names: {completed_goal_names} (len={len(completed_goal_indexes)})")
        print(f"Incompleted goal idx: {incompleted_goal_indexes} | names: {incompleted_goal_names} (len={len(incompleted_goal_indexes)})")
        print(f"Completed items (len={len(completed_items_names)}): {completed_items_names}")
        print(f"Incompleted items (len={len(incompleted_items_names)}): {incompleted_items_names}")
        print(f"Suggested target: {suggested_target_name} -> {target_frame}")

        return suggested_goal, suggested_target_name, completed_goal_names, target_frame, completed_items_names, incompleted_items_names

    def handle_inference_request(self, geometry_frames_dict, initial_request):

        #If it is the first request, reset the lists
        if initial_request:
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
        inference_results_dict = self.simple_inference.perform_inference(self.goals_dict, transformed_current_geometry_frames, self.incorrect_goals, 0.03, 3.0)
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