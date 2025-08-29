#TODO: This is a temporary file to test inference functionality. Needs to be updated by Camillas final implementation
from compas.data import json_load
from compas.geometry import Transformation, Frame
import random
import os

class InferenceManager:

    def __init__(self, goals_folder_path):

        self.goals_dict = self._load_goals(goals_folder_path)
        self.incorrect_goals = []
        self.incorrect_targets = []
        self.correct_targets = []
        
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
        target_name = random.choice(incompleted_goal_names) if incompleted_goal_names else None
        target_frame = None
        if target_name:
            # Your schema: goal["cube_locations"][Gx]["data"]["frame"] (often a Box with .frame via compas json_load)
            try:
                entry = goal_data["cube_locations"][target_name]
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
        print(f"Suggested target: {target_name} -> {target_frame}")

        return suggested_goal, completed_goal_names, target_frame, completed_items_names, incompleted_items_names

    def handle_inference_request(self, geometry_frames_dict, initial_request):

        #If it is the first request, reset the lists
        if initial_request:
            self.incorrect_goals = []
            self.incorrect_targets = []
            self.correct_targets = []
        else:
            print(f"Previous inference results so far: {len(self.incorrect_goals)} incorrect goals, {len(self.incorrect_targets)} incorrect targets, {len(self.correct_targets)} correct targets.")

        #Get the anchor cube frame
        anchor_cube_frame = geometry_frames_dict.get("AnchorCube")
        if not anchor_cube_frame:
            raise ValueError("AnchorCube frame is missing from geometry_frames.")

        #Transform geometry frames to world frame for comparision wiht goals
        T_anchor_to_world, inverse_T = self._construct_transformation_matrices(anchor_cube_frame)
        transformed_frames = self._transform_geometry_dict_for_inference(geometry_frames_dict, T_anchor_to_world)

        #Perform inference on transformed frames
        suggested_goal, completed_goal_indexes, target_frame, completed_items_names, incompleted_items_names = self.perform_inference(transformed_frames)

        print(f"INFERENCE MANAGER: Suggested goal: {suggested_goal}, completed_goal_indexes: {completed_goal_indexes}, target_frame: {target_frame}, completed_items_names: {completed_items_names}, incompleted_items_names: {incompleted_items_names}")

        if suggested_goal == None:
            return None
        if target_frame == None or len(completed_goal_indexes) <= 0 or len(completed_items_names) <= 0 or len(incompleted_items_names) <= 0:
            raise ValueError("InferenceManager : Incomplete inference result. One of the required fields is None or empty.")


        #Package in a dictionary to return
        suggested_target_transformed = target_frame.transformed(inverse_T)
        inference_result = {
            "suggested_goal": suggested_goal,
            "completed_goals": completed_goal_indexes,
            "suggested_target": suggested_target_transformed,
            "completed_items": completed_items_names,
            "incompleted_items": incompleted_items_names
        }

        return inference_result
    
