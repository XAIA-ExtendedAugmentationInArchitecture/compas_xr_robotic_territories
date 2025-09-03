from compas.geometry import Frame, Quaternion, Vector, Point, distance_point_point, Box
import math
from typing import Dict, Tuple, List, Optional


class SimpleInference:

    def quaternion_angle_difference(self, q1, q2):
        """
        Compute the angular difference between two quaternions in degrees.
        
        Uses the dot product to find the angle between quaternions.
        """
        dot_product = abs(q1.dot(q2))  # Ensure dot product is positive
        dot_product = min(1.0, max(-1.0, dot_product))  # Clamp to avoid domain errors
        angle_rad = 2 * math.acos(dot_product)  # Convert to angle
        return math.degrees(angle_rad)  # Convert to degrees

    def check_position_difference(self, frame1, frame2, position_threshold):
        """
        Checks if the positional difference between two frames exceeds the threshold.

        :param frame1: First frame.
        :param frame2: Second frame.
        :param position_threshold: Threshold for positional difference.
        :return: True if the positional difference exceeds the threshold, otherwise False.
        """
        position_diff = distance_point_point(frame1.point, frame2.point)
        return position_diff > position_threshold

    def check_rotation_difference(self, quat1, quat2, rotation_threshold):
        """
        Checks if the rotational difference between two quaternions exceeds the threshold.

        :param quat1: First quaternion.
        :param quat2: Second quaternion.
        :param rotation_threshold: Threshold for rotational difference (in degrees).
        :return: True if the rotational difference exceeds the threshold, otherwise False.
        """
        rotation_diff = self.quaternion_angle_difference(quat1, quat2)
        return rotation_diff > rotation_threshold

    # ---------------- Internal utilities ----------------

    @staticmethod
    def _gn_index(name: str) -> Optional[int]:
        """Extract numeric suffix from 'Gk' (e.g., 'G12' -> 12)."""
        if not name or name[0].upper() != 'G':
            return None
        try:
            return int(name[1:])
        except Exception:
            return None

    @classmethod
    def _sorted_g_names(cls, g_names: List[str]) -> List[str]:
        """Sort names like ['G0','G2','G10',...] by numeric order."""
        return sorted(
            g_names,
            key=lambda g: (cls._gn_index(g) is None, cls._gn_index(g) if cls._gn_index(g) is not None else 10**9, g),
        )

    # ---------------- Main inference method ----------------

    def _match_geometry_to_targets(
        self,
        goal_cube_locations: Dict[str, Box],
        current_geometry_positions_dict: Dict[str, Frame],
        position_threshold: float,
        rotation_threshold: float,
    ) -> Tuple[List[str], List[str], List[Tuple[str, str]], float, float]:
        """
        Greedy one-to-one assignment of geometry items to Gk targets within thresholds.

        Returns:
            completed_target_names: List[str]            # ['G0','G1',...]
            completed_items_names:  List[str]            # ['AnchorCube','Cube01',...]
            assignments:            List[(item_name,g)]  # item->Gk matches
            total_pos_err:          float                # sum of positional errors for matches
            total_ang_err:          float                # sum of angular errors (deg) for matches
        """
        candidates = []
        for item_name, item_frame in current_geometry_positions_dict.items():
            for g_name, box in goal_cube_locations.items():
                target_frame: Frame = box.frame

                # Position test (your helper returns True if > threshold, so negate for "within")
                if self.check_position_difference(item_frame, target_frame, position_threshold):
                    continue  # too far

                # Rotation test via quaternions
                q_item = Quaternion.from_frame(item_frame)
                q_target = Quaternion.from_frame(target_frame)
                if self.check_rotation_difference(q_item, q_target, rotation_threshold):
                    continue  # too rotated

                pos_err = distance_point_point(item_frame.point, target_frame.point)
                ang_err = self.quaternion_angle_difference(q_item, q_target)
                candidates.append((pos_err, ang_err, item_name, g_name))

        # Greedy: sort by (pos_err, ang_err), then assign unique item/target
        candidates.sort(key=lambda t: (t[0], t[1]))
        used_items = set()
        used_targets = set()
        assignments: List[Tuple[str, str]] = []
        total_pos_err = 0.0
        total_ang_err = 0.0

        for pos_err, ang_err, item_name, g_name in candidates:
            if item_name in used_items or g_name in used_targets:
                continue
            used_items.add(item_name)
            used_targets.add(g_name)
            assignments.append((item_name, g_name))
            total_pos_err += pos_err
            total_ang_err += ang_err

        completed_target_names = [g for (_, g) in assignments]
        completed_items_names = [it for (it, _) in assignments]
        return completed_target_names, completed_items_names, assignments, total_pos_err, total_ang_err


    def perform_inference(
        self,
        goals_dict: Dict[str, dict],
        current_geometry_positions_dict: Dict[str, Frame],
        incorrect_goals_list: List[str] = [],
        position_threshold: float = 0.03,
        rotation_threshold: float = 3.0,
    ) -> Dict[str, object]:
        """
        Suggest the next goal & target based on current geometry.

        Returns dict:
            - suggested_goal: str ('Goal00', 'Goal01', ...)
            - suggested_target_name: str ('G0','G1',...) or None if complete
            - completed_target_names: List[str]
            - suggested_target_frame: Frame or None
            - completed_items_names: List[str]
            - incompleted_items_names: List[str]
        """
        if not goals_dict:
            raise ValueError("perform_inference: goals_dict is empty.")
        if not isinstance(current_geometry_positions_dict, dict) or not current_geometry_positions_dict:
            raise ValueError("perform_inference: current_geometry_positions_dict is empty.")

        incorrect = set(incorrect_goals_list or [])

        # Evaluate candidate goals
        best = None  # (rank_key, payload)
        for goal_name, goal_data in goals_dict.items():
            if goal_name in incorrect:
                print (f"SimpleInference: skipping incorrect goal '{goal_name}'.")
                continue
            cube_locations = goal_data.get("cube_locations", {})
            if not cube_locations:
                raise ValueError(f"perform_inference: goal '{goal_name}' has no 'cube_locations'.")
                continue

            completed_target_names, completed_items_names, assignments, pos_err_sum, ang_err_sum = self._match_geometry_to_targets(
                cube_locations,
                current_geometry_positions_dict,
                position_threshold,
                rotation_threshold,
            )

            num_matches = len(assignments)
            # Rank: more matches is better, then lower total error; tie-break by goal_name
            rank_key = (-num_matches, pos_err_sum, ang_err_sum, goal_name)

            payload = {
                "goal_name": goal_name,
                "cube_locations": cube_locations,
                "completed_target_names": completed_target_names,
                "completed_items_names": completed_items_names,
                "pos_err_sum": pos_err_sum,
                "ang_err_sum": ang_err_sum,
            }

            if best is None or rank_key < best[0]:
                best = (rank_key, payload)

        # If nothing met thresholds, deterministic fallback: first goal not incorrect, lowest G as suggestion
        if best is None:
            # Choose a stable goal order by name
            candidate_goal_names = sorted([g for g in goals_dict.keys() if g not in incorrect])
            fallback_goal_name = candidate_goal_names[0] if candidate_goal_names else next(iter(goals_dict.keys()))
            cube_locations = goals_dict[fallback_goal_name].get("cube_locations", {})
            all_g = self._sorted_g_names(list(cube_locations.keys()))
            suggested_target_name = all_g[0] if all_g else None
            suggested_target_frame = cube_locations[suggested_target_name].frame if suggested_target_name else None

            return {
                "suggested_goal": fallback_goal_name,
                "suggested_target_name": suggested_target_name,
                "completed_target_names": [],
                "suggested_target_frame": suggested_target_frame,
                "completed_items_names": [],
                "incompleted_items_names": list(current_geometry_positions_dict.keys()),
            }

        # Build final result from best goal
        payload = best[1]
        goal_name = payload["goal_name"]
        cube_locations = payload["cube_locations"]

        completed_target_names = self._sorted_g_names(payload["completed_target_names"])
        completed_items_names = payload["completed_items_names"]

        # 2.5) Lowest incomplete target name
        all_g_sorted = self._sorted_g_names(list(cube_locations.keys()))
        completed_set = set(completed_target_names)
        suggested_target_name = next((g for g in all_g_sorted if g not in completed_set), None)
        suggested_target_frame = cube_locations[suggested_target_name].frame if suggested_target_name else None

        # Completed vs incompleted items (based on what we matched)
        all_items = list(current_geometry_positions_dict.keys())
        incompleted_items_names = [n for n in all_items if n not in set(completed_items_names)]

        return {
            "suggested_goal": goal_name,
            "suggested_target_name": suggested_target_name,
            "completed_target_names": completed_target_names,
            "suggested_target_frame": suggested_target_frame,
            "completed_items_names": completed_items_names,
            "incompleted_items_names": incompleted_items_names,
        }

