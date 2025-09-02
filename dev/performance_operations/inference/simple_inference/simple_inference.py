from compas.geometry import Frame, Quaternion, Vector, Point, distance_point_point
import math


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

    def perform_inference(self, goals_dict, current_geometry_positions_dict, incorrect_goals_list, position_threshold=0.03, rotation_threshold=3):
        """
        Perform inference to suggest the next goal based on current geometry positions. Returns the suggested goal, target name, completed and incompleted items.
        suggested_goal, suggested_target_name, completed_goal_indexes, target_frame, completed_items_names, incompleted_items_names
        """
        
        

        pass
