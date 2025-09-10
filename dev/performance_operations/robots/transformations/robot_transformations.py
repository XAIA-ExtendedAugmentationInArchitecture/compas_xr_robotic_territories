import numpy as np
from compas.geometry import Point, Quaternion, Frame, Vector
from compas.data import json_load, json_dump
from scipy.spatial.transform import Rotation as R
from compas.geometry import Transformation
from compas.geometry import Translation
from compas_xr.realtime_database import RealtimeDatabase
import os

class RobotTransformationsFromObserved:

    def __init__(self, project_config_fp):
        """
        Initialize the RobotTransformationsFromObserved class with a file path to the transformations.
        :param transformations_fp: File path to the transformations JSON file.
        """

        self._project_config_dict = self._load_project_config_dict(project_config_fp)
        self.transformations_fp = self._project_config_dict.get("robot_transformations_fp", None)
        self.project_name = self._project_config_dict.get("project_name", None)
        self.fb_config_fp = self._project_config_dict.get("firebase_config_fp", None)
        # self.optitrack_info_dict = self._project_config_dict.get("optitrack_info", None)

        if not os.path.exists(self.fb_config_fp):
            raise ValueError("Firebase configuration file path must be provided.")
        if not os.path.exists(self.transformations_fp):
            raise ValueError("Transformations file path must be provided in the project configuration.")

        self.transformations = self._load_transformations()
        self.rtdb_reference = RealtimeDatabase(self.fb_config_fp)
        print(f"RobotTransformationsFromObserved : [RobotTransformationsFromObserved] Initialized with RTDB for project: {self.project_name}")

    def _load_project_config_dict(self, project_config_fp):
        """
        Load the project configuration dictionary from the specified file path.
        :return: Dictionary containing the project configuration.
        """
        if not os.path.exists(project_config_fp):
            raise FileNotFoundError(f"Project configuration file not found: {project_config_fp}")
        config_dict = json_load(project_config_fp)
        return config_dict
    
    def _load_transformations(self):
        """
        Load transformations from the specified JSON file.
        
        :return: Dictionary containing the transformations.
        """
        try:
            return json_load(self.transformations_fp)
        except Exception as e:
            print(f"Error loading transformations from {self.transformations_fp}: {e}")
            return {}

    def update_robot_transformation(self, robot_name, observed_frame):
        """
        Update the robot transformation based on the observed frame.
        
        :param robot_name: Name of the robot.
        :param observed_frame: The observed frame to be transformed.
        """

        if robot_name not in self.transformations:
            print(f"Robot {robot_name} not found in transformations.")
            return
        else:
            if robot_name == "UR20":
                # Special handling for UR20
                self.update_observed_transforms_for_ur20(robot_name, observed_frame)
            elif robot_name == "UR3Table":
                # Special handling for UR3
                self.update_observed_transforms_for_UR3Table(robot_name, observed_frame)
            elif robot_name == "ABBTable":
                # Special handling for SmallAbbs
                self.update_observed_transforms_for_ABBTable(robot_name, observed_frame)
            else:
                print(f"Robot {robot_name} does not have a specific transformation update method.")

    # ========================================================================================
    # Update Tansformations for the UR20 Robots
    # ========================================================================================

    def update_observed_transforms_for_ur20(self, robot_name, observed_frame, translation_dist = 0.4577, vertical_z_translation = 0.0135):
        """
        Update the observed transformations for the UR20 robot.
        This includes a translation for the observed frame to the actual frame
        and a transformation from the translated frame to the Rhino frame.
        """
        urdf_base_frame = self.transformations[robot_name]["static"]["urdf_base_frame"]

        translation_vector = observed_frame.xaxis + observed_frame.yaxis
        unitized_translation_vector = translation_vector.unitized()
        translation_vector = unitized_translation_vector * translation_dist
        translation = Translation.from_vector(translation_vector)

        observed_base_frame = observed_frame.transformed(translation)

        #TODO: Testing I just added this based on the Rhino model, but should verify with real life
        #TODO: Previous was just a ranslation in the XY axis by 0.517 m
        translation_vector_z = Vector.Zaxis() * (-vertical_z_translation)
        z_translation = Translation.from_vector(translation_vector_z)
        observed_base_frame_z_translated = observed_base_frame.transformed(z_translation)
        #TODO: Testing I just added this based on the Rhino model, but should verify with real life

        transformation_to_urdf_base = Transformation.from_frame_to_frame(urdf_base_frame, observed_base_frame_z_translated)
        inverse_transformation_to_observed_base = transformation_to_urdf_base.inverse()

        self.transformations[robot_name]["observed"]["urdf_base_frame"] = observed_base_frame
        self.transformations[robot_name]["observed"]["transformation_to_urdf"] = transformation_to_urdf_base
        self.transformations[robot_name]["observed"]["inverse_transform_to_observed"] = inverse_transformation_to_observed_base

        #TODO: I THINK THE TRANSFORMATIONS SHOULD BE UPLOADED TO FIREBASE AND PULLED ON THE ROBOT COMMUNICATION SIDE... THIS WOULD SLOW COMMUNICATION A BIT, BUT MAKES IT DYNAMIC.
        # Update the base frame on Firebase & save the transformations
        json_dump(self.transformations, self.transformations_fp, pretty=True)
        self.update_baseframe_on_firebase(robot_name, observed_base_frame)
        print(f"Updated observed transformations for {robot_name} with base frame: {observed_base_frame}")

    # ========================================================================================
    # Update Tansformations for the UR3 Robots
    # ========================================================================================

    def update_observed_transforms_for_UR3Table(self, robot_name, observed_frame):
        """
        Update the observed transformations for the UR3 robot.
        This includes a translation for the observed frame to the actual frame
        and a transformation from the translated frame to the Rhino frame.
        """
        ur3_table_data = self.transformations[robot_name]
        # self.update_observed_transforms_for_UR31("UR31", observed_frame)
        # self.update_observed_transforms_for_UR32("UR32", observed_frame)

        print(f"WIP : Transformation for UR3Table is not implemented yet. Current data: {ur3_table_data}")

    def update_observed_transforms_for_UR31(self, robot_name, observed_frame): #TODO: UR3
        """
        Update the observed transformations for the UR31 robot.
        This includes a translation for the observed frame to the actual frame
        """
        urdf_base_frame = self.transformations["UR3Table"][robot_name]["static"]["urdf_base_frame"]
        print(f"Updating Observed Transformation for UR1 robot: {robot_name} static base frame: {urdf_base_frame}, observed frame: {observed_frame}, current information: {self.transformations['UR3Table'][robot_name]['observed']}")

        #TODO: Insert Transformation Logic here.

        # transformation_to_urdf_base = Transformation.from_frame_to_frame(urdf_base_frame, observed_base_frame)
        # inverse_transformation_to_observed_base = transformation_to_urdf_base.inverse()

        # self.transformations["UR3Table"][robot_name]["observed"]["urdf_base_frame"] = observed_base_frame
        # self.transformations["UR3Table"][robot_name]["observed"]["transformation_to_urdf"] = transformation_to_urdf_base
        # self.transformations["UR3Table"][robot_name]["observed"]["inverse_transform_to_observed"] = inverse_transformation_to_observed_base
        # self.update_baseframe_on_firebase("UR31", observed_frame)

    def update_observed_transforms_for_UR32(self, robot_name, observed_frame): #TODO: UR3 2
        """
        Update the observed transformations for the UR32 robot.
        This includes a translation for the observed frame to the actual frame
        """
        urdf_base_frame = self.transformations["UR3Table"][robot_name]["static"]["urdf_base_frame"]
        print(f"Updating Observed Transformation for UR1 robot: {robot_name} static base frame: {urdf_base_frame}, observed frame: {observed_frame}, current information: {self.transformations['UR3Table'][robot_name]['observed']}")

        #TODO: Insert Transformation Logic here.

        # transformation_to_urdf_base = Transformation.from_frame_to_frame(urdf_base_frame, observed_base_frame)
        # inverse_transformation_to_observed_base = transformation_to_urdf_base.inverse()

        # self.transformations["UR3Table"][robot_name]["observed"]["urdf_base_frame"] = observed_base_frame
        # self.transformations["UR3Table"][robot_name]["observed"]["transformation_to_urdf"] = transformation_to_urdf_base
        # self.transformations["UR3Table"][robot_name]["observed"]["inverse_transform_to_observed"] = inverse_transformation_to_observed_base
        # self.update_baseframe_on_firebase("UR32", observed_frame)

    # ========================================================================================
    # Update Tansformations for the ABB Robots
    # ========================================================================================

    def update_observed_transforms_for_ABBTable(self, robot_name, observed_frame):
        """
        Update the observed transformations for the ABB robot table.
        This includes a translation for the observed frame to the actual frame
        and a transformation from the translated frame to the Rhino frame.
        """
        abb_table_data = self.transformations[robot_name]

        # self.update_observed_transforms_for_UR31("ABB1", observed_frame)
        # self.update_observed_transforms_for_UR32("ABB2", observed_frame)

        print(f"WIP : Transformation for ABBTable is not implemented yet. Current data: {abb_table_data}")

    def update_observed_transforms_for_ABB1(self, robot_name, observed_frame): #TODO: ABB1
        """
        Update the observed transformations for the ABB-1 robot.
        This includes a translation for the observed frame to the actual frame
        and a transformation from the translated frame to the Rhino frame.
        """
        urdf_base_frame = self.transformations["ABBTable"][robot_name]["static"]["urdf_base_frame"]
        print(f"Updating Observed Transformation for ABB1 robot: {robot_name} static base frame: {urdf_base_frame}, observed frame: {observed_frame}, current information: {self.transformations['ABBTable'][robot_name]['observed']}")

        #TODO: Insert Transformation Logic here.

        # transformation_to_urdf_base = Transformation.from_frame_to_frame(urdf_base_frame, observed_base_frame)
        # inverse_transformation_to_observed_base = transformation_to_urdf_base.inverse()

        # self.transformations["ABBTable"][robot_name]["observed"]["urdf_base_frame"] = observed_base_frame
        # self.transformations["ABBTable"][robot_name]["observed"]["transformation_to_urdf"] = transformation_to_urdf_base
        # self.transformations["ABBTable"][robot_name]["observed"]["inverse_transform_to_observed"] = inverse_transformation_to_observed_base
        # self.update_baseframe_on_firebase("ABB1", observed_frame)

    def update_observed_transforms_for_ABB2(self, robot_name, observed_frame): #TODO: ABB2
        """
        Update the observed transformations for the ABB-2 robot.
        This includes a translation for the observed frame to the actual frame
        and a transformation from the translated frame to the Rhino frame.
        """
        urdf_base_frame = self.transformations["ABBTable"][robot_name]["static"]["urdf_base_frame"]
        print(f"Updating Observed Transformation for ABB2 robot: {robot_name} static base frame: {urdf_base_frame}, observed frame: {observed_frame}, current information: {self.transformations['ABBTable'][robot_name]['observed']}")

        #TODO: Insert Transformation Logic here.

        # transformation_to_urdf_base = Transformation.from_frame_to_frame(urdf_base_frame, observed_base_frame)
        # inverse_transformation_to_observed_base = transformation_to_urdf_base.inverse()

        # self.transformations["ABBTable"][robot_name]["observed"]["urdf_base_frame"] = observed_base_frame
        # self.transformations["ABBTable"][robot_name]["observed"]["transformation_to_urdf"] = transformation_to_urdf_base
        # self.transformations["ABBTable"][robot_name]["observed"]["inverse_transform_to_observed"] = inverse_transformation_to_observed_base
        # self.update_baseframe_on_firebase("ABB2", observed_frame)

    # ========================================================================================
    # Update Base Frames on Firebase
    # ========================================================================================

    def update_baseframe_on_firebase(self, robot_name, tansformed_frame):
        """
        Update the base frame of the robot on Firebase.
        
        :param robot_name: Name of the robot.
        :param transformed_frame: The transformed frame to be updated.
        """
        # Placeholder for Firebase update logic
        database_reference = self.rtdb_reference
        ref_list = [self.project_name, "robot_base_frame", robot_name]
        print(f"Reference list for {robot_name}: {ref_list}")
        database_reference.upload_data_to_deep_reference(tansformed_frame.__data__, ref_list)
        print(f"Updating base frame for {robot_name} on Firebase with frame: {tansformed_frame}")

#TODO: ##################################################################C

"""
This file will make the correct transformations, but it needs to be supplimented by the 
optitrack tracking file, static robotic configuration files, and additional other information
for the creation of geometry.
"""

 #TODO : EXACT THINGS ###################################################C

  # TODO: 1 Robot Transformations Class (Translations from observed to actual)
  # TODO: 2 Overwrite the transformation in a static json configuration file
  # TODO: 3 Overwrite information on the FB for the AR Robot

#TODO: ##################################################################C


# # ------------------------------------------------------------------------
# # Transformations From Motive to Rhino
# # -----------------------------------------------------------------------

# # ------------------------------------------------------------------------
# # Coordinate Transform: Motive (Z forward, Y up, X left) → Rhino (Z up)
# # ------------------------------------------------------------------------

# MOTIVE_ZFWD_TO_RHINO_ZUP_4x4 = np.array([
#     [-1, 0,  0, 0],  # X (left) → -X (right)
#     [ 0, 0,  1, 0],  # Z (fwd)  → Y (fwd)
#     [ 0, 1,  0, 0],  # Y (up)   → Z (up)
#     [ 0, 0,  0, 1]
# ])

# # ------------------------------------------------------------------------
# # Helper Functions # TODO: These move to the motive file
# # ------------------------------------------------------------------------

# def get_motive_pose(pos_dict, quat_dict) -> tuple[Point, Quaternion]:
#     point = Point(pos_dict["x"], pos_dict["y"], pos_dict["z"])
#     quat = Quaternion(quat_dict["w"], quat_dict["x"], quat_dict["y"], quat_dict["z"])
#     return point, quat

# def transform_point(point: Point, matrix: np.ndarray) -> Point:
#     p = np.array([point.x, point.y, point.z, 1.0])
#     transformed = matrix @ p
#     return Point(*transformed[:3])

# def transform_quaternion(q: Quaternion, matrix: np.ndarray) -> Quaternion:
#     R_motive = R.from_quat([q.x, q.y, q.z, q.w])
#     C = matrix[:3, :3]
#     R_rhino = C @ R_motive.as_matrix() @ C.T
#     q_rhino = R.from_matrix(R_rhino).as_quat()
#     return Quaternion(q_rhino[3], q_rhino[0], q_rhino[1], q_rhino[2])

# ------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------

# if __name__ == "__main__":

#     # Input/output file paths
#     # fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\nat_net_lib_test\samples\out\20250603_robot_transformation_testing\recordings\20250603_142657\rigid_bodies_by_timestamp.json"
#     fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\nat_net_lib_test\samples\out\20250603_robot_transformation_testing_2\recordings\20250603_182112\rigid_bodies_by_timestamp.json"
#     outfp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\nat_net_lib_test\samples\out\20250603_robot_transformation_testing_2\recordings\20250603_182112\frame_transformation_tests_22.json"

#     # Load data
#     data = json_load(fp)
#     ur20_data = data["UR20"]
#     print(len(ur20_data))
#     blue01_data = data["Blue01"]
#     print(len(blue01_data))
#     red01_data = data["Red01"]
#     print(len(red01_data))
#     origin_data = data["Origin"]

#     # Prepare output
#     item_data_dict = {
#         "motive_frames": [],
#         "rhino_frames": [],
#     }

#     data_dict = {
#         "UR20": {
#             "motive_frames": [],
#             "rhino_frames": [],
#         },
#         "Blue01": {
#             "motive_frames": [],
#             "rhino_frames": [],
#         },
#         "Red01": {
#             "motive_frames": [],
#             "rhino_frames": [],
#         },
#     }

#     # Convert UR20 poses
#     for entry in ur20_data:
#         point_motive, quat_motive = get_motive_pose(entry["position"], entry["rotation"])

#         point_rhino = transform_point(point_motive, MOTIVE_ZFWD_TO_RHINO_ZUP_4x4)
#         quat_rhino = transform_quaternion(quat_motive, MOTIVE_ZFWD_TO_RHINO_ZUP_4x4)

#         frame_motive = Frame.from_quaternion(quat_motive, point_motive)
#         frame_rhino = Frame.from_quaternion(quat_rhino, point_rhino)

#         data_dict["UR20"]["motive_frames"].append(frame_motive)
#         data_dict["UR20"]["rhino_frames"].append(frame_rhino)

#     # TODO: TEST THIS......

#     # Convert Blue01 poses
#     for entry in blue01_data:
#         point_motive, quat_motive = get_motive_pose(entry["position"], entry["rotation"])

#         point_rhino = transform_point(point_motive, MOTIVE_ZFWD_TO_RHINO_ZUP_4x4)
#         quat_rhino = transform_quaternion(quat_motive, MOTIVE_ZFWD_TO_RHINO_ZUP_4x4)

#         frame_motive = Frame.from_quaternion(quat_motive, point_motive)
#         frame_rhino = Frame.from_quaternion(quat_rhino, point_rhino)

#         data_dict["Blue01"]["motive_frames"].append(frame_motive)
#         data_dict["Blue01"]["rhino_frames"].append(frame_rhino)

#     # Convert Red01 poses
#     for entry in red01_data:
#         point_motive, quat_motive = get_motive_pose(entry["position"], entry["rotation"])

#         point_rhino = transform_point(point_motive, MOTIVE_ZFWD_TO_RHINO_ZUP_4x4)
#         quat_rhino = transform_quaternion(quat_motive, MOTIVE_ZFWD_TO_RHINO_ZUP_4x4)

#         frame_motive = Frame.from_quaternion(quat_motive, point_motive)
#         frame_rhino = Frame.from_quaternion(quat_rhino, point_rhino)

#         data_dict["Red01"]["motive_frames"].append(frame_motive)
#         data_dict["Red01"]["rhino_frames"].append(frame_rhino)

#     # Convert Origin pose
#     origin_pos = origin_data[0]["position"]
#     origin_rot = origin_data[0]["rotation"]

#     point_motive, quat_motive = get_motive_pose(origin_pos, origin_rot)
#     point_rhino = transform_point(point_motive, MOTIVE_ZFWD_TO_RHINO_ZUP_4x4)
#     quat_rhino = transform_quaternion(quat_motive, MOTIVE_ZFWD_TO_RHINO_ZUP_4x4)

#     data_dict["origin_frame_motive"] = Frame.from_quaternion(quat_motive, point_motive)
#     data_dict["origin_frame_rhino"] = Frame.from_quaternion(quat_rhino, point_rhino)

#     # Write transformed output
#     json_dump(data=data_dict, fp=outfp, pretty=True)