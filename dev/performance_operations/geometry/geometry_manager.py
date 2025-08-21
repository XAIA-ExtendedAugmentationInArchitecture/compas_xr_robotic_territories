# from compas.geometry import Transformation

#TODO: This should connect to the REALTIME DATABASE

import os
from compas.data import json_load, json_dump, json_dumps
from compas.geometry import Frame, Box, Transformation, Quaternion
from scipy.spatial.transform import Rotation as R

from compas_xr.realtime_database import RealtimeDatabase



class GeometryManager:

    def __init__(self, rigid_body_names, marker_types, box_xsize, box_ysize, box_zsize, firebase_upload=False):
        self.project_config_dict = self._load_project_config_dict() 
        self.active_geometry_dict = self._create_init_geometry_dict(rigid_body_names, marker_types, box_xsize, box_ysize, box_zsize)
    
        self.ACTIVE_MARKER_RHINO_TRANSFORMATION, self.PASSIVE_MARKER_RHINO_TRANSFORMATION = self._load_rhino_transformatons_from_project_config_dict(self.project_config_dict)
        self.ACTIVE_MARKER_CUBE_CENTER_OFFSET_MOTIVE, self.PASSIVE_MARKER_CUBE_CENTER_OFFSET_MOTIVE = self._load_motive_cube_center_offsets_from_project_config_dict(self.project_config_dict)

        #Construct Geometry Reference for RTDB
        self._upload_to_firebase = firebase_upload
        self.realtime_database, self.project_name = self._setup_realtime_database(self.project_config_dict)
        self.observed_geometry_db_reference_list = [self.project_name, "geometry", "observed_geometries"]
        self.goal_geometry_reference_list = [self.project_name, "geometry", "goal_geometries"]

    def _load_project_config_dict(self):
        dir_path = os.path.dirname(__file__)
        root = os.path.abspath(os.path.join(dir_path, os.pardir))
        config_fp = os.path.join(root, "project_config.json")
        config = json_load(config_fp)
        return config
    
    def _create_init_geometry_dict(self, rigid_body_names, marker_types, box_xsize, box_ysize, box_zsize):
        geometry_dict = {}

        for key, rigid_body_name in rigid_body_names.items():
            if rigid_body_name == "UR20" or rigid_body_name == "UR3Table" or rigid_body_name == "ABBTable" or rigid_body_name == "Origin":
                continue
            geometry_dict[rigid_body_name] = {}
            box = Box(box_xsize, box_ysize, box_zsize, Frame.worldXY())
            geometry_dict[rigid_body_name]['box'] = box
            geometry_dict[rigid_body_name]['marker_type'] = marker_types[key]
        return geometry_dict
    
    def _load_rhino_transformatons_from_project_config_dict(self, project_config_dict):
        geo_all   = project_config_dict.get("geometry_transformations", {})
        rhino_all = geo_all.get("rhino", {})

        # 2) Pull out the raw JSON data for each marker type
        active_data  = rhino_all.get("active_marker", {})
        passive_data = rhino_all.get("passive_marker", {})

        # 3) Warn if either is missing entirely
        if not active_data:
            print("No 'geometry_transformations.rhino.active_marker' data—using identity transform")
        if not passive_data:
            print("No 'geometry_transformations.rhino.passive_marker' data—using identity transform")

        # 4) Load via COMPAS’s Data API, or default to identity
        try:
            ACTIVE_MARKER_GEO_TRANSFORMATION = Transformation.__from_data__(active_data)
        except Exception:
            print("Failed to load 'geometry_transformations.rhino.active_marker' data—using identity transform")
            ACTIVE_MARKER_GEO_TRANSFORMATION = Transformation()

        try:
            PASSIVE_MARKER_GEO_TRANSFORMATION = Transformation.__from_data__(passive_data)
        except Exception:
            print("Failed to load 'geometry_transformations.rhino.active_marker' data—using identity transform")
            PASSIVE_MARKER_GEO_TRANSFORMATION = Transformation()
        
        return ACTIVE_MARKER_GEO_TRANSFORMATION, PASSIVE_MARKER_GEO_TRANSFORMATION
    
    def _load_motive_cube_center_offsets_from_project_config_dict(self, project_config_dict):
        geo_all   = project_config_dict.get("geometry_transformations", {})
        motive_all = geo_all.get("motive", {})

        active_data  = motive_all.get("ACTIVE_MARKER_CUBE_CENTER_OFFSET_MOTIVE", {})
        passive_data = motive_all.get("PASSIVE_MARKER_CUBE_CENTER_OFFSET_MOTIVE", {})

        if not active_data:
            print("No 'geometry_transformations.motive.ACTIVE_MARKER_CUBE_CENTER_OFFSET_MOTIVE' data—using identity transform")
        if not passive_data:
            print("No 'geometry_transformations.motive.PASSIVE_MARKER_CUBE_CENTER_OFFSET_MOTIVE' data—using identity transform")
        
        return active_data, passive_data

    def _setup_realtime_database(self, project_config_dict):
        rt_db_config_fp = project_config_dict.get("firebase_config_fp", {})
        if not rt_db_config_fp:
            raise ValueError("No 'firebase_config_fp' found in project_config.json")
            return None, None
        rt_db = RealtimeDatabase(rt_db_config_fp)
        project_name = project_config_dict["project_name"]
        goal_geometry_reference_list = [project_name, "geometry", "goals_list"] 
        if not project_name:
            raise ValueError("No 'project_name' found in project_config.json")
            return None, None
        return rt_db, project_name

    ##########################################################################################################################
    # ABOVE ARE THE PRIVATE METHODS
    ##########################################################################################################################

    def apply_local_offset_to_cube_center_for_motive_data(self, raw_point, raw_rot, marker_type):
        """
        raw_point: dict {"x":…, "y":…, "z":…}
        raw_rot:   dict {"x":…, "y":…, "z":…, "w":…}
        """
        if marker_type == "unique":
            return None

        q = Quaternion(raw_rot["w"], raw_rot["x"], raw_rot["y"], raw_rot["z"])

        if marker_type == "active":
            local_off = self.ACTIVE_MARKER_CUBE_CENTER_OFFSET_MOTIVE
        elif marker_type == "passive":
            local_off = self.PASSIVE_MARKER_CUBE_CENTER_OFFSET_MOTIVE
        else:
            raise ValueError(f"Unknown marker type: {marker_type}")

        # rotate into world-space (using scipy as before)
        rot = R.from_quat([q.x, q.y, q.z, q.w])
        world_off = rot.apply(local_off)

        # extract x,y,z from the input dict and add:
        return {
            "x": raw_point["x"] + world_off[0],
            "y": raw_point["y"] + world_off[1],
            "z": raw_point["z"] + world_off[2],
        }
    
    def update_geometry_dict(self, model_name, geo_frame):
        if model_name in self.active_geometry_dict:
            box = self.active_geometry_dict[model_name]['box']
            box.frame = geo_frame
            self.active_geometry_dict[model_name]['box'] = box
            print(f"GeometryManager: [GeometryManager] Updated geometry for '{model_name}' with frame {geo_frame}")
            print(f"GeometryManager: [GeometryDict] Current keys: {list(self.active_geometry_dict.keys())}")

            if self._upload_to_firebase:
                self.upload_current_geometry_to_realtime_database(self.active_geometry_dict)
            else:
                print(f"GeometryManager: [GeometryManager] Not uploading to Firebase, upload_to_firebase is set to {self._upload_to_firebase}")
        else:
            print(f"GeometryManager: [GeometryManager] Warning: model_name '{model_name}' not found in active_geometry_dict.")
            print(f"YOU SHOULD THINK ABOUT IF YOU NEED TO ADD THIS TO THE DICT.... {model_name}")
            print(f"Current keys: {list(self.active_geometry_dict.keys())}")

    def apply_transformation_for_rhino_geometry(self, model_name, rhino_frame_from_motive_observation, marker_type):
        if marker_type == "unique":
            return None
        elif marker_type == "active":
            geo_frame = rhino_frame_from_motive_observation.transformed(self.ACTIVE_MARKER_RHINO_TRANSFORMATION)
        elif marker_type == "passive":
            geo_frame = rhino_frame_from_motive_observation.transformed(self.PASSIVE_MARKER_RHINO_TRANSFORMATION)
        else:
            raise ValueError(f"Unknown marker type: {marker_type}")
        
        self.update_geometry_dict(model_name, geo_frame)
        print(f"GeometryManager: [GeometryManager] Updated geometry for '{model_name}' with marker type '{marker_type}'")
        return geo_frame

    #TODO: This should upload the current geometry to the Realtime Database ONCE YOU SORT OUT THE STRUCTURE YOU WANT.
    def upload_current_geometry_to_realtime_database(self, data):
        if not self.realtime_database or not self.project_name:
            print("GeometryManager: [GeometryManager] Realtime database not set up correctly.")
            return
        else:
            print("HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE")
            self.realtime_database.upload_data_to_deep_reference(data=data, reference_list=self.observed_geometry_db_reference_list)
            print(f"GeometryManager: [GeometryManager] Uploading current geometry to Realtime Database tracking {len(self.active_geometry_dict)}")