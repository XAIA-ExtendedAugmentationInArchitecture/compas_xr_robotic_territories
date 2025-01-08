import os

from compas.datastructures import Assembly
from compas.datastructures import Mesh
from compas.datastructures import Part
from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Transformation
from compas_rhino.conversions import plane_to_compas_frame, frame_to_rhino_plane, box_to_compas, point_to_compas
from compas.geometry import Translation
from compas.geometry import Box
from Rhino.Geometry import Point3d
import Rhino.Geometry as rg

from copy import deepcopy
import math

from compas_xr.project.assembly_extensions import AssemblyExtensions
from compas_xr.realtime_database import RealtimeDatabase
from compas_xr.storage import Storage
from compas_xr.project.project_manager import ProjectManager

class OptiTrackConversions(object):

    def convert_to_rhino_planes(self, optitrack_planes):
        rhino_planes = []
        for plane in optitrack_planes:
            new_point = Point3d(-plane.Origin.X, plane.Origin.Z, plane.Origin.Y)
            plane.Origin = new_point
            rhino_planes.append(plane)
        return rhino_planes

class OptitrackItems(object):
    
    def __init__(self, rb_ids, rb_names, rb_planes, rb_frames=None):
        # Initialize the dictionary
        self.names = rb_names
        self.planes = rb_planes
        self.frames = rb_frames or [plane_to_compas_frame(plane) for plane in rb_planes]
        self.ids = rb_ids
        self.dict = self._create_items_dict(rb_ids, rb_names, rb_planes)
        # Add list attribute for geomtry

    # Create box geometry in a method.

    # move method for transforming to an input plane in this class.
    # double check if transformed will make a new instance of this class and return that.... this might be the best way to go.

    def _create_items_dict(self, rb_ids, rb_names, rb_planes):
        # Create a dictionary to store items
        items_dict = {}
        for i in range(len(rb_names)):
            name = rb_names[i]
            id = rb_ids[i]
            plane = rb_planes[i]
            # Populate the dictionary
            items_dict[name] = {
                "id": id,
                "rhino_plane": plane,
                "compas_frame": plane_to_compas_frame(plane),
            }
        return items_dict
    
    def transformed_to_alignment_frame(self, alignment_frame):
        """
        Transform the assembly to the origin based on the OptiTrack data,
        and return a new instance of the class with transformed data.

        Parameters
        ----------
        alignment_frame : compas.geometry.Frame
            The target alignment frame.

        Returns
        -------
        OptitrackItems
            A new instance of the OptitrackItems class, transformed to the alignment frame.
        """
        # Get the origin frame from the current instance
        optitrack_origin = self.dict['Origin']["compas_frame"]

        # Compute the transformation
        transformation = Transformation.from_frame_to_frame(optitrack_origin, alignment_frame)

        # Create transformed data
        transformed_dict = deepcopy(self.dict)
        transformed_planes = []
        transformed_frames = []

        for key, value in transformed_dict.items():
            frame = value['compas_frame']
            transformed_frame = frame.transformed(transformation)
            transformed_frames.append(transformed_frame)
            transformed_planes.append(frame_to_rhino_plane(transformed_frame))

            # Update the dictionary for the transformed instance
            value['compas_frame'] = transformed_frame
            value['rhino_plane'] = frame_to_rhino_plane(transformed_frame)

        # Create a new instance of the class with transformed data
        transformed_instance = OptitrackItems(
            rb_ids=self.ids,
            rb_names=self.names,
            rb_planes=transformed_planes,
            rb_frames=transformed_frames
        )
        return transformed_instance


    #INCORRECT NEEDS UPDATING
    def create_box_geometry_from_items_dict(self, optitrack_items_dict, xsize, ysize, zsize):
        boxes = []
        # Create a box geometry for each item in the dictionary
        for key, value in optitrack_items_dict.items():
            if key != 'Origin' and key != 'Robot':
                frame = value['compas_frame']
                box = self._create_box_geo(frame, xsize, ysize, zsize)
                boxes.append(box)
        return boxes
    
    #TEST ME
    def _create_box_geo(self, frame, xsize, ysize, zsize):
        """
        Create a box geometry for each item in the dictionary.
        """
        # Extract frame vectors
        x_vec = frame.xaxis
        y_vec = frame.yaxis
        z_vec = frame.zaxis

        # Compute individual translation components
        translation_vector = (x_vec.unitized() + -y_vec.unitized()).unitized()
        horiz_translation_dist = math.sqrt((xsize / 2)**2 + (ysize / 2)**2)

        # Scale the normalized vector
        horiz_translation_vector = translation_vector * horiz_translation_dist

        # Create the translation transformation
        translation = Translation.from_vector(horiz_translation_vector)

        # Create a translation for the negative z-axis
        neg_z_vec = z_vec.unitized() * -(zsize / 2)
        z_translation = Translation.from_vector(neg_z_vec)

        # Apply the translation to the frame's origin point
        frame_point = frame.point
        point_translated = frame_point.transformed(translation)
        point_z_translated = point_translated.transformed(z_translation)

        # Create a new frame and box geometry
        transformed_frame = Frame(point_z_translated, x_vec, y_vec)
        box = Box(xsize, ysize, zsize, transformed_frame)

        return box
    
class RoboticTerritoriesFormattingHelpers(object):

    def format_box_dict(self, rhino_box):
        """
        Converts a Rhino Box to a compas Box for returning required dictionary.
        
        Parameters:
        - box: Rhino.Geometry.Box, the input box.
    
        Returns:
        - compas box data (Compas.Geometry.Box.__data__).
        """
        if not isinstance(rhino_box, rg.Box):
            raise TypeError("The input is not a Rhino Box.")
        
        else:
            compas_box = box_to_compas(rhino_box)
            dict = compas_box.__data__
            return dict

    def format_polyline_to_compas_points(self, polyline):
        """
        Extract points from a Rhino.Geometry.Polyline.
    
        Parameters:
        - polyline: Rhino.Geometry.Polyline, the input polyline.
    
        Returns:
        - List of points (Rhino.Geometry.Point3d).
        """
        if isinstance(polyline, Polyline):  # Ensure it's a Polyline
            points = [point for point in polyline]
            compas_points = [point_to_compas(pt) for pt in points]
            points_dict = {index: pt.__data__ for index, pt in enumerate(compas_points)}
            print (points_dict)
            return points_dict

        else:
            raise TypeError("The input is not a Polyline.")
            
class RoboticTerritoriesProjectManager(object):
    # """
    # The ProjectManager class is responsible for managing project specific data and operations that involve
    # Firebase Storage and Realtime Database configuration.

    # Parameters
    # ----------
    # config_path : str
    #     The path to the configuration file for the project.

    # Attributes
    # ----------
    # storage : Storage
    #     The storage instance for the project.
    # database : RealtimeDatabase
    #     The realtime database instance for the project.
    # """

    def __init__(self, config_path):
        if not os.path.exists(config_path):
            raise Exception("Could not create Storage or Database with path {}!".format(config_path))
        self.storage = Storage(config_path)
        self.database = RealtimeDatabase(config_path)
        self.project_manager = ProjectManager(config_path)
        self.FormatHelpers = RoboticTerritoriesFormattingHelpers()

    
    def upload_rt_project_data(self, project_name, qr_frames, boundary_zone, infer_pick_zone, infer_collab_zone, mimic_human_zone, mimic_robot_zone, tele_mimic_zone):
        
        upload_dict = {}
        
        boundary_dict = self.FormatHelpers.format_box_dict(boundary_zone)
        upload_dict["boundry_zone"] = boundary_dict
        
        infer_dict = {}
        infer_pick_dict = self.FormatHelpers.format_box_dict(infer_pick_zone)
        infer_collab_dict = self.FormatHelpers.format_box_dict(infer_collab_zone)
        infer_dict["collaboration_zone"] = infer_collab_dict
        infer_dict["pick_zone"] = infer_pick_dict
        upload_dict["inferance_zones"] = infer_dict
        
        mimic_dict = {}
        mimic_human_dict = self.FormatHelpers.format_box_dict(mimic_human_zone)
        mimic_robot_dict = self.FormatHelpers.format_box_dict(mimic_robot_zone)
        mimic_dict["human_zone"] = mimic_human_dict
        mimic_dict["robot_zone"] = mimic_robot_dict
        upload_dict["mimic_zones"] = mimic_dict
        
        tele_mimic_dict = self.FormatHelpers.format_box_dict(tele_mimic_zone)
        upload_dict["tele_mimic_zone"] = tele_mimic_dict
        
        self.database.upload_data_to_reference_as_child(upload_dict, project_name, "zones")
        self.project_manager.upload_qr_frames_to_project(project_name, qr_frames)
        

