import os

from compas.datastructures import Assembly
from compas.datastructures import Mesh
from compas.datastructures import Part
from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Transformation
from compas_rhino.conversions import plane_to_compas_frame, frame_to_rhino_plane
from compas.geometry import Translation
from compas.geometry import Box
from Rhino.Geometry import Point3d

from copy import deepcopy
import math


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


    #TEST ME
    def _create_box_geo(self, frame, xsize, ysize, zsize):
        # Create a box geometry for each item in the dictionary

        x_vec = frame.x()
        y_vec = frame.y()
        z_vec = frame.z()

        x_trans = -x_vec ((xsize/2) - 24.5)
        y_trans = -y_vec ((ysize/2) - 52)
        z_trans = -z_vec * (zsize/2)

        translation_vector = [x_trans ,y_trans, z_trans]

        frame_point = frame.point
        tx = Transformation.from_translation(translation_vector)
        point_translated = frame_point.transformed(tx)
        frame = Frame(point_translated, x_vec, y_vec)
        box = Box(frame, xsize, ysize, zsize)

        transformed_box_frame = Frame(frame_z_transformed_point, frame.xaxis, frame.yaxis)
    # return None
    
    

