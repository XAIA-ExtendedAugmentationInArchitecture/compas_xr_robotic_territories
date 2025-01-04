import os

from compas.datastructures import Assembly
from compas.datastructures import Mesh
from compas.datastructures import Part
from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Transformation
from compas_rhino.conversions import plane_to_compas_frame, frame_to_rhino_plane

class OptitrackItems(object):
    
    def __init__(self, rb_ids, rb_names, rb_planes, rb_frames=None):
        # Initialize the dictionary
        self.dict = self._create_items_dict(rb_ids, rb_names, rb_planes)
        self.names = rb_names
        self.planes = rb_planes
        self.frames = rb_frames or [plane_to_compas_frame(plane) for plane in rb_planes]
        self.ids = rb_ids
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


class GeometryHelpers(object):
    """
    AssemblyExtensions is a class for extending the functionality of the :class:`~compas.datastructures.Assembly` class.

    The AssemblyExtensions class provides additional functionalities such as exporting parts as .obj files
    and creating a frame assembly from a list of :class:`~compas.geometry.Frame` with a specific data structure
    for localization information.

    """

    def transform_optitrack_items_to_origin(self, optitrack_items_dict, alignment_frame):
        """
        Transform the assembly to the origin based on the optitrack data.

        Parameters
        ----------
        optitrackdict : dict
            A dictionary with the optitrack data.

        """
        #Transform from the origin to the desired alignment frame
        optitrack_origin = optitrack_items_dict['Origin']["compas_frame"]
        transformation = Transformation.from_frame_to_frame(optitrack_origin, alignment_frame)

        for key, value in optitrack_items_dict.items():
            frame = value['compas_frame']
            transformed_frame = frame.transformed(transformation)
            value['compas_frame'] = transformed_frame
            value['rhino_plane'] = frame_to_rhino_plane(transformed_frame)

        return optitrack_items_dict
    
    

