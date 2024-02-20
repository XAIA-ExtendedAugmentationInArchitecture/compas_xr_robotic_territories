import os
from Rhino.Geometry import Mesh, MeshingParameters
from compas.datastructures import Assembly, Mesh
from compas.geometry import Transformation, Point, Vector, Frame
from compas_timber import assembly as TA

#TODO: Remove - This is a temporary fix for exporting Breps.
def mesh_to_compas(rhinomesh, cls=None):
    """Convert a Rhino mesh object to a COMPAS mesh.

    Parameters
    ----------
    rhinomesh : :class:`Rhino.Geometry.Mesh`
        A Rhino mesh object.
    cls: :class:`~compas.datastructures.Mesh`, optional
        The mesh type.

    Returns
    -------
    :class:`compas.datastructures.Mesh`
        A COMPAS mesh.

    """
    cls = cls or Mesh
    mesh = cls()
    mesh.default_vertex_attributes.update(normal=None, color=None)
    mesh.default_face_attributes.update(normal=None)

    vertexcolors = rhinomesh.VertexColors
    if not vertexcolors:
        vertexcolors = [None] * rhinomesh.Vertices.Count

    for vertex, normal, color in zip(rhinomesh.Vertices, rhinomesh.Normals, vertexcolors):
        mesh.add_vertex(
            x=vertex.X,
            y=vertex.Y,
            z=vertex.Z,
            normal=vector_to_compas(normal),
            color=Color(color.R, color.G, color.B) if color else None,
        )

    facenormals = rhinomesh.FaceNormals
    if not facenormals:
        facenormals = [None] * rhinomesh.Faces.Count

    for face, normal in zip(rhinomesh.Faces, facenormals):
        if face.IsTriangle:
            vertices = [face.A, face.B, face.C]
        else:
            vertices = [face.A, face.B, face.C, face.D]
        mesh.add_face(vertices, normal=vector_to_compas(normal) if normal else None)

    for key in rhinomesh.UserDictionary:
        mesh.attributes[key] = rhinomesh.UserDictionary[key]

    return mesh

class AssemblyExtensions(object):

    def __init__(self):
        pass

    #Functions for adding and editing attributes to assemblies
    def add_assembly_attributes(self, assembly, data_type, robot_keys=None, built_keys=None, planned_keys=None):
        
        data_type_list = ['0.Cylinder','1.Box','2.ObjFile','3.Mesh']

        data = assembly.__data__
        graph = assembly.graph.__data__
        graph_node = graph["node"]

        for key in graph_node:
            graph_node[str(key)]['type_id'] = key
            graph_node[str(key)]['type_data'] = data_type_list[data_type]
            graph_node[str(key)]['is_built'] = False
            graph_node[str(key)]['is_planned'] = False
            graph_node[str(key)]['placed_by'] = "human"

        for k in robot_keys:
            graph_node[str(k)]['placed_by'] = "robot"

        if built_keys:
            for l in built_keys:
                graph_node[str(l)]['is_built'] = True

        if planned_keys:
            for m in planned_keys:
                graph_node[str(m)]['is_planned'] = True

        assembly = Assembly.__from_data__(data)
        return assembly

    def add_assembly_attributes_timbers(self, assembly, data_type, robot_keys=None, built_keys=None, planned_keys=None):
        
        data_type_list = ['0.Cylinder','1.Box','2.ObjFile','3.Mesh']

        data = assembly.__data__
        beam_keys = assembly.beam_keys
        graph = assembly.graph.__data__
        graph_node = graph["node"]

        for key in beam_keys:
            graph_node[str(key)]['type_id'] = str(key)
            graph_node[str(key)]['type_data'] = data_type_list[data_type]
            graph_node[str(key)]['is_built'] = False
            graph_node[str(key)]['is_planned'] = False
            graph_node[str(key)]['placed_by'] = "human"

        for k in robot_keys:
            if k in beam_keys:
                graph_node[str(k)]['placed_by'] = "robot"

        if built_keys:
            for l in built_keys:
                    if l in beam_keys:
                        graph_node[str(l)]['is_built'] = True

        if planned_keys:
            for m in planned_keys:
                    if m in beam_keys:
                        graph_node[str(m)]['is_planned'] = True

        timber_assembly = TA.assembly.TimberAssembly.__from_data__(data)
        return timber_assembly

    def adjust_item_attributes(self, assembly, key, data_type, built_status, planning_status, placed_by):
        
        data_type_list = ['0.Cylinder','1.Box','2.ObjFile','3.Mesh']
        graph = assembly.graph.__data__
        graph_node = graph["node"]
        
        graph_node[key]['type_data'] = data_type_list[data_type]
        graph_node[key]['is_built'] = built_status
        graph_node[key]['is_planned'] = planning_status
        graph_node[key]['placed_by'] = placed_by

        data = graph_node[key]

        return assembly, data
    
    #Function for Managing Assembly exports
    def export_timberassembly_objs(self, assembly, folder_path, new_folder_name):
        
        assembly_graph = assembly.graph.__data__
        nodes = assembly_graph["node"]
        assembly_beam_keys = assembly.beam_keys
        z_to_y_mapped_frame = Frame(Point(0,0,0), Vector.Xaxis(), Vector.Zaxis())

        #Construct file path with the new folder name
        target_folder_path = os.path.join(folder_path, new_folder_name)
        
        #Make a folder with the folder name if it does not exist
        if not os.path.exists(target_folder_path):
            os.makedirs(target_folder_path)
        
        #Loop through Assembly Keys and export individual elements
        for key in assembly_beam_keys:
            beam = nodes[str(key)]["part"]
            frame = beam.frame
            
            #Get Beam Brep
            beam_brep = beam.geometry
            
            #Make combined mesh from brep faces
            joint_mesh = Mesh()
            meshes = Mesh.CreateFromBrep(beam_brep.native_brep, MeshingParameters.Minimal)
            for m in meshes:
                joint_mesh.Append(m)
            
            #Convert combined mesh to a compas mesh
            compas_mesh = mesh_to_compas(joint_mesh)
                
            #Create Transformation from beam frame to constructed frame
            transformation = Transformation.from_frame_to_frame(frame, z_to_y_mapped_frame)
            
            # Transform mesh from local frame to constructed frame
            mesh_transformed = compas_mesh.transformed(transformation)
            
            # Mesh to obj with file name being KEY and folder being the newly created folder
            if os.path.exists(target_folder_path):
                file_name = str(key)+".obj"
                obj_file_path = os.path.join(target_folder_path, file_name)
                mesh_transformed.to_obj(obj_file_path)
            
            #Raise Exception if the target folder does not exist.
            else:
                raise Exception("File path does not exist {}".format(target_folder_path))

    #Function for Managing Assembly exports
    def export_assembly_objs(self, assembly, folder_path, new_folder_name):
        
        assembly_graph = assembly.graph.__data__
        nodes = assembly_graph["node"]
        z_to_y_mapped_frame = Frame(Point(0,0,0), Vector.Xaxis(), Vector.Zaxis())

        #Construct file path with the new folder name
        target_folder_path = os.path.join(folder_path, new_folder_name)

        #Make a folder with the folder name if it does not exist
        if not os.path.exists(target_folder_path):
            os.makedirs(target_folder_path)

        #Iterate through assembly and perform transformation and export
        for key in nodes:
            
            #extract part
            part = nodes[str(key)]["part"]
            
            frame = part.frame
            
            #Create Transformation from beam frame to world_xy frame and maybe a copy?
            transformation = Transformation.from_frame_to_frame(frame, z_to_y_mapped_frame)
            
            #Transform box from frame to world xy
            part_transformed = part.transformed(transformation)
            
            if nodes[str(key)]["type_data"] in ["0.Cylinder", "1.Box"]:
                part_vert, part_faces = part_transformed.to_vertices_and_faces()
                mesh = Mesh.from_vertices_and_faces(part_vert, part_faces)
            
            elif nodes[str(key)]["type_data"] == "3.Mesh":
                mesh = part_transformed
            
            #TODO: what should I do with .obj, also not sure if this needs to be here or if we just make it from assembly data.

            #Mesh to obj with file name being KEY and folder being the newly created folder
            if os.path.exists(target_folder_path):
                file_name = str(key)+".obj"
                obj_file_path = os.path.join(target_folder_path, file_name)
                mesh.to_obj(obj_file_path)
            
            else:
                raise Exception("File path does not exist {}".format(target_folder_path))