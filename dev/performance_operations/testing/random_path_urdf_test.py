# import os
# pkg_root = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\robots\urdf\ur_description"
# print(os.path.isfile(os.path.join(pkg_root, "meshes", "ur20", "collision", "base.stl")))
# # → True expected

import os
pkg_root = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\robots\urdf\ur_description"
print("pkg_root exists:", os.path.isdir(pkg_root))
print("base.stl exists:", os.path.isfile(os.path.join(pkg_root, "meshes", "ur20", "collision", "base.stl")))