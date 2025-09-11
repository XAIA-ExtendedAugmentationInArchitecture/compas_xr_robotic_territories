from compas_xr.realtime_database import RealtimeDatabase
from compas.data import json_load
import os
import time


if __name__ == "__main__":

    # project_name = "robotic_territories_general_inference_testing"
    project_name = "rt_larget_zones_expiriment_test"
    project_config_fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\project_config.json"
    project_config_dict = json_load(project_config_fp)
    fb_db_config_fp = project_config_dict.get("firebase_config_fp", None)
    if not fb_db_config_fp:
        raise ValueError("No 'firebase_config_fp' found in project_config.json")

    import os, pprint
    folder = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\fire_base_geo_upload_and_tracking_testing"
    print("Folder exists:", os.path.isdir(folder))
    pprint.pprint([repr(x) for x in os.listdir(folder)])

    rt_db = RealtimeDatabase(fb_db_config_fp)
    # data_to_upload_fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\fire_base_geo_upload_and_tracking_testing\robotic-territories-default-rtdb-robotic_territories_testing_base_frame-export.json"
    # data_to_upload_fp = r"\\?\C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\fire_base_geo_upload_and_tracking_testing\robotic-territories-default-rtdb-robotic_territories_testing_base_frame-export.json"
    # data_to_upload_fp = r"\\?\C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\fire_base_geo_upload_and_tracking_testing\robotic-territories-default-rtdb-robotic_territories_testing_base_frame_geo_tracking-export.json"
    # data_to_upload_fp = r"C:\Users\jk6372\Downloads\robotic-territories-telemimic_removal_update.json"
    # data_to_upload_fp = r"C:\Users\jk6372\Downloads\robotic-territories-telemimic_general_working_again.json"
    # data_to_upload_fp = r"C:\Users\jk6372\Downloads\rt_inference_upload_again.json"
    # data_to_upload_fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\fire_base_geo_upload_and_tracking_testing\rt_inference_upload_again_larger_zones_expiriment_test.json"
    data_to_upload_fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\fire_base_geo_upload_and_tracking_testing\rt_inference_upload_again_larger_zones_expiriment_test_actual.json"

    if not os.path.exists(data_to_upload_fp):
        raise FileNotFoundError(f"Data file not found: {data_to_upload_fp}")
    # data = json_load(data_to_upload_fp)

    # rt_db.upload_data_from_file(path_local=data_to_upload_fp, refernce_name=project_name)

    start = time.time()
    data = rt_db.get_data(reference_name=project_name)
    end = time.time()
    print(f"type of data retrieved: {type(data)}")
    print(f"Retrieved data keys: {list(data.keys())}")
    print(f"Data retrieval took {end - start:.4f} seconds.")
    # TODO: Example : Data retrieval took 0.1171 seconds.


