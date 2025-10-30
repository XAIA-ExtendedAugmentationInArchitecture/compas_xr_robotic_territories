from compas_xr.realtime_database import RealtimeDatabase
from compas.data import json_load
import os


if __name__ == "__main__":

    # project_name = "robotic_territories_general_inference_testing"
    project_name = "rt_user_study_setup_six_targets_many_QR_codes_backup"
    # project_config_fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\project_config.json"
    # project_config_dict = json_load(project_config_fp)
    # fb_db_config_fp = project_config_dict.get("firebase_config_fp", None)
    # if not fb_db_config_fp:
    #     raise ValueError("No 'firebase_config_fp' found in project_config.json")

    # import os, pprint
    # folder = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\fire_base_geo_upload_and_tracking_testing"
    # print("Folder exists:", os.path.isdir(folder))
    # pprint.pprint([repr(x) for x in os.listdir(folder)])
    fb_db_config_fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\fb_config\robotic_territories_fb.json"
    rt_db = RealtimeDatabase(fb_db_config_fp)
    # data_to_upload_fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\fire_base_geo_upload_and_tracking_testing\robotic-territories-default-rtdb-robotic_territories_testing_base_frame-export.json"
    # data_to_upload_fp = r"\\?\C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\fire_base_geo_upload_and_tracking_testing\robotic-territories-default-rtdb-robotic_territories_testing_base_frame-export.json"
    # data_to_upload_fp = r"\\?\C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\fire_base_geo_upload_and_tracking_testing\robotic-territories-default-rtdb-robotic_territories_testing_base_frame_geo_tracking-export.json"
    # data_to_upload_fp = r"C:\Users\jk6372\Downloads\robotic-territories-telemimic_removal_update.json"
    # data_to_upload_fp = r"C:\Users\jk6372\Downloads\robotic-territories-telemimic_general_working_again.json"
    # data_to_upload_fp = r"C:\Users\jk6372\Downloads\rt_inference_upload_again.json"
    # data_to_upload_fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\fire_base_geo_upload_and_tracking_testing\rt_inference_upload_again_larger_zones_expiriment_test.json"
    # data_to_upload_fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\fire_base_geo_upload_and_tracking_testing\rt_inference_upload_again_larger_zones_expiriment_test_actual.json"
    # data_to_upload_fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\firebase_p01_files\rt_larget_zones_equal_size_actual.json"
    # data_to_upload_fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\firebase_p01_files\20251017_rt_user_study_zone_setup_base.json"
    # data_to_upload_fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\firebase_p01_files\robotic_territories_user_study_six_geometries.json"
    data_to_upload_fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\user_study_json_setup\robotic_territories_6_targets_many_QR.json"
    if not os.path.exists(data_to_upload_fp):
        raise FileNotFoundError(f"Data file not found: {data_to_upload_fp}")
    # data = json_load(data_to_upload_fp)

    rt_db.upload_data_from_file(path_local=data_to_upload_fp, refernce_name=project_name)
