from compas_xr.realtime_database import RealtimeDatabase
from compas_xr.project import ProjectManager
from compas.data import json_load
import os


if __name__ == "__main__":

    # project_name = "robotic_territories_general_inference_testing"
    # project_config_fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\project_config.json"
    # project_config_dict = json_load(project_config_fp)
    # fb_db_config_fp = project_config_dict.get("firebase_config_fp", None)
    # if not fb_db_config_fp:
        # raise ValueError("No 'firebase_config_fp' found in project_config.json")
    # print (fb_db_config_fp)
    # data = json_load(fb_db_config_fp)
    # print (data)
    project_config_fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\fb_config\robotic_territories_fb.json"

    pm = ProjectManager(project_config_fp)
    # pm.application_settings_writer("rt_larget_zones_expiriment_test")
    pm.applicaition_settings_writer_2("rt_performance_ll_zones")
    # pm.applicaition_settings_writer_2("rt_performance_rr_zones")
