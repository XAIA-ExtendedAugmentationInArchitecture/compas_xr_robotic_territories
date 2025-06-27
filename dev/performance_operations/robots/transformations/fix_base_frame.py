from compas_xr.realtime_database import RealtimeDatabase
import json
from compas.data import json_dump



def get_base_observed_base_frame_from_static(transformation_file_path, robot_name):
    """
    Retrieves the base frame of a robot from the RealtimeDatabase.
    
    Parameters:
        robot_name (str): The name of the robot whose base frame is to be retrieved.
    """
    with open(transformation_file_path, "r") as f:
        data = json.load(f)

    if robot_name not in data:
        print(f"RobotManager : [RobotManager] No base frame found for robot '{robot_name}'")
        return None
    
    base_frame = data[robot_name]["observed"]["urdf_base_frame"]["data"]
    return base_frame

def upload_base_observed_base_frame_from_static(robot_transformatoin_file_path, robot_name, fb_config_file_path, project_name="robotic_territories_testing_base_frame"):
    """
    Uploads the base frame of a robot to the RealtimeDatabase.
    
    Parameters:
        robot_name (str): The name of the robot whose base frame is to be uploaded.
    """

    db = RealtimeDatabase(fb_config_file_path)
    base_frame = get_base_observed_base_frame_from_static(robot_transformatoin_file_path, robot_name)
    
    if base_frame is None:
        print(f"RobotManager : [RobotManager] No base frame found for robot '{robot_name}'")
        return
    
    #TODO: Set the whole project information.

    if base_frame is None:
        print(f"RobotManager : [RobotManager] No base frame found for robot '{robot_name}'")
        return

    robot_base_frame_ref_list = [project_name, "robot_base_frame", robot_name] 
    db.upload_data_to_deep_reference(base_frame, reference_list=robot_base_frame_ref_list)

    print(f"RobotManager : [RobotManager] Uploaded base frame for robot '{robot_name}'")


def get_json_file_from_realtime_database(reference, fb_config_file_path="C:\\Users\\jk6372\\Desktop\\00_princeton_projects\\00_robotic_territories\\00_git\\compas_xr_robotic_territories\\dev\\performance_operations\\fb_config\\robotic_territories_fb.json"):
    db = RealtimeDatabase(fb_config_file_path)
    data = db.get_data(reference)
    if data is None:
        print(f"RobotManager : [RobotManager] No data found for reference list '{reference}'")
        return None
    else:
        print(f"RobotManager : [RobotManager] Data retrieved for reference list '{reference} {data}'")
    fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\new_data_structure.json"
    json_dump(fp=fp, data=data, pretty=True)

if __name__ == "__main__":

    firebase_config_fp = "C:\\Users\\jk6372\\Desktop\\00_princeton_projects\\00_robotic_territories\\00_git\\compas_xr_robotic_territories\\dev\\performance_operations\\fb_config\\robotic_territories_fb.json"
    project_name = "robotic_territories_testing_base_frame"
    robot_transformatoin_file_path = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\robots\transformations\robot_transformation_info.json"
    robot_name = "UR20"  # Replace with the actual robot name
    # Example usage
    robot_name = "UR20"  # Replace with the actual robot name
    upload_base_observed_base_frame_from_static(robot_transformatoin_file_path, robot_name, firebase_config_fp, project_name)
    
    get_json_file_from_realtime_database(
        reference=project_name,
        fb_config_file_path=firebase_config_fp
    )

    # To retrieve the base frame
    # base_frame = get_base_observed_base_frame_from_static(robot_transformatoin_file_path, robot_name)
    # print(f"Base frame for {robot_name}: {base_frame}")