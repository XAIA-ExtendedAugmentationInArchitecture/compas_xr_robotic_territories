from compas_xr.realtime_database import RealtimeDatabase

if __name__ == "__main__":
    # Initialize the RealtimeDatabase
    fb_config = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\fb_config\robotic_territories_fb.json"
    db = RealtimeDatabase(fb_config)

    # Define the reference name
    reference_name = "test_reference"
    deep_reference = ["test_reference", "child_reference", "deep_child_reference", "deepest_child"]

    # Upload some test data
    test_data = {"key": "value", "number": 42}
    # db.upload_data(test_data, reference_name)
    db.upload_data_to_deep_reference(test_data, deep_reference)
