import json

class JsonAnalyzer:
    def __init__(self, file_path, save_path=None):
        self.file_path = file_path
        self.save_path = save_path

    def analyze_json(self):
        # Load the JSON file
        if not self.file_path.endswith('.json'):
            raise ValueError("The file must be a JSON file.")
        try:
            with open(self.file_path, 'r') as f:
                data = json.load(f)
        except FileNotFoundError:
            raise FileNotFoundError(f"The file {self.file_path} was not found.")

        def transform(obj, key=None):
            # When we hit the "meshes" field, count all the sub-points in their "data" arrays
            if key == "meshes" and isinstance(obj, list):
                total_points = 0
                for mesh in obj:
                    if isinstance(mesh, dict) and isinstance(mesh.get("data"), list):
                        total_points += len(mesh["data"])
                return total_points

            # Recurse into dicts
            if isinstance(obj, dict):
                return {k: transform(v, key=k) for k, v in obj.items()}

            # Recurse into lists
            if isinstance(obj, list):
                return [transform(item) for item in obj]

            # Otherwise replace with the primitive's type name
            return type(obj).__name__

        structured_types = transform(data)

        # Save the transformed structure
        out_path = self.save_path or self.file_path.replace('.json', '_analyzed.json')
        with open(out_path, 'w') as out_f:
            json.dump(structured_types, out_f, indent=4)

        return structured_types, out_path


# Example usage
if __name__ == "__main__":
    fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\random\robot_data.json"
    sp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\random\analysis\robot_data_reviewed_mesh_count.json"
    analyzer = JsonAnalyzer(fp, sp)
    try:
        result, path = analyzer.analyze_json()
        print(f"Analysis complete. Results saved to {path}.")
        print(json.dumps(result, indent=4))
    except Exception as e:
        print(f"An error occurred: {e}")


# fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\random\robot_vs_geo.json"
# sp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\random\analysis\robot_vs_geo_reviewed.json"
