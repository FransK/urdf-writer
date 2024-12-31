import argparse
import json
from urdf_generator import URDFGenerator

def generate_urdf_from_file(input_file):
    with open(input_file, "r") as f:
        robot_description = json.load(f)
    
    urdfg = URDFGenerator()
    return urdfg.generate_urdf(robot_description)

def main():
    parser = argparse.ArgumentParser(description="Generate URDF file from robot description JSON")
    parser.add_argument("input_file", help="Path to the robot description JSON file")

    args = parser.parse_args()

    try:
        urdf_output = generate_urdf_from_file(args.input_file)
        print(urdf_output)
    except Exception as e:
        print(f"Error: {e}")
        exit(1)

if __name__ == "__main__":
    main()