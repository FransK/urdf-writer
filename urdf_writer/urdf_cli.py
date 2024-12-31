import argparse
import json
from urdf_generator import URDFGenerator
from robot_prompt import prompt_for_robot_description, save_robot_description_to_file

def generate_urdf_from_file(input_file):
    with open(input_file, "r") as f:
        robot_description = json.load(f)
    
    urdfg = URDFGenerator()
    return urdfg.generate_urdf(robot_description)

def main():
    parser = argparse.ArgumentParser(description="Generate URDF file from robot description JSON")
    parser.add_argument("input_file", help="Path to the robot description JSON file")
    parser.add_argument("--output", help="Path to save the generated URDF file", default=None)
    parser.add_argument("--create", help="Create a new robot description JSON file", action="store_true")

    args = parser.parse_args()

    if args.create:
        # Prompt the user for the robot description and save it to a file
        robot_description = prompt_for_robot_description()
        output_filename = input("Enter the filename to save the robot descrition (e.g., robot_description.json): ")
        save_robot_description_to_file(robot_description, output_filename)
        return

    try:
        urdf_output = generate_urdf_from_file(args.input_file)
        
        if args.output:
            # Save the URDF to the output file
            with open(args.output, "w") as f:
                f.write(urdf_output)
            print(f"URDF file saved to: {args.output}")
        else:
            # Print the URDF to stdout if no output file is specified
            print(urdf_output)
    except Exception as e:
        print(f"Error: {e}")
        exit(1)

if __name__ == "__main__":
    main()