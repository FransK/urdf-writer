import tempfile
import os
import json
import subprocess
from urdf_writer.robot_prompt import prompt_for_robot_description
from unittest.mock import patch
import unittest

class TestURDFCLIIntegration(unittest.TestCase):

    @patch("builtins.input")
    def test_prompt_to_urdf_cli(self, mock_input):
        # Simulate user input
        mock_input.side_effect = [
            "TestRobot",  # Robot name
            "1",          # Number of materials
            "red",        # Material name
            "1 0 0 1",    # Color (RGBA)
            "1",          # Number of links
            "base_link",  # Link name
            "cylinder",   # Geometry type
            "0.5",        # Cylinder radius
            "2.0",        # Cylinder length
            "red",        # Material name
            "1 2 3",      # Origin xyz
            "0.1 0.2 0.3",# Origin rpy
            "1",          # Number of joints
            "joint1",     # Joint name
            "revolute",   # Joint type
            "base_link",  # Parent link
            "child_link", # Child link
            "1 2 3",      # Origin xyz
        ]

        # Generate the robot description using the prompt
        robot_description = prompt_for_robot_description()

        # Create a temporary JSON file to save the robot description
        with tempfile.NamedTemporaryFile("w", delete=False, suffix=".json") as json_file:
            json_file_path = json_file.name
            json.dump(robot_description, json_file)

        # Create a temporary file for the URDF output
        with tempfile.NamedTemporaryFile("w", delete=False, suffix=".urdf") as urdf_file:
            urdf_file_path = urdf_file.name

        try:
            # Run the URDF CLI with the generated JSON as input and specify the output file
            result = subprocess.run(
                ["python3", "urdf_writer/urdf_cli.py", json_file_path, "--output", urdf_file_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            # Verify the CLI executed successfully
            self.assertEqual(result.returncode, 0, f"CLI failed: {result.stderr}")

            # Verify the URDF file is written and not empty
            with open(urdf_file_path, "r") as urdf_file:
                urdf_content = urdf_file.read()
                self.assertIn(f'<robot name="TestRobot">', urdf_content)
                self.assertIn(f'<link name="base_link">', urdf_content)
                self.assertIn(f'<geometry>', urdf_content)
                self.assertIn(f'<cylinder length="2.0" radius="0.5" />', urdf_content)
                self.assertIn(f'<material name="red" />', urdf_content)
                self.assertIn(f'<origin rpy="0.1 0.2 0.3" xyz="1 2 3" />', urdf_content)
                self.assertIn(f'<joint name="joint1" type="revolute">', urdf_content)
                self.assertIn(f'<parent link="base_link" />', urdf_content)

        finally:
            # Clean up temporary files
            if os.path.exists(json_file_path):
                os.remove(json_file_path)
            if os.path.exists(urdf_file_path):
                os.remove(urdf_file_path)

if __name__ == "__main__":
    unittest.main()
