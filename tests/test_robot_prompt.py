import json
import os
import tempfile
import unittest
from unittest.mock import patch, mock_open
from urdf_writer.robot_prompt import prompt_for_robot_description, save_robot_description_to_file

class TestRobotPrompt(unittest.TestCase):
    @patch("builtins.input")
    def test_prompt_for_robot_description(self, mock_input):
        # Simulate user input using mock
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
            "0.1 0.2 0.3" # Origin rpy
        ]

        # Call the function to prompt for robot description
        robot_description = prompt_for_robot_description()

        # Check if the robot description is correct
        expected_description = {
            "name": "TestRobot",
            "materials": [
                {
                    "name": "red",
                    "color": "1 0 0 1"
                }
            ],
            "links": [
                {
                    "name": "base_link",
                    "visual": {
                        "geometry": {
                            "type": "cylinder",
                            "radius": "0.5",
                            "length": "2.0"
                        },
                        "material": {
                            "name": "red",
                        },
                        "origin": {
                            "xyz": "1 2 3",
                            "rpy": "0.1 0.2 0.3"
                        }
                    }
                }
            ]
        }

        self.assertEqual(robot_description, expected_description)

    @patch("builtins.input")
    def test_save_robot_description_to_file(self, mock_input):
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
            "0.1 0.2 0.3" # Origin rpy
        ]

        robot_description = prompt_for_robot_description()

        # File to save the description
        with tempfile.NamedTemporaryFile("w", delete=False, suffix='.json') as f:
            temp_output_file_path = f.name

        try:
            save_robot_description_to_file(robot_description, temp_output_file_path)

            with open(temp_output_file_path, 'r') as f:
                saved_content = json.load(f)

            # Verify if the saved content matches the expected description
            self.assertEqual(saved_content, robot_description)
        finally:
            # Remove the temporary files
            os.remove(temp_output_file_path)

    @patch("builtins.input")
    def test_prompt_for_robot_description_missing_input(self, mock_input):
        # Simulate missing input (e.g., geometry radius)
        mock_input.side_effect = [
            "TestRobot",  # Robot name
            "1",          # Number of materials
            "red",        # Material name
            "base_link",  # Link name
            "cylinder",   # Geometry type
            "0.5",        # Cylinder radius
            "2.0",        # Cylinder length
            "red",        # Material name
            "1 2 3",      # Origin xyz
            "0.1 0.2 0.3" # Origin rpy
        ]

        with self.assertRaises(ValueError):
            prompt_for_robot_description()

if __name__ == "__main__":
    unittest.main()
