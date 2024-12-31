import json
import os
import subprocess
import tempfile
import unittest

class TestCLI(unittest.TestCase):
    def setUp(self):
        # Define the robot description for testing
        self.robot_description = {
            "name": "test_robot",
            "materials": [
                {
                    "name": "blue",
                    "color": "0 0 1 1",
                }
            ],
            "links": [
                {
                    "name": "base_link",
                    "visual": {
                        "geometry": {
                            "type": "cylinder",
                            "length": "2.0",
                            "radius": "0.5",
                        },
                        "material": {
                            "name": "blue",
                        },
                        "origin": {
                            "xyz": "1 2 3",
                            "rpy": "0.1 0.2 0.3",
                        }
                    }
                }
            ],
        }

    def test_urdf_cli_generate(self):
        # Save the robot description to a temporary file
        with tempfile.NamedTemporaryFile("w", delete=False, suffix='.json') as f:
            json.dump(self.robot_description, f)
            temp_file_path = f.name

        try:
            # Run the CLI command
            result = subprocess.run(
                ["python3", "-m", "urdf_writer.cli", temp_file_path],
                capture_output=True,
                text=True,
            )

            # Read the generated URDF
            expected_output = (
                '<?xml version="1.0" ?>\n'
                '<robot name="test_robot">\n'
                '  <material name="blue" >\n'
                '    <color rgba="0 0 1 1" />\n'
                '  </material>\n'
                '  <link name="base_link">\n'
                '    <visual>\n'
                '      <geometry>\n'
                '        <cylinder length="2.0" radius="0.5" />\n'
                '      </geometry>\n'
                '      <origin rpy="0.1 0.2 0.3" xyz="1 2 3" />\n'
                '      <material name="blue" />\n'
                '    </visual>\n'
                '  </link>\n'
                '</robot>'
            )

            # Check the result
            assert result.returncode == 0
            assert result.stdout.strip() == expected_output
        finally:
            # Remove the temporary file
            os.remove(temp_file_path)

    def test_urdf_cli_generate_with_output_file(self):
        # Save the robot description to a temporary file
        with tempfile.NamedTemporaryFile("w", delete=False, suffix='.json') as f:
            json.dump(self.robot_description, f)
            temp_file_path = f.name

        # Save the URDF to a temporary file
        with tempfile.NamedTemporaryFile("w", delete=False, suffix='.urdf') as f:
            temp_output_file_path = f.name

        try:
            # Run the CLI command
            result = subprocess.run(
                ["python3", "-m", "urdf_writer.cli", temp_file_path, "--output", temp_output_file_path],
                capture_output=True,
                text=True,
            )

            # Check the result
            assert result.returncode == 0

            # Read the generated URDF
            with open(temp_output_file_path, "r") as f:
                urdf_output = f.read()

            expected_output = (
                '<?xml version="1.0" ?>\n'
                '<robot name="test_robot">\n'
                '  <material name="blue" >\n'
                '    <color rgba="0 0 1 1" />\n'
                '  </material>\n'
                '  <link name="base_link">\n'
                '    <visual>\n'
                '      <geometry>\n'
                '        <cylinder length="2.0" radius="0.5" />\n'
                '      </geometry>\n'
                '      <origin rpy="0.1 0.2 0.3" xyz="1 2 3" />\n'
                '      <material name="blue" />\n'
                '    </visual>\n'
                '  </link>\n'
                '</robot>'
            )

            assert urdf_output.strip() == expected_output
        finally:
            # Remove the temporary files
            os.remove(temp_file_path)
            os.remove(temp_output_file_path)
