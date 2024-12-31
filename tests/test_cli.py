import json
import os
import subprocess
import tempfile

def test_urdf_cli_generate():
    # Define the robot description for testing
    robot_description = {
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

    # Save the robot description to a temporary file
    with tempfile.NamedTemporaryFile("w", delete=False, suffix='.json') as f:
        json.dump(robot_description, f)
        temp_file_path = f.name

    try:
        # Run the CLI command
        result = subprocess.run(
            ["python3", "urdf_writer/urdf_cli.py", temp_file_path],
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