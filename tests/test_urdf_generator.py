import unittest
from urdf_writer.urdf_generator import URDFGenerator

class TestURDFGenerator(unittest.TestCase):
    def test_generate_urdf(self):
        urdfg = URDFGenerator()
        robot_description = {
            "name": "test_robot",
            "links": [{"name": "base_link"}],
            "joints": [],                
        }
        expected_output = (
            '<?xml version="1.0" ?>\n'
            '<robot name="test_robot">\n'
            '  <link name="base_link">\n'
            '  </link>\n'
            '</robot>'
        )
        self.assertEqual(urdfg.generate_urdf(robot_description), expected_output)

    def test_generate_urdf_with_joints(self):
        urdfg = URDFGenerator()
        robot_description = {
            "name": "test_robot",
            "links": [{"name": "base_link"}, {"name": "link1"}],
            "joints": [{
                "name": "joint1",
                "type": "revolute",
                "parent": "base_link",
                "child": "link1",
                "origin": {"rpy": "0 0 0", "xyz": "0 0 0"},
                "axis": "0 0 1",
            }]
        }
        expected_output = (
            '<?xml version="1.0" ?>\n'
            '<robot name="test_robot">\n'
            '  <link name="base_link">\n'
            '  </link>\n'
            '  <link name="link1">\n'
            '  </link>\n'
            '  <joint name="joint1" type="revolute">\n'
            '    <parent link="base_link" />\n'
            '    <child link="link1" />\n'
            '    <origin rpy="0 0 0" xyz="0 0 0" />\n'
            '    <axis xyz="0 0 1" />\n'
            '  </joint>\n'
            '</robot>'
        )
        self.assertEqual(urdfg.generate_urdf(robot_description), expected_output)

    def test_generate_urdf_joint_without_axis(self):
        urdfg = URDFGenerator()
        robot_description = {
            "name": "test_robot",
            "links": [{"name": "base_link"}, {"name": "link1"}],
            "joints": [{
                "name": "joint1",
                "type": "fixed",
                "parent": "base_link",
                "child": "link1",
                "origin": {"xyz": "1 0 0"}, # No "rpy" and no "axis"
            }]
        }
        expected_output = (
            '<?xml version="1.0" ?>\n'
            '<robot name="test_robot">\n'
            '  <link name="base_link">\n'
            '  </link>\n'
            '  <link name="link1">\n'
            '  </link>\n'
            '  <joint name="joint1" type="fixed">\n'
            '    <parent link="base_link" />\n'
            '    <child link="link1" />\n'
            '    <origin xyz="1 0 0" />\n' # No "rpy" and no "axis" is fine
            '  </joint>\n'
            '</robot>'
        )
        self.assertEqual(urdfg.generate_urdf(robot_description), expected_output)

    def test_generate_urdf_joint_with_unsupported_type(self):
        urdfg = URDFGenerator()
        robot_description = {
            "name": "test_robot",
            "links": [{"name": "base_link"}, {"name": "link1"}],
            "joints": [{
                "name": "joint1",
                "type": "unsupported",
                "parent": "base_link",
                "child": "link1",
                "origin": {"rpy": "0 0 0", "xyz": "0 0 0"},
                "axis": "0 0 1",
            }]
        }
        with self.assertRaises(ValueError) as context:
            urdfg.generate_urdf(robot_description)
        
        self.assertEqual(
            str(context.exception),
            f'Unsupported joint type: \'unsupported\'. Supported types are: {URDFGenerator.SUPPORTED_JOINT_TYPES}'
        )

    def test_generate_urdf_with_visuals(self):
        urdfg = URDFGenerator()
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
                            "type": "box",
                            "size": "1 1 1",
                        },
                        "material": {
                            "name": "blue",
                        },
                    }
                }
            ],
        }
        expected_output = (
            '<?xml version="1.0" ?>\n'
            '<robot name="test_robot">\n'
            '  <material name="blue" >\n'
            '    <color rgba="0 0 1 1" />\n'
            '  </material>\n'
            '  <link name="base_link">\n'
            '    <visual>\n'
            '      <geometry>\n'
            '        <box size="1 1 1" />\n'
            '      </geometry>\n'
            '      <material name="blue" />\n'
            '    </visual>\n'
            '  </link>\n'
            '</robot>'
        )
        self.assertEqual(urdfg.generate_urdf(robot_description), expected_output)

    def test_generate_urdf_with_cylinder_geometry(self):
        urdfg = URDFGenerator()
        robot_description = {
            "name": "test_robot",
            "materials": [
                {
                    "name": "red",
                    "color": "1 0 0 1",
                }
            ],
            "links": [
                {
                    "name": "base_link",
                    "visual": {
                        "geometry": {
                            "type": "cylinder",
                            "radius": "0.5",
                            "length": "2.0",
                        },
                        "material": {
                            "name": "red",
                        },
                    }
                }
            ],
        }
        expected_output = (
            '<?xml version="1.0" ?>\n'
            '<robot name="test_robot">\n'
            '  <material name="red" >\n'
            '    <color rgba="1 0 0 1" />\n'
            '  </material>\n'
            '  <link name="base_link">\n'
            '    <visual>\n'
            '      <geometry>\n'
            '        <cylinder length="2.0" radius="0.5" />\n'
            '      </geometry>\n'
            '      <material name="red" />\n'
            '    </visual>\n'
            '  </link>\n'
            '</robot>'
        )
        self.assertEqual(urdfg.generate_urdf(robot_description), expected_output)

    def test_generate_urdf_with_visual_and_origin(self):
        urdfg = URDFGenerator()
        robot_description = {
            "name": "test_robot",
            "materials": [
                {
                    "name": "red",
                    "color": "1 0 0 1",
                }
            ],
            "links": [
                {
                    "name": "base_link",
                    "visual": {
                        "geometry": {
                            "type": "cylinder",
                            "radius": "0.5",
                            "length": "2.0",
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
        expected_output = (
            '<?xml version="1.0" ?>\n'
            '<robot name="test_robot">\n'
            '  <material name="red" >\n'
            '    <color rgba="1 0 0 1" />\n'
            '  </material>\n'
            '  <link name="base_link">\n'
            '    <visual>\n'
            '      <geometry>\n'
            '        <cylinder length="2.0" radius="0.5" />\n'
            '      </geometry>\n'
            '      <origin rpy="0.1 0.2 0.3" xyz="1 2 3" />\n'
            '      <material name="red" />\n'
            '    </visual>\n'
            '  </link>\n'
            '</robot>'
        )
        self.assertEqual(urdfg.generate_urdf(robot_description), expected_output)