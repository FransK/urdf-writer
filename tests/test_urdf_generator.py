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
            '  <link name="base_link" />\n'
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
            '  <link name="base_link" />\n'
            '  <link name="link1" />\n'
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
            '  <link name="base_link" />\n'
            '  <link name="link1" />\n'
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