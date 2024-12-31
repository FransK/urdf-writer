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