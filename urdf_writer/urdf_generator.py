class URDFGenerator:
    def generate_urdf(self, robot_description):
        name = robot_description["name"]
        links = robot_description["links"]
        joints = robot_description["joints"]

        urdf = f'<?xml version="1.0" ?>\n<robot name="{name}">\n'

        # Add links
        for link in links:
            urdf += f'  <link name="{link["name"]}" />\n'

        # Add joints
        for joint in joints:
            urdf += (
                f'  <joint name="{joint["name"]}" type="{joint["type"]}">\n'
                f'    <parent link="{joint["parent"]}" />\n'
                f'    <child link="{joint["child"]}" />\n'
                f'    <origin rpy="{joint["origin"]["rpy"]}" xyz="{joint["origin"]["xyz"]}" />\n'
                f'    <axis xyz="{joint["axis"]}" />\n'
                f'  </joint>\n'
            )

        urdf += '</robot>'
        return urdf