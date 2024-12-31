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
            urdf += f'  <joint name="{joint["name"]}" type="{joint["type"]}">\n'
            urdf += f'    <parent link="{joint["parent"]}" />\n'
            urdf += f'    <child link="{joint["child"]}" />\n'
            
            # Handle origin
            origin = joint.get("origin", {})
            xyz = origin.get("xyz", "0 0 0")
            rpy = origin.get("rpy", None)
            if rpy:
                urdf += f'    <origin rpy="{rpy}" xyz="{xyz}" />\n'
            else:
                urdf += f'    <origin xyz="{xyz}" />\n'

            # Handle axis
            if "axis" in joint:
                urdf += f'    <axis xyz="{joint["axis"]}" />\n'

            urdf += f'  </joint>\n'

        urdf += '</robot>'
        return urdf