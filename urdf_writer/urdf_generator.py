class URDFGenerator:
    SUPPORTED_JOINT_TYPES = ["revolute", "continuous", "prismatic", "fixed", "floating", "planar"]

    def generate_urdf(self, robot_description):
        name = robot_description["name"]
        materials = robot_description.get("materials", [])
        links = robot_description["links"]
        joints = robot_description.get("joints", [])

        urdf = f'<?xml version="1.0" ?>\n<robot name="{name}">\n'

        # Add materials
        for material in materials:
            urdf += (
                f'  <material name="{material["name"]}" >\n'
                f'    <color rgba="{material["color"]}" />\n'
                f'  </material>\n'
            )

        # Add links
        for link in links:
            urdf += f'  <link name="{link["name"]}">\n'
            if "visual" in link:
                visual = link["visual"]
                urdf += f'    <visual>\n'

                # Handle geometry
                geometry = visual.get("geometry", {})
                urdf += f'      <geometry>\n'
                if geometry["type"] == "box":
                    urdf += f'        <box size="{geometry["size"]}" />\n'
                elif geometry["type"] == "cylinder":
                    urdf += f'        <cylinder length="{geometry["length"]}" radius="{geometry["radius"]}" />\n'
                urdf += f'      </geometry>\n'
                
                # Handle material
                material = visual.get("material", {})
                urdf += f'      <material name="{material["name"]}" />\n'

                urdf += f'    </visual>\n'
            urdf += f'  </link>\n'

        # Add joints
        for joint in joints:
            if joint["type"] not in self.SUPPORTED_JOINT_TYPES:
                raise ValueError(f'Unsupported joint type: \'{joint["type"]}\'. Supported types are: {self.SUPPORTED_JOINT_TYPES}')
            
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