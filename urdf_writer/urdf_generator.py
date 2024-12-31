class URDFGenerator:
    def generate_urdf(self, robot_description):
        name = robot_description["name"]
        links = robot_description["links"]
        urdf = f'<?xml version="1.0" ?>\n<robot name="{name}">\n'
        for link in links:
            urdf += f'  <link name="{link["name"]}" />\n'
        urdf += '</robot>'
        return urdf