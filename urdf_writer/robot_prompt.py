import json

def prompt_for_robot_description():
    # Prompt for the robot's name
    robot_name = input("Enter the robot name: ")

    # Initialize the robot description dictionary
    robot_description = {"name": robot_name, "materials": [], "links": []}

    # Ask for the number of materials
    num_materials = int(input("How many materials does the robot have? "))

    for i in range(num_materials):
        material_name = input(f"Enter the name of material {i + 1}: ")
        color = input(f"Enter the color for {material_name} (format: R G B A): ")

        # Initialize the material dictionary
        material = {"name": material_name, "color": color}

        # Add the material to the robot description
        robot_description["materials"].append(material)

    # Ask for the number of links
    num_links = int(input("How many links does the robot have? "))

    for i in range(num_links):
        link_name = input(f"Enter the name of link {i + 1}: ")

        # Initialize the link dictionary
        link = {"name": link_name, "visual": {}}

        # Ask for the visual properties
        geometry_type = input(f"Enter the geometry type for {link_name} (cylinder, box):")

        geometry = {}
        if geometry_type == "cylinder":
            geometry["type"] = "cylinder"
            radius = input("Enter the radius of the cylinder: ")
            if not radius:
                raise ValueError("Radius is required for a cylinder.")
            geometry["radius"] = radius
            length = input("Enter the length of the cylinder: ")
            if not length:
                raise ValueError("Length is required for a cylinder.")
            geometry["length"] = length
        elif geometry_type == "box":
            geometry["type"] = "box"
            size = input("Enter the size of the box (format: x y z): ")
            if not size:
                raise ValueError("Size is required for a box.")
            geometry["size"] = size
        else:
            print("Unsupported geometry type. Skipping link.")
            continue
        
        # Add the geometry to the link
        link["visual"]["geometry"] = geometry

        # Ask for material properties
        material_name = input(f"Enter the material name for {link_name}: ")
        link["visual"]["material"] = {"name": material_name}

        # Ask for origin properties
        origin_xyz = input(f"Enter the origin XYZ for {link_name} (format: x y z): ")
        origin_rpy = input(f"Enter the origin rpy (roll, pitch, yaw) for {link_name} (format: r p y): ")
        link["visual"]["origin"] = {"xyz": origin_xyz, "rpy": origin_rpy}

        # Add the link to the robot description
        robot_description["links"].append(link)

    # Return the generated robot description
    return robot_description

def save_robot_description_to_file(robot_description, filename):
    with open(filename, "w") as f:
        json.dump(robot_description, f, indent=4)
    print(f"Robot description saved to: {filename}")