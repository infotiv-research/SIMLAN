import xml.etree.ElementTree as ET


def extract_joint_limits(xml_file_path):
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    joint_limits = []

    for joint in root.findall('joint'):
        joint_name = joint.get('name')
        joint_type = joint.get('type')

        # Only consider joints that are not fixed and have limits
        limit = joint.find('limit')
        if joint_type != 'fixed' and limit is not None:
            lower = limit.get('lower')
            upper = limit.get('upper')
            joint_limits.append({
                'name': joint_name,
                'lower': lower,
                'upper': upper
            })

    return joint_limits


filename = '/home/ros/src/src/humanoid_robot/model/human-gazebo/humanSubjectWithMeshes/humanSubjectWithMesh_simplified.urdf'
limits = extract_joint_limits(filename)
if True:
    # Print results as a table
    print(len(limits))
    print("\n\njoint\n")
    for joint in limits:
        print(f"'{joint['name']}'",end=", ")
    

    print("\n\nlower\n")
    for joint in limits: 
        print(f"{joint['lower']}",end=", ")
    print("\n\nupper\n")
    for joint in limits:
        print(f"{joint['upper']}",end=", ")