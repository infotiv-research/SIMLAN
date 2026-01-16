#!/usr/bin/env python
# coding: utf-8

# > Install jupyter notebook extension in vscode and run this notebook from vscode. Make sure that the virtual kernel (inside `venv/opt`) is selected instead of the global kernel.
#
# ### Generate extrinsic yaml content.
#
# If we want to add new cameras in the same way as the existing ones we need to define an [intrinsic](./intrinsic/) and [extrinsic](./extrinsic/) yaml file with its camera name as file name, i.e. 500.yaml.
# To view references of how an extrensic yaml file should look, look at a file inside [extrinsic folder](./extrinsic/).
#
#
# To create an intrinsic yaml file, copy an existing one and save the copy with the new camera name. For now, the intrinsic does not change.
# However, to create an extrinsic file you must change the rotation_matrix, t_vec, and position. These vary depending on where you want to place the camera and how it should be angled.
#
# To solve this, use generate_extrinsic_complete(rpy, position) to generate a complete multi-string of the extrinsic yaml content. Easy to copy and paste into the new extrinsic file you want to create. It inputs:
#
# - **position**: The gazebo position, actual position relative to whatever origin you are dealing with.
# - **rpy**: A list containing the roll pitch yaw. This you need to calculate/experiment with yourself to find what angle you want the camera to be.
#
# Note: The angles are calculated using Radians, meaning PI=180 degrees, PI/2=90 degrees.
#
# To get started, run the cells below and modify the position and rpy however you want.

# ### Imports

# In[5]:


import math
from scipy.spatial.transform import Rotation
import numpy as np


# ### Method definitions

# In[6]:


def generate_extrinsic_data(rpy=[0, 0, 0], world_pose=[[3.5], [0], [2.0]]):
    rpy_ordered = [rpy[1], rpy[2], -rpy[0]]  # [rpy[2],rpy[0],rpy[1]]
    rotation_matrix_new = Rotation.from_euler("xyz", rpy_ordered).as_matrix()
    # rotation_matrix_new = rotation_matrix_new @ rot_convert_matrix
    rot_matrix = rotation_matrix_new.flatten()
    C = np.array(world_pose)  # camera center in world coords
    t_vec = -rotation_matrix_new @ C
    t_vec = t_vec.flatten()

    return (
        rot_matrix,
        t_vec,
    )


def generate_new_yaml_file_contents(rotation_matrix=[], t_vec=[], position=[]):

    rotation_matrix_stringified = "".join(
        str(item) + "\n \t" if (index + 1) % 3 == 0 else str(item) + ", "
        for index, item in enumerate(rotation_matrix)
    )

    t_vec_stringified = "".join(
        str(item) + ", " if index != len(t_vec) - 1 else str(item)
        for index, item in enumerate(t_vec)
    )
    position_stringified = "".join(
        str(item) + ", " if index != len(t_vec) - 1 else str(item)
        for index, item in enumerate(position)
    )

    yaml_string_content = """
    %YAML:1.0
    ---
    rot_mat: !!opencv-matrix
        rows: 3
        cols: 3
        dt: d
        data: [{}]
    t_vec: !!opencv-matrix
        rows: 3
        cols: 1
        dt: d
        data: [{}]
    camera_matrix_p: !!opencv-matrix
        rows: 3
        cols: 4
        dt: d
        data: [ -1.1306032963423152e+03, -1.8188048767616505e+02,
            -2.7195034400162358e+03, -5.4816749983310810e+03,
            4.9458696447975876e+02, 2.1668377152678245e+03,
            -1.1124778730630001e+03, 1.8050389422122491e+04,
            4.5228824572093629e-01, -6.1839271619715024e-02,
            -8.8972537744419822e-01, 1.7751578734943330e+01 ]
    position: !!opencv-matrix
        rows: 3
        cols: 1
        dt: d
        data: [{}]
    """.format(
        rotation_matrix_stringified, t_vec_stringified, position_stringified
    )
    return yaml_string_content


def generate_extrinsic_complete(rpy, position):

    rot_matrix, t_vec = generate_extrinsic_data(rpy, position)  # base
    return generate_new_yaml_file_contents(rot_matrix, t_vec, [x[0] for x in position])


# ### SETUP.
#
# Modify these variables.
#
# Position = X, Y, Z
#
# RPY = Roll, Pitch, Yaw

# In[7]:


position = [[3.5], [0], [2.0]]  # Needs to be a matrix with shape (3,1)
rpy = [0, math.pi / 2, -math.pi / 2]  # Is a list [r,p,y]


# ### Run the function

# In[8]:


generated_yaml_content = generate_extrinsic_complete(rpy, position)
print(generated_yaml_content)
