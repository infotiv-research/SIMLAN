NUM_JOINTS = 47 # number of items in motion = NUM_JOINTS
NUM_MARKERS = 33 # number of items in pose = NUM_MARKERS * 3
# https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker
MARKER_L_HIP =24 # index for left hip 
MARKER_R_HIP =23 # index for right hip
MARKER_R_KNEE=25 # index for right knee


pose_names = [f'{i}_{axis}' for i in range(0, NUM_MARKERS) for axis in ['x', 'y', 'z']]
movable_joint_names = [
     'jL5S1_rotx', 'jL5S1_roty', 'jT9T8_rotx', 'jT9T8_roty', 'jT9T8_rotz', 'jC7RightShoulder_rotx', 'jC7LeftShoulder_rotx', 'jT1C7_rotx', 'jT1C7_roty', 'jT1C7_rotz', 'jC1Head_rotx', 'jC1Head_roty', 'jRightShoulder_rotx', 'jRightShoulder_roty', 'jRightShoulder_rotz', 'jRightElbow_roty', 'jRightElbow_rotz', 'jRightWrist_rotx', 'jRightWrist_rotz', 'jLeftShoulder_rotx', 'jLeftShoulder_roty', 'jLeftShoulder_rotz', 'jLeftElbow_roty', 'jLeftElbow_rotz', 'jLeftWrist_rotx', 'jLeftWrist_rotz', 'jRightHip_rotx', 'jRightHip_roty', 'jRightHip_rotz', 'jRightKnee_roty', 'jRightKnee_rotz', 'jRightAnkle_rotx', 'jRightAnkle_roty', 'jRightAnkle_rotz', 'jRightBallFoot_roty', 'jLeftHip_rotx', 'jLeftHip_roty', 'jLeftHip_rotz', 'jLeftKnee_roty', 'jLeftKnee_rotz', 'jLeftAnkle_rotx', 'jLeftAnkle_roty', 'jLeftAnkle_rotz', 'jLeftBallFoot_roty'
     ,
     'link1_link2_joint', 'link2_link3_joint', 'link3_link4_joint'
  ]
movable_joint_upper = [
    -0.610865, -0.523599, -0.349066, -0.261799, -0.610865, -0.785398, -0.0872665, -0.610865, -0.959931, -1.22173, -0.610865, -0.436332, -2.35619, -1.5708, -0.785398, -1.5708, 0.0, -0.872665, -0.523599, -1.5708, -1.5708, -3.14159, -1.5708, -2.53073, -1.0472, -0.349066, -0.785398, -2.0944, -0.785398, 0.0, -0.698132, -0.610865, -0.523599, -0.436332, -0.174533, -0.523599, -2.0944, -0.785398, 0.0, -0.523599, -0.785398, -0.523599, -0.872665, -0.174533
    ,
    1.5707, 1.5707, 1.5707

]
movable_joint_lower = [
    0.610865, 1.309, 0.349066, 0.698132, 0.610865, 0.0872665, 0.785398, 0.610865, 1.5708, 1.22173, 0.610865, 0.174533, 1.5708, 1.5708, 3.14159, 1.48353, 2.53073, 1.0472, 0.349066, 2.35619, 1.5708, 0.785398, 1.48353, 0.0, 0.872665, 0.523599, 0.523599, 0.261799, 0.785398, 2.35619, 0.523599, 0.785398, 0.872665, 0.872665, 1.5708, 0.785398, 0.261799, 0.785398, 2.35619, 0.698132, 0.610865, 0.872665, 0.436332, 1.5708
    ,
    -1.5707, -1.5707, -1.5707
]


assert NUM_JOINTS == len(movable_joint_names), "Inconsistent number of joints in the list"
assert NUM_JOINTS == len(movable_joint_upper), "Inconsistent number of joints in the list"
assert NUM_JOINTS == len(movable_joint_lower), "Inconsistent number of joints in the list"