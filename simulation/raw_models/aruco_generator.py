# reference: https://pyimagesearch.com/2020/12/14/generating-aruco-markers-with-opencv-and-python/
# https://github.com/nlamprian/grobot/tree/master
import numpy as np
import cv2
import cv2.aruco as aruco

import shutil
import os

"""
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}
"""

aruco_type = aruco.DICT_5X5_1000
aruco_sidepixel = 100  # size of the image in pixels
aruco_borderbits = 10  # width of the marker border.
aruco_max = 100
texture_path = "../simlan_gazebo_environment/models/aruco/materials/textures"
material_script_path = "../simlan_gazebo_environment/models/aruco/materials/scripts/"

shutil.rmtree(texture_path, ignore_errors=True)
os.mkdir(texture_path)

shutil.rmtree(material_script_path, ignore_errors=True)
os.mkdir(material_script_path)

# Select type of aruco marker (size)
aruco_dict = aruco.Dictionary_get(aruco_type)

for indx in range(0, aruco_max):
    img = aruco.drawMarker(aruco_dict, indx, aruco_sidepixel, aruco_borderbits)
    png_file = str(indx) + ".png"
    filename = texture_path + "/" + png_file
    print(filename)
    cv2.imwrite(filename, img)
    file_object = open(material_script_path + "aruco_" + str(indx) + ".material", "w")
    file_object.write(
        """
material aruco_"""
        + str(indx)
        + """
{
  technique
  {
    pass
    {
      texture_unit
      {
        texture """
        + png_file
        + """
        filtering anisotropic
        max_anisotropy 16
      }
    }
  }
}
"""
    )
    file_object.close()
