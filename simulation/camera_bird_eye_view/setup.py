from setuptools import find_packages, setup

package_name = "camera_bird_eye_view"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hamid Ebadi, Pär Aronsson",
    maintainer_email="hamid.ebadi@gmail.com, par.aronsson@infotiv.se",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_bird_eye_view_service = camera_bird_eye_view.camera_bird_eye_view_service:main",
            "camera_bird_eye_view_publisher = camera_bird_eye_view.camera_bird_eye_view_publisher:main",
        ],
    },
)
