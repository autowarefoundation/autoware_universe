from setuptools import setup

package_name = "autoware_launch_ros_extension"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Takagi, Isamu",
    maintainer_email="isamu.takagi@tier4.jp",
    license="Apache License 2.0",
    entry_points={
        "launch.frontend.launch_extension": [
            "autoware_launch_ros_extension = autoware_launch_ros_extension",
        ],
    },
)
