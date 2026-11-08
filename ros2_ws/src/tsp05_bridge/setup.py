from setuptools import find_packages, setup

package_name = "tsp05_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Raoul Bouri",
    maintainer_email="rahulbouri16@gmail.com",
    description="Serial to ROS 2 bridge for the TSP05 Arduino Uno.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["bridge = tsp05_bridge.bridge_node:main"]},
)
