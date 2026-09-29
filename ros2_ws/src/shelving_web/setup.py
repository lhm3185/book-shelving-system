from glob import glob
import os

from setuptools import find_packages, setup


package_name = "shelving_web"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            os.path.join("share", package_name),
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "static"),
            glob("shelving_web/static/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hymi",
    maintainer_email="lhm6582@gmail.com",
    description="ROS 2 web gateway and operator dashboard.",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            (
                "web_gateway_node = "
                "shelving_web.web_gateway_node:main"
            ),
        ],
    },
)