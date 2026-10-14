from glob import glob
from setuptools import setup

package_name = "originbot_delivery"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.json")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/static", glob("static/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OriginBot",
    maintainer_email="support@ps-micro.com",
    description="ArUco localization and web-controlled delivery mission services for OriginBot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "aruco_localization = originbot_delivery.aruco_localization:main",
            "delivery_server = originbot_delivery.delivery_server:main",
            "map_viewer = originbot_delivery.map_viewer:main",
            "reregister_markers = originbot_delivery.reregister_markers:main",
            "route_follower = originbot_delivery.route_follower:main",
            "web_server = originbot_delivery.web_server:main",
        ],
    },
)
