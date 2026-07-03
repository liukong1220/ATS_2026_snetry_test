from glob import glob
from setuptools import find_packages, setup


package_name = "ats_mujoco_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={"mujoco_lidar": ["scan_mode/*.npy"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/models", glob("models/*.xml") + glob("models/*.png")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lihan Chen",
    maintainer_email="lihanchen2004@163.com",
    description="MuJoCo chassis simulation and sensor bridge for ATS navigation and control validation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ats_mujoco_sim = ats_mujoco_sim.sim_node:main",
            "generate_ats_mujoco_map = ats_mujoco_sim.terrain_assets:main",
            "generate_ats_mujoco_scene = ats_mujoco_sim.scene_assets:main",
        ],
    },
)
