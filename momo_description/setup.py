from glob import glob

from setuptools import setup

package_name = "momo_description"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob("launch/*.py")),
        ("share/" + package_name + "/urdf/", glob("urdf/*")),
        ("share/" + package_name + "/rviz/", glob("rviz/*")),
        (
            "share/" + package_name + "/meshes/collision/",
            glob("meshes/collision/*"),
        ),
        ("share/" + package_name + "/meshes/visual/", glob("meshes/visual/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Markus Knitt",
    maintainer_email="markus.knitt@tuhh.de",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
