from setuptools import find_packages, setup

package_name = "go2_semantic_map"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/places.yaml", "config/places_junior.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kan",
    maintainer_email="mohammad.mk012@gmail.com",
    description="Manual named-place -> pose lookup for the Go2 semantic navigation layer.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
