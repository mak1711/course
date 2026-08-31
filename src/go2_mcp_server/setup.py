from setuptools import find_packages, setup

package_name = "go2_mcp_server"

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
    maintainer="kan",
    maintainer_email="mohammad.mk012@gmail.com",
    description="MCP server exposing Go2 Nav2 navigation as LLM-callable tools.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "go2_mcp_server = go2_mcp_server.server:main",
        ],
    },
)
