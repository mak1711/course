from setuptools import find_packages, setup

package_name = "go2_llm_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "aiohttp", "pywebview"],
    zip_safe=True,
    maintainer="kan",
    maintainer_email="mohammad.mk012@gmail.com",
    description="Natural-language front end for Go2 navigation via a local LLM + MCP tools.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "go2_llm_nav = go2_llm_nav.agent:main",
            "go2_llm_nav_web = go2_llm_nav.web_ui:main",
            "go2_llm_nav_gui = go2_llm_nav.desktop_ui:main",
        ],
    },
)
