from setuptools import setup

package_name = "mk0_event_server"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "playsound"],
    zip_safe=True,
    maintainer="cherry",
    maintainer_email="kadyn_martinez@hotmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mk0_event_server = mk0_event_server.event:main",
            "controller_receiver = mk0_event_server.mk0_controller_receiver:main",
        ],
    },
)
