from setuptools import find_packages, setup

setup(
    name="t2_server_side",
    version="0.1.0",
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    entry_points={
        "console_scripts": [
            "start_server=t2_server_side.main:main",
        ],
    },
)