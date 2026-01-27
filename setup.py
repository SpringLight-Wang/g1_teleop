from setuptools import setup, find_packages

setup(
    name="g1_teleop",
    version="0.1.0",
    description="Unitree G1 MuJoCo Teleoperation with Joy-Con Control",
    author="Chunguang Wang", 
    author_email="wangchunguang408@163.com",  
    url="https://github.com/SpringLight-Wang/g1_teleop.git", 

    packages=find_packages(),

    py_modules=[
        "g1_dual_mujoco",
        "g1_dual_real",       
        "joycon_driver_dual"
    ],

    install_requires=[

        "numpy>=1.17.4",
        "scipy",
        "matplotlib",
        "pyglm",
        "hidapi",
        "hid==1.0.4", 
        "mujoco>=3.0.0",
        "ansitable",
        "progress",
        "typing_extensions",
        "ipykernel",
    ],

    include_package_data=True,

    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Operating System :: POSIX :: Linux",
        "Topic :: Scientific/Engineering :: Robotics",
    ],
    python_requires='>=3.8',
)
