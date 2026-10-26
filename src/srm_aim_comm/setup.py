from setuptools import find_packages, setup

package_name = 'srm_aim_comm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='gyqros',
    maintainer_email='3418558797@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'yolo': ['ultralytics>=8.3.0,<9.0', 'numpy>=1.23,<2'],
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_comm_node = srm_aim_comm.serial_comm_node:main',
            'yolo_pose_node = srm_aim_comm.yolo_pose_node:main',
            'yolo_visualizer_node = srm_aim_comm.yolo_visualizer_node:main',
        ],
    },
)
