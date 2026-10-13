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
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'comm_node = srm_aim_comm.comm_node:main',
        'vision_sim_node = srm_aim_comm.vision_sim_node:main',
        'aim_solver_node = srm_aim_comm.aim_solver_node:main',
        'aim_control_node = srm_aim_comm.aim_control_node:main',
        'camera_node = srm_aim_comm.camera_node:main',
        'gimbal_sim_node = srm_aim_comm.gimbal_sim_node:main',
        'serial_comm_node = srm_aim_comm.serial_comm_node:main',
        'armor_detector_node = srm_aim_comm.armor_detector_node:main',
        ],
    },
)
