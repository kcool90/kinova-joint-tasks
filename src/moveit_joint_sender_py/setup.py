from setuptools import find_packages, setup

package_name = 'moveit_joint_sender_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pascal',
    maintainer_email='pascal@todo.todo',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'send_joint_state_goal = moveit_joint_sender_py.send_joint_state_goal:main',
            'trisafe_execute = moveit_joint_sender_py.trisafe_execute:main',
        ],
    },
)
