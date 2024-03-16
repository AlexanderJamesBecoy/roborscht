from setuptools import setup

package_name = 'mock_hand_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexander James Becoy',
    maintainer_email='alexanderjames.becoy@outlook.com',
    description='The ROS2 package to mock the hand interface of the humanoid robot hand which controls the abduction/adduction and flexion/extension of each finger.',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_hand_interface_node = mock_hand_interface.mock_hand_interface:main'
        ],
    },
)
