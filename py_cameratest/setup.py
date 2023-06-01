import os
from glob import glob

from setuptools import setup

package_name = 'py_cameratest'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jacob',
    maintainer_email='superlolcopter2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_cameratest.webcam_publisher_function:main',
            "rs_talker = py_cameratest.realsense_publisher_function:main",
            'listener = py_cameratest.webcam_subscriber_function:main',
            'multi_listener = py_cameratest.multicam_subscriber_function:main',
            "detection_tester = py_cameratest.detection_requester:main",
            "focus_tracker = py_cameratest.focus_tracker:main"
        ],
    },
)
