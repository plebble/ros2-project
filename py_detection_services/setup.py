from setuptools import setup

package_name = 'py_detection_services'

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
    maintainer='jacob',
    maintainer_email='superlolcopter2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "face_detector = py_detection_services.facial_detection_service_torch:main",
            "pose_detector = py_detection_services.head_pose_detection_service:main",
            "multi_detector = py_detection_services.multi_detection_service:main",
            "test_client = py_detection_services.test_client:main",
            "test_client_pose = py_detection_services.test_client_pose:main"
        ],
    },
)
