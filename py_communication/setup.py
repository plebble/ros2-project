from setuptools import setup

package_name = 'py_communication'

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
            'tts_subscriber = py_communication.tts_subscriber:main',
            'tts_test_client = py_communication.tts_test_client:main',
            "voice_recogniser = py_communication.voice_recog_publisher:main",
            "voice_recog_control = py_communication.voice_recog_control:main",
            "chat_log = py_communication.chat_log_display:main",
            "davinci_responder = py_communication.davinci_responder:main"
        ],
    },
)
