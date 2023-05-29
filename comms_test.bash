gnome-terminal --title="Chat Log" -e "bash -c 'cd ~/ros2_ws/; source /opt/ros/galactic/setup.bash; source /opt/ros/dashing/setup.bash; . install/setup.bash; ros2 run py_communication chat_log'"

gnome-terminal --title="Manual Input" -e "bash -c 'cd ~/ros2_ws/; source /opt/ros/galactic/setup.bash; source /opt/ros/dashing/setup.bash; . install/setup.bash; ros2 run py_communication tts_test_client'"

gnome-terminal --title="Voice Recognition" -e "bash -c 'cd ~/ros2_ws/; source /opt/ros/galactic/setup.bash; source /opt/ros/dashing/setup.bash; . install/setup.bash; ros2 run py_communication voice_recogniser'"

gnome-terminal --title="Voice Recog. Controller" -e "bash -c 'cd ~/ros2_ws/; source /opt/ros/galactic/setup.bash; source /opt/ros/dashing/setup.bash; . install/setup.bash; ros2 run py_communication voice_recog_control'"

gnome-terminal --title="Text-to-Speech Output" -e "bash -c 'cd ~/ros2_ws/; source /opt/ros/galactic/setup.bash; source /opt/ros/dashing/setup.bash; . install/setup.bash; ros2 run py_communication tts_subscriber'"