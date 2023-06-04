gnome-terminal --title="Multi Listener (Display)" -e "bash -c 'cd ~/ros2_ws/; source /opt/ros/galactic/setup.bash; source /opt/ros/dashing/setup.bash; . install/setup.bash; ros2 run py_cameratest multi_listener'"

gnome-terminal --title="Webcam Publisher Node" -e "bash -c 'cd ~/ros2_ws/; source /opt/ros/galactic/setup.bash; source /opt/ros/dashing/setup.bash; . install/setup.bash; ros2 run py_cameratest talker'"

gnome-terminal --title="Face & Pose Detector" -e "bash -c 'cd ~/ros2_ws/; source /opt/ros/galactic/setup.bash; source /opt/ros/dashing/setup.bash; . install/setup.bash; ros2 run py_detection_services multi_detector'"

sleep 5

gnome-terminal --title="Focus Tracker" -e "bash -c 'cd ~/ros2_ws/; source /opt/ros/galactic/setup.bash; source /opt/ros/dashing/setup.bash; . install/setup.bash; ros2 run py_cameratest focus_tracker'"