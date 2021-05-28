# wqdrone

Thesis project to develop a water quality sensing platform
The GitHub contains the arduino codes, ros dev ws, python post processing and the thesis document.

Under Arduino one can find the Teensy, main water quality arduino files, LoRa communication code, and the additional power sensing for the in-box monitoring .ino files.

The three dashboard folders contain the original dashboard. The actually used dashboard_v2. And the dashboard_timeseries for gif-creation.

Plotting contains the complete interpolation pipeline, from loading to final interpolation,as well as the calibration plots. This is used for creating all the plots in the thesis.Furthermore, the subfolder data contains all the final .tiff files.

python_test_serial is python scripts where the serial communications for the teensy and kogger sonar were tested. They are kept as examples for later.

ros_dev_ws is the main dev_ws as required for ROS2, the important files in here are the launch files, and the files under the src folder. In src, the data reading and publsihing scripts can be found. teensy, powermonitoring, lora and the kogger sonar scripts.

rosbags are acquired rosbags, folder was used to share rosbags,and store the final rosbags.

stl_models is the folder where the used stl files are stored, created with fusion360 and 3d printed.






