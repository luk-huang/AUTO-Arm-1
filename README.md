# MIT AUTO Arm
#### The AUTO Arm is an effort by the MIT Theoretical Physics Lab to create a robot that can assemble complex optical configurations quickly, speeding up the process by 90 percent. Furthermore, the robot can also implement AI and ML techniques to optimize layouts and to make second harmonic generation a possibility.
##Specifications
 - XArm 7
 - Stereo vision using two 4k Cameras: [ELP 4K USB Camera](https://www.amazon.com/ELP-Microphone-5-50mm-Varifocal-Vari-focus/dp/B0BVFKTM6Z/ref=asc_df_B0BVFKTM6Z/?tag=hyprod-20&linkCode=df0&hvadid=693308325592&hvpos=&hvnetw=g&hvrand=53687256479189848&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9191806&hvtargid=pla-2088917187303&psc=1&mcid=d2580302f178300ebafadc199fc36bd2&gad_source=1)
 - RealSense Depth Camera D435
 - Windows 10 development with Python 3.10
## Overview
 - The system utilizes **Aruco** Tags to autonomously find the position of elements placed in the bank.
 - The Stereo System finds the approximate location of the tag
 - The Arm moves to the predicted position placing RealSense camera above the tag
 - The Realsense camera fine adjusts and finds depth and orientation using linear algebra
 - Finally, we can pick up the object with 0.1mm translational precision and a tenth of a degree rotational precision

## Code
- [alignment.py](https://github.com/ShrishChou/AUTO-Arm/blob/main/alginment.py): Code for the assembly of an interferometer setup to test the precision of the robotic arm
- [aligment_with_drop.py](https://github.com/ShrishChou/AUTO-Arm/blob/main/aligment_with_drop.py): Code for the assembly of an interferometer with release of the mirror
- [finding_camera.py](https://github.com/ShrishChou/AUTO-Arm/blob/main/finding_camera.py): Code for finding the ports where the cameras are located. Change port values based on availability (varies per machine)
- [fine_adjustment.py](https://github.com/ShrishChou/AUTO-Arm/blob/main/fine_adjustment.py): Code for fine-adjustment using solely RealSense camera
- [full_adjustment.py](https://github.com/ShrishChou/AUTO-Arm/blob/main/fulladjustment.py): **Main Code** for running the full autonomous pick up code
- [internaltest.py](https://github.com/ShrishChou/AUTO-Arm/blob/main/internaltest.py): Code for testing rotation calculations
- [mainv2.py](https://github.com/ShrishChou/AUTO-Arm/blob/main/mainv2.py): First Version of working code without depth sensing and purely stereo system
- [stereo_callibration.npz](https://github.com/ShrishChou/AUTO-Arm/blob/main/stereo_calibration.npz): npz file generated from calibration of stereo system (currently using v2

## Using
- Run [fulladjustment.py](https://github.com/ShrishChou/AUTO-Arm/blob/main/fulladjustment.py) and wait for the 4k cameras to boot up.
- Once the start procedure has finished and the robot has homed, place the item and make sure the cameras can see the tag.

## Issues/Warnings and Solutions
- IP Issues: When setting up IP for the computer, set alternate IP configuration to 192.168.1 with basic masks and DNS Servers. Check that packages are being recieved and sent afterwards
- Camera port Access: Ensure that many cameras are not connected at the same time, ensure the camera app is off at the time of use, and make sure to use finding_camera.py to find ports of cameras before hand
- Calibration Issues: Use chackerboard of known size and measure dimensions of boxes after print. **Specified checkerboard size may differ to print.** Adjust scaling factor by measuring actual distance between camera and predicted translational distance
- RealSense Issues: Ensure correct version of Python is downloaded (3.6~3.10) if not you must wipe the virtual environment, downgrade, and then activate a new environment
- Movement Issues: Using the given set_position will often fail. For this reason I have developed a movement code that utilizes trigonometry and planned motion to eliminate collisions. Do not use set_position and instead utilize the pickup and drop methods
- Camera Crash issues: Due to the running of 2 4k cameras and a RealSense camera, some systems cannot handle the numerous feeds. For this you can change the stream code to instead take a singular picture at the start of each motion rather than keeping continuous streams for the 4k cameras.
- Gripper VS Suction Gripper: For the use of different grippers, different methods have been made. Depending on which attachment is being used please use the correct code.
- Crashing of robot: Ensure that robot joints are not overheating and make sure the robot is enabled with sufficient power
- Detecting Failure: Ensure that correct cameras are being used and make sure that tag is in range of **both** cameras due to stereo system
- Issues in 2nd quadrant: Due to rotational constraints and issues with set_position, be sure to only let the base servo turn to a max of 90 degrees (from the horizontal) and let set_position do calculations
- Issues with horizontal movement: Cameras may or may not be reversed. In our code the horizontal is reversed so we have negated the values. Be sure to adjust the value on your cases
- **Warning**: **Do NOT let the arm crash into the ground and always be positioned near the emergency stop button in case of failure**
- **Warning**: **Be sure to not move stereo system after calibration or else readings will fail**
