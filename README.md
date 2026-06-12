

It's a 2 DOF Crawler, learns to crawl with the help of Reinforcement learning. I have reproduced Dr. Tokic's work: https://www.tokic.com/the-crawling-robot/

## Breif explanation about state and action spaces

## Videos
**Training:**
https://github.com/user-attachments/assets/65e78f59-ca38-4cd4-91ea-735fb18d2261


**Execution:**
https://github.com/user-attachments/assets/c02a3ba8-13a6-41f9-95b5-c9a6a0725c32


## Parts list, and Assembly

**Parts required:**
1. Arduino Mega
2. IMU
3. Servo Motors x 2
4. Breadboard with Jumper wires
5. And of course, a 3d printer to print CAD files

**CAD files**
The full CAD file is here: CAD files/crawler_full_assembly.f3z

**Wiring**


## Explanation of Files and main code

The major files are:

1. Q_table_update
   - It's the most basic implementation of RL. After calibration of the reward function, this file is the first go-to file to run the Crawler.
2. RewardFunction
   - Needed to calibrate the Imu. Just run this, move the Imu/Robot, and see if the rewards are appropriate according to motion.
3. Q_table_update_from_local_python
   - As the name suggests, we can run heavy lifting calculations in the PC (when we want to increase the state and action spaces)
4. PC stuff/ pc_processing.ipynb
   - Goes along with 'Q_table_update_from_local_python'. Before running this file, run the 'Q_table_update_from_local_python' in Arduino, and close the Arduino window.


