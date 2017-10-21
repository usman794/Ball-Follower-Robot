# Ball-Follower-Robot
The objective of this project is to control a robot in such a way that the robot can follow a specific coloured ball and maintain a specific distance between robot and ball.
In the first step, the video taken from robot camera is firstly processed. Colour segmentation technique is used to identify red portion in the image which can identify the red ball in the image. But the segmented image will have some other identified rec portion due to background.
The the red colour portion is filteted out using blob ananlysis in which blob size and number of blobs are mentioned in parametrs.
Then as the ball is moving then in order to track the ball position, Kalman fiter is designed to keep the tracking of red coloured ball.
After this stage, the exact location of red coloured ball in the image is identified.
Using Kinect Sensor, depth information for each pixel is received. Using the depth information for the centre pixel of red coloured ball, control scheme is designed.
Control scheme is the proportional controlled scheme whioch maintains the ball at the centre of image and also make the robot to move forwrad or backward depending upon on the error of the depth information of the ecntre pixel of ball and the specific distance that is to be maintined.
THe code wil run with trutlebot configuration.


-Run directly the code track_red_ball_08_07_integral.m

For visualization of the working of turtlebot please visit,
https://www.youtube.com/watch?v=yZw_IG3000E
