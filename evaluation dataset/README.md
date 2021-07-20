### Evaluation Data

Here you will find csv file containing end-effector positions and the corresponding angle values within a body chain of both physical and virtual robot. 

For Pepper we have defined 3 body chains: head, arm, torso. For the head and arm chain, the end-effector 3D positons are stored in the Torso coordinate system, while for the torso chain the 3D positions are stored in the Robot coordinate system (this is the middle of the three wheels of the Pepper's leg, see this [link](http://doc.aldebaran.com/2-5/naoqi/motion/control-cartesian.html#control-cartesian) for more information on the Torso and Robot frame).

To generate this evaluation data for the kinematical model, different joint angle combinations (within the range of the joints) for each body chain were sent as motion commands to Pepper. For exmaple, for the head different values for the HeadYaw and HeadPitch were sent as commands and once executed (Pepper in new posture) the joint angles and the end-effector were stored. Thus, each row in the csv file corresponds to one individial posture that has specific combination of joint angle values within the body chain and the corresponding end-effector position. 

In the csv files in the header row:

- *joint*_sent is the sent angle value used to send the motion command to the robot

- *joint*_sensed is the sense angle valuse stored after the motion command has been executed 

- *end-effector*_x, *end-effector*_y, and *end-effector*_z are the X, Y, and Z positions of the end-effector stored after the motion command has been executed
