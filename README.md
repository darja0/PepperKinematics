# PepperKinematics
Forward (FK) and inverse (IK) kinematics for the robot Pepper and a dataset of angle combinations and the corresponding end effector positions of both virtual and physical robot. The method is explained in more detailed in the following paper: 
- Analytical Solution of Pepper's Inverse Kinematics for a Pose Matching Imitation System

## Requirements
- Python 2.7 (numpy)
- NAOqi 2.5 Python (to run the main.py)

## Files Description

The code included in this repository includes functions that calculate the FK and IK of the Pepper robot for 3 body chains: head, arms, and torso. For more details on the derivation of the formulas please refer to the paper. The positions are in millimeters and the angles are in radians. 

`forward_kinematics.py`	        - functions for computing end-effector positions from joint angles 

`hip_knee_pairs.py`	  - dictionary for estimated values for the KneePitch and HipPitch relationship

`inverse_kinematics.py`	        - functions for computing joint angles from end-effector positions

`math_functions.py`     - functions for computing rotation and transformation matrices 

`main.py` - main file which executes motion command for each body chain (head, arms, torso) as an example for using the functions from the inverse and forward kinematics 

## Example Code

To run the example code in the `main.py`, which uses the functions derived from the IK solutions, use this command in the terminal:

```
python main.py "robot-ip-address"
```

example:

```
python main.py "127.0.0.1"
```


## Citation

Please cite the following paper in your publications if in your research you use the work published here:

```latex
@inproceedings{stoeva2021,
  title={Analytical Solution of Pepper's Inverse Kinematics for a Pose Matching Imitation System},
  author={Stoeva, Darja and Frijns, Helena Anna and Gelautz, Margrit and Sch{\"u}rer, Oliver},
  booktitle={RO-MAN},
  year={2021}
}
```

## License

The PepperKinematics code is freely available for free non-commercial use or academic research, and may be redistributed under these conditions. See [license](/LICENSE) for details.
