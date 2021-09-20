# PepperKinematics
Forward (FK) and inverse (IK) kinematics for the robot Pepper and a dataset of angle combinations and the corresponding end effector positions of both virtual and physical robot. The method is explained in more detailed in the following paper: 
- D. Stoeva, H. A. Frijns, M. Gelautz and O. Schürer, "Analytical Solution of Pepper’s Inverse Kinematics for a Pose Matching Imitation System," _2021 30th IEEE International Conference on Robot & Human Interactive Communication (RO-MAN)_, 2021, pp. 167-174, doi: 10.1109/RO-MAN50785.2021.9515480.

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
cd PepperKinematics
python main.py "robot-ip-address" "port"
```

example:

```
python main.py "127.0.0.1" "9559"
```


## Citation

Please cite the following paper in your publications if in your research you use the work published here:

```latex
@inproceedings{stoeva2021,
  title={Analytical Solution of Pepper's Inverse Kinematics for a Pose Matching Imitation System},
  booktitle={2021 30th IEEE International Conference on Robot   Human Interactive Communication (RO-MAN)}, 
  author={Stoeva, Darja and Frijns, Helena Anna and Gelautz, Margrit and Sch{\"u}rer, Oliver},
  booktitle={RO-MAN},
  pages={167-174},
  doi={10.1109/RO-MAN50785.2021.9515480},
  year={2021}
}
```

## License
Attribution-NonCommercial-ShareAlike 4.0 International  ([CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/)).
The PepperKinematics code is available for free non-commercial use or academic research, and may be redistributed under these conditions. See [license](/LICENSE) for details.
