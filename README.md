# MagVehi: Realtime Online One-Dimensional Magnetic Field SLAM for drift correction

This repository is an incremental version of the original repository base on the follow research：
* Manon Kok and Arno Solin (2024). **Online One-Dimensional Magnetic Field SLAM with Loop-Closure Detection**. In *IEEE International Conference on Multisensor Fusion and Integration (MFI)*. [arXiv preprint](https://arxiv.org/abs/2409.01091)

### Motivation


## Dependencies

The codes in this repository have been tested with **Mathworks MATLAB R2024a (Update 1)**. The core functions (those in `src`) do not depend on any additional toolboxes, but helper files in `tools`. The main file runSLAM.m and the plotting file makePlots.m in tools use the procrustes function from the following built-in toolboxes:
* Statistics and Machine Learning Toolbox (tested with version 24.1)
to compute the RMSE.

## Structure of the codes

### Main file to run

```
  runDemo - Main file to make all results
```

### Core functions (under `src`)

```
  magSLAMwithLoopClosures - Run 1D magnetic field SLAM with loop closures
  run_filter_from_scratch - Runs the EKF from the start of the data set
```

### Helper functions (under `tools`)

```
             dynamics - Dynamic model
            makePlots - Generates plots from the paper
	  prepareData - Prepares data to be used in EKF, also adds a random noise realization to the odometry
    	     quat2eul - Converts quaternion to Euler angles
                 rotx - Computes a rotation matrix from a rotation angle around the x-axis
		 roty - Computes a rotation matrix from a rotation angle around the y-axis
		 rotz - Computes a rotation matrix from a rotation angle around the z-axis
```

## Data availability and access

Data used in the examples are provided in the folder `data`.

## Running experiment examples

In the `runDemo.m` file it is possible to choose which of the four data sets to run in the variable `indDataSet` on line 41.

## License

This software is provided under the [MIT License](LICENSE).
