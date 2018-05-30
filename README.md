# Estimation Project #

## The Goal of this Project ##

In this project, we will be developing an estimator to be used by your controller to successfully fly a desired flight path using realistic sensors. 

<p align="center">
<img src="images/intro.png" width="400"/>
</p>

## Writeup ##
- [README](./README.md) 

## Implement Estimator ##
- [QuadEstimatorEKF.cpp](src/QuadEstimatorEKF.cpp) and [QuadEstimatorEKF.txt](config/QuadEstimatorEKF.txt) containing your estimator and associated estimator parameters that successfully meets all the performance criteria.
- [QuadController.cpp](src/QuadControl.cpp) and [QuadControlParams.txt](config/QuadControlParams.txt) containing your re-tuned controller needed to work successfully with your estimator.
---


### Step 1: Sensor Noise ###
#### Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data. ####
The calculated standard deviation should correctly capture ~68% of the sensor measurements. Your writeup should describe the method used for determining the standard deviation given the simulated sensor measurements. 

- changes are reflected in [config/06_SensorNoise.txt](config/06_SensorNoise.txt)
- MeasuredStdDev_GPSPosXY = 0.67
- MeasuredStdDev_AccelXY = .49
- Config logs [config/log/Graph1.txt](test/GraphSN1.txt) and [config/log/Graph1.txt](test/GraphSN2.txt)
- [Standard deviation processor](test/SensorNoise.py)

```
- Run simulator using 06_NoisySensors.txt
- Collect Config logs
- Process standard deviation using collected files 
- Updated config/6_Sensornoise.txt with computed MeasuredStdDev_GPSPosXY and MeasuredStdDev_AccelXY
- Run the simulator using update 06_NoisySensors.txt, sensor mesurements should correctly capture ~68%
```
 <img src="images/sensor-noise-value.PNG" width="500" height="40" alt="Values" /> 

 Before                     |  After
 :-------------------------:|:-------------------------:
 <img src="images/sensor-noise-before.PNG" width="450" height="250" alt="Before" /> |  <img src="images/sensor-noise-after.PNG" width="450" height="250" alt="After" />
 
 ----
### Step 2: Attitude Estimation ###
#### Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function. ####
The improved integration scheme should result in an attitude estimator of < 0.1 rad for each of the Euler angles for a duration of at least 3 seconds during the simulation. The integration scheme should use quaternions to improve performance over the current simple integration scheme. 

- changes are reflected in [src/QuadEstimatorEKF.cpp#L96-L108](src/QuadEstimatorEKF.cpp#L96-L108)

```
- Comment out predictedPitch ,predictedRoll and ekfState(6) to avoid integrating yaw twice
- Use quaternions as integration scheme to improve performance over current simple integration scheme
- Though not Euler angles, Quaternion has IntegrateBodyRate to use.
- Compute predictedPitch, predictedRoll and ekfState(6) using Quaternion Roll, Pitch and Yaw
```
<p align="center">
 <img src="images/AttitudeEstimation.PNG" width="800" height="600" alt="Before" /> 
</p>

 ----
### Step 3: Prediction Step ###
#### Implement all of the elements of the prediction step for the estimator. ####
The prediction step should include the state update element (PredictState() function), a correct calculation of the Rgb prime matrix, and a proper update of the state covariance. The acceleration should be accounted for as a command in the calculation of gPrime. The covariance update should follow the classic EKF update equation. 

- changes are reflected for PredictState in [PredictState()#L173-L192](src/QuadEstimatorEKF.cpp#L173-L192)
- changes are reflected for GetRbgPrime in [GetRbgPrime()#L216-L234](src/QuadEstimatorEKF.cpp#L216-L234)
- changes are reflected for Predict in [Predict()#L277-L291](src/QuadEstimatorEKF.cpp#L277-L291)
- changes are reflected for Covariance in [QuadEstimatorEKF.cpp#L289](src/QuadEstimatorEKF.cpp#L289)
- [Predict ref](images/function_predict.gif)
- [RBGPrime ref](images/rbg_prime.gif)
- [Jacobian ref](images/jacobian.gif)
- [Transition ref](images/transition_function.gif)
- [Covariance ref](images/update_state_covariance.gif)

```
Predict consist of the following
- PredictState(7 states):
  -- predict x, y, z (3 states)
  -- attitude.Rotate_BtoI(<V3F>) to rotate a vector from body frame to inertial frame
  -- predict x_dot, y_dot, z_dot (3 states)
  -- yaw get updated in IMU code (1 state)
- GetRbgPrime(as indicated by RBGPrime):
  -- This is just a matter of putting the right sin() and cos() functions in the right place.
- Compute GPrime using Transition and Jacobian reference
- Finally compute state covariance
```
PredictState                     |  PredictCovariance
 :-------------------------:|:-------------------------:
 <img src="images/PredictState.PNG" width="450" height="250" alt="Before" /> |  <img src="images/PredictCovariance.PNG" width="450" height="250" alt="After" />
 
 ----
### Step 4: Magnetometer Update ###
#### Implement the magnetometer update. ####
The update should properly include the magnetometer data into the state. Note that the solution should make sure to correctly measure the angle error between the current state and the magnetometer value (error should be the short way around, not the long way). 

- changes are reflected in [src/QuadEstimatorEKF.cpp#L342-L361](src/QuadEstimatorEKF.cpp#L342-L361)

```
- Get yaw estimates
- Normalize the difference between your measured and estimated yaw to -pi .. pi
- Update Partial Derivative of Mesurement model hPrime
```
<p align="center">
 <img src="images/MagUpdate.PNG" width="800" height="600" alt="Before" /> 
</p>

 ----
### Step 5: Closed Loop + GPS Update ###
#### Implement the GPS update. ####
The estimator should correctly incorporate the GPS information to update the current state estimate. 

- changes are reflected in [src/QuadEstimatorEKF.cpp#L313-L323](src/QuadEstimatorEKF.cpp#L313-L323)
- changes are reflected in [config/11_GPSUpdate.txt](config/11_GPSUpdate.txt)
- Quad.UseIdealEstimator = 0
- #SimIMU.AccelStd = 0,0,0
- #SimIMU.GyroStd = 0,0,0
- "Estimation for Quadrotors" paper equations (53), (54), and (55)  
<p align="center">
 <img src="images/GPSUpdate.PNG" width="800" height="600" alt="Before" /> 
</p>

```
- Get yaw estimates
- Update Partial Derivative hPrime which is identity matrix here
```
<p align="center">
 <img src="images/GPSUpdate-Controller.PNG" width="800" height="600" alt="Before" /> 
</p>


 ----
## Flight Evaluation ##

### Meet the performance criteria of each step. ###
#### For each step of the project, the final estimator should be able to successfully meet the performance criteria with the controller provided. The estimator's parameters should be properly adjusted to satisfy each of the performance criteria elements. ####
```
```

### De-tune your controller to successfully fly the final desired box trajectory with your estimator and realistic sensors. ###
#### The controller developed in the previous project should be de-tuned to successfully meet the performance criteria of the final scenario (<1m error for entire box flight). ####
```
```


## The Tasks ##

Once again, you will be building up your estimator in pieces.  At each step, there will be a set of success criteria that will be displayed both in the plots and in the terminal output to help you along the way.

Project outline:

 - [Step 6: Adding Your Controller](#step-6-adding-your-controller)





### Step 6: Adding Your Controller ###

Up to this point, we have been working with a controller that has been relaxed to work with an estimated state instead of a real state.  So now, you will see how well your controller performs and de-tune your controller accordingly.

1. Replace `QuadController.cpp` with the controller you wrote in the last project.

2. Replace `QuadControlParams.txt` with the control parameters you came up with in the last project.

3. Run scenario `11_GPSUpdate`. If your controller crashes immediately do not panic. Flying from an estimated state (even with ideal sensors) is very different from flying with ideal pose. You may need to de-tune your controller. Decrease the position and velocity gains (we’ve seen about 30% detuning being effective) to stabilize it.  Your goal is to once again complete the entire simulation cycle with an estimated position error of < 1m.

**Hint: you may find it easiest to do your de-tuning as a 2 step process by reverting to ideal sensors and de-tuning under those conditions first.**

***Success criteria:*** *Your objective is to complete the entire simulation cycle with estimated position error of < 1m.*


## Tips and Tricks ##

 - When it comes to transposing matrices, `.transposeInPlace()` is the function you want to use to transpose a matrix

 - The [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj) document contains a helpful mathematical breakdown of the core elements on your estimator

## Submission ##

For this project, you will need to submit:

 - a completed estimator that meets the performance criteria for each of the steps by submitting:
   - `QuadEstimatorEKF.cpp`
   - `config/QuadEstimatorEKF.txt`

 - a re-tuned controller that, in conjunction with your tuned estimator, is capable of meeting the criteria laid out in Step 6 by submitting:
   - `QuadController.cpp`
   - `config/QuadControlParams.txt`

 - a write up addressing all the points of the rubric

## Authors ##

Thanks to Fotokite for the initial development of the project code and simulator.
