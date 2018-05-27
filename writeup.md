## Writeup

---

**Extended Kalman Filter Project Project**

[//]: # (Image References)
[image1]: ./pics/final.png
[image2]: ./pics/without_phi_normalization.png
[image3]: ./pics/laser_only.png
[image4]: ./pics/radar_only.png


## [Rubric](https://review.udacity.com/#!/rubrics/748/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 0. General discussion

Code is provided with writeup.md or available [here](https://github.com/psilin/CarND-Extended-Kalman-Filter-Project/). I have not made any changes to standard `cmake`-based building process
as I ended up only filling some methods.


#### 1. Algorithm

Firstly, I implemented `CalculateRMSE()` and `CalculateJacobian()` helpers method of `Tools` class (`src/tools.cpp`). I changed `CalculateJacobian()` signature so it could return error to 
its caller in explicit way. I used standard implementations of those methods provided in lesson materials.

Then I implemented `Predict()`, `Update()` and `UpdateEKF()` methods of `KalmanFilter` class (`src/kalman_filter.cpp`). In `Predict()` and `Update()` methods I used standard formulas used
in basic (linear) version of Kalman filter. I just reused formulas given in lesson materials. As for `UpdateEKF()`, I precomputed Jacobian matrix in current state point and passed it to
that method. Then I computed `h(x)` (as it is used instead of `H * x` in `y = z - h(x)` in EKF) using transformation from Cartesian to Polar corrdinate system. Usage of Jacobian matrix 
in covariance matrix `P` update summed up that method.

Then I moved to `FusionEKF` class (`src/FusionEKF.cpp`). I started with constructor method. I initialized `H` matrix for lidar mesurements and pre-initialized state transition 
matrix `F` and transition noise matrix `Q`. I did it to avoid changing `F` and `Q` matrix elemets that would not change later. Then I moved to `Process Measurement()` method. I started 
with initialization part. I initilized state covariance matrix `P` with `1` as position variables sigma square and `1000` as speed variables sigma square. Other elements were zeros. I 
used initialization of `P` provided in lesson and decided to stick with it as it gave me reasonable performance. Then I saved last timestamp and initialized position variables with
raw measurements in lidar (laser) case and set speed variables to zero as lidar is incapable of mesuring speed. In radar case I transformed measurements from Polar to Cartesian coordinate
system as follows `px = rho*cos(phi); py = -rho*sin(phi); vx = rhodot*cos(phi); vy = -rhodot*sin(phi);`. I obtained `px` and `py` directly using raw measurements and chose to use projection
of `vx` and `vy` on `rho` axis in Polar coordinates as initial speed estimation as in my opinion it is not accurate but still better the zero.

Then I moved to prediction step. I initialized elements of `F` and `Q` matrices that depended on `dt` - difference between two closest measurement timestamps. I made sure that if we had to
measurements arrived in same time from lidar and radar algorithm would not do unnecessary second prediction (as `dt == 0` in that case). I used acceleration noise values that were provided 
with code base. Then I moved to update step. I laser case I just passed laser `H` and `R` (measurement noise) matrices to `KalmanFilter` class and called `Update()` method. In case of radar
I passed radar `R` matrix to `KalmanFilter` class, computed Jacobian matrix and called `UpdateEKF()` method.

At that moment my Kalman Filter implementation was ready and I moved to testing phase.

#### 2. Accuracy

I tried to use my Kalman Filter implementation with simulator and got following result:

![alt text][image2]

I got unaaceptable results in terms of RMSE and got one big spike, that can be seen on image above. I decided that I need to improve ribustness of my filter. I made two changes. First, I
improved computation of Jacobian matrix. I realized that when term `sqrt(px*px + py*py)` very was small it could result in an innacurate Jacobian matrix computation as that term was used
in denominator for several matrix elements. I decided to discard Jacobian (and UpdateEKF step at all) when `sqrt(px*px + py*py)` is small. Second, I noticed that I did not normalaze
angle `phi` in formula `y = z - h(x)` in radar case. So I made sure that `phi` is in `[-pi,pi]` interval. Those changes helped (especially second one) and I got the following result:

![alt text][image1]

That result was acceptable interms of RMSE `(0.0974, 0.0855, 0.4517, 0.4404)` was smaller that desired RMSE of `(0.11, 0.11, 0.52, 0.52)` and more importantly the resulting estimate had 
no spikes.

I performend an estimation with only radar available:

![alt text][image4]

I performend an estimation with only laser available as well:

![alt text][image3]

It can be seen that using both sensors results in better performance (as we have more information about system). Lidar-only results looks better then radar-only as lidar produces less
noisy measurements then radar. Though radar helps to improve lidar-only solution. My opinion is the biggest advantage of using radar is the fact that it provides direct radial speed 
measurements while lidar does not measure speed in direct way. That is why it is reaaly helpful to use both sensors.


#### 3. Discussion

Although this filter performs quite nice it can only be used to estimate near linear motion as it model acceleration as a noise. So first suggestion to improve this filter is to use
more complicated vehicle model during prediction step.

Second suggestion is to implement some sort of catchers that can help in finding anomalies in measurements. If we can find and discard such measurements it can really improve robustness
of Kalman Filter.