# Localization of a micro aerial vehicle
This script is an application of the EKF to estimate the state of a tiny drone using its onboard IMU sensor data and Vicon data.

The state of the MAV includes [position, velocity, orientation, gyroscope sensor bias, accelerometer sensor bias]

Three datasets were prepared for the model to be tested on. 

Data set 1:

![image](https://user-images.githubusercontent.com/96152967/217721936-13149f67-f3dd-431f-8cfc-13822fb06cd0.png)

Data set 2:

![image](https://user-images.githubusercontent.com/96152967/217721993-853b9573-36c1-49d0-a20b-ec566342c1f1.png)

Data set 3:

![image](https://user-images.githubusercontent.com/96152967/217722095-e0b360ca-9887-4806-ab33-4c3bb9d80e1d.png)


Onboard IMU sensor data being used for prediction and VICON data being used for measurement updates

## Overview

This project entails the design of an Extended Kalman Filter to estimate the state of a Micro Aerial Vehicle. The state of the MAV contains position, velocity, and orientation, and gyroscope sensor bias and accelerometer sensor bias. The body frame acceleration and angular velocity from the on board IMU are being used as the inputs. The process and measurement noise follow normal distributions.

##	Process model

The state of the MAV is a vector $x$ :

$$
\mathbf{x}=\left[\begin{array}{l}
\mathbf{x}_1 \\
\mathbf{x}_2 \\
\mathbf{x}_3 \\
\mathbf{x}_4 \\
\mathbf{x}_5
\end{array}\right]=\left[\begin{array}{l}
\mathbf{p} \\
\mathbf{q} \\
\dot{\mathbf{p}} \\
\mathbf{b}_g \\
\mathbf{b}_{\boldsymbol{a}}
\end{array}\right]=\left[\begin{array}{c}
\text { position } \\
\text { orientation } \\
\text { linear velocity } \\
\text { gyroscope bias } \\
\text { accelerometer bias }
\end{array}\right] \in \mathbf{R}^{15}
$$

In MATLAB this can be represented by a 15x1 vector, with the vectors: $\mathbf{x}_1=[x, y, z]^{\top}, \mathbf{x}_2=\left[\text { angle} \_x, \text { angle} \_y, \text { angle} \_z\right]^{\top}, \mathbf{x}_3=[v x, v y, v z]^{\top}, \mathbf{x}_4=[b g 1, b g 2, b g 3]^{\top}, \mathbf{x}_5=[b a 1, b a 2, b a 3]^{\top}$

### R Matrix

The rotation order is $\mathrm{ZYX}$ by the Euler angles $\left[\text { angle} \_x, \text { angle} \_y, \text { angle} \_z\right]$ $=[\phi, \theta, \psi]$ in this project. So we can find the rotation matrix $\mathrm{R}$

$$
{ }^w R_B=R_z(\psi) R_y(\theta) R_x(\phi)
$$

$\mathrm{R}$ is precalculated and implemented in the code, we only need to substitute the required orientation angles.
### G Matrix calculation

The $\mathrm{G}$ matrix is a mapping from Euler angle rates to angular velocity. Our input is the body frame angular velocity of the robot from the gyroscope, and we need to map it to the Euler angle rates so we can find $\dot{\boldsymbol{x}}_2$.

$$
\dot{x}_2=G^{-1} \quad{ }^B \omega
$$

The $\mathrm{G}$ matrix can be found by splitting the angular velocity into the angular velocity contributions of the Euler angle rates. In other words, angular velocity contribution is the (Euler angle rate $) \times($ Projection of axis of rotation in the body frame).

![image](https://user-images.githubusercontent.com/96152967/217720614-be43654f-79a3-4a93-a91e-96d1bc5d08b2.png)

$$
\begin{gathered}
{ }^B \omega=\left[\begin{array}{l}
1 \\
0 \\
0
\end{array}\right] \dot{\phi}+\left[\begin{array}{c}
0 \\
\cos \phi \\
-\sin \phi
\end{array}\right] \dot{\theta}+\left[\begin{array}{c}
-\sin \theta \\
\cos \theta \sin \phi \\
\cos \theta \cos \phi
\end{array}\right] \dot{\psi} \\
{ }^B \omega=\left[\begin{array}{ccc}
1 & 0 & -\sin \theta \\
0 & \cos \phi & \cos \theta \sin \phi \\
0 & -\sin \phi & \cos \theta \cos \phi
\end{array}\right]\left[\begin{array}{l}
\dot{\phi} \\
\dot{\theta} \\
\dot{\psi}
\end{array}\right] \\
{ }^B \omega=G \dot{x}_2
\end{gathered}
$$

As the projected the axes of rotation are in the body frame, the angular velocity in the derivation is in the body frame. There is no need to rotate the angular velocity, so there's no need to multiply G with Rinv.

As $G^{-1}$ is used in the process model it has been pre calculated and only the Euler angle values need to be substituted to find $G^{-1}$

### Process Model
The process model assumes that $\dot{x}=f(x, u, n)$

$$
\dot{\mathbf{x}}=\left[\begin{array}{c}
\mathbf{x}_3 \\
G\left(\mathbf{x}_2\right)^{-1}\left(\boldsymbol{\omega}_m-\mathbf{x}_4-\mathbf{n}_g\right) \\
\mathbf{g}+R\left(\mathbf{x}_2\right)\left(\mathbf{a}_m-\mathbf{x}_5-\mathbf{n}_a\right) \\
\mathbf{n}_{b g} \\
\mathbf{n}_{b a}
\end{array}\right]=f(x, u, n)
$$

Where $\mathbf{g}$ is acceleration due to gravity implemented by the vector $[0,0,-9.81]^{\top}, \mathbf{n}_{b g}$ is the noise due to gyroscope bias and $\mathbf{n}_{b a}$ is noise due to accelerometer bias. The inputs to the system (angular velocity and acceleration) are represented by the vector $\mathrm{U}$, and the process noise is represented by the noise vector $N$. $\mathbf{x}$ is calculated and implemented in the prediction step around the point $\left(\mu_{t-1} u_t, 0\right)$

The process noise has a covariance matrix $Q[15 \times 15]$ and is randomly assigned when the program is run using the MATLAB rand() and diag() functions. It remains constant between iterations so Q was implemented by persistent data type.

## Measurement model

The measurement model can be represented by the equation,

$$
\begin{aligned}
& Z_t=g\left(x, v_t\right)=C_t x_t+v_t \\
& v_t \sim N\left(0, R_t\right)
\end{aligned}
$$

The measurement model is linear.
Where $v_t$ is the measurement noise. $R_t[6 \times 6]$ and is randomly assigned when the program is run using the MATLAB rand() and $\operatorname{diag}()$ functions. It remains constant between iterations so $R_t$ was implemented by persistent data type.

### Part 1
The project is split into two parts, in part one we are measuring the position $\mathbf{x}_1$ and orientation $\mathbf{x}_2$.

$$
C_t \text { in part one is a }[6 \times 15] \text { matrix }\left[\begin{array}{ccccc}
I_3 & 0 & 0 & 0 & 0 \\
0 & I_3 & 0 & 0 & 0
\end{array}\right] \text { where } I_3 \text { is a } 3 \times 3 \text { identity matrix }
$$

So $g\left(x, v_t\right)$ gives us a $6 \times 1$ vector

### Part 2
In part two we measure velocity $\mathbf{x}_3$ and 

$$
C_t \text { in part two is a }[3 \times 15] \text { matrix }\left[\begin{array}{lllll}
0 & I_3 & 0 & 0 & 0
\end{array}\right]
$$

Here $g\left(x, v_t\right)$ gives us a $3 \times 1$ vector

## Prediction Step
As the process is non-linear, the Extended Kalman Filter linearizes the system around $\dot{\mathbf{x}}\left(\mu_{t-1} u_t, 0\right)$
And the discrete prediction step can be found to be

$$
\begin{gathered}
\bar{\mu}_t=\mu_{t-1}+\delta t f\left(\mu_{t-1}, u_t, 0\right) \\
\bar{\Sigma}_t=F_t \Sigma_{t-1} F_t^T+V_t Q_d V_t^T \\
\dot{x}=f(x, u, n) \\
n \sim N(0, Q) \\
A_t=\left.\frac{\partial f}{\partial x}\right|_{\mu_{t-1}, u_t, 0} \\
U_t=\left.\frac{\partial f}{\partial n}\right|_{\mu_{t-1}, u_t, 0} \\
F_t=I_{15}+\delta t A_t \\
V_t=U_t \\
Q_d=Q \delta t
\end{gathered}
$$

The step size $\mathrm{dt}=\delta t$ is given as an input into the pred_step function, it's calculation is shown in the main loop description below. The calculation of the process noise has a covariance matrix $\mathrm{Q}$ was shown above in the process model.

The Jacobians $A_t$ and $U_t$ were pre calculated and are implemented as persistent symbolic functions using matlabFunction(), this was done to increase the speed of the program. The values for the state (form uPrev), noise (all 0 as we linearize around $\mathrm{N}=0$ ) and inputs (angVel and acc) are substituted directly into the symbolic MATLAB functions to calculate At and Ut. The prediction step stays the same for part one and part two.

The pred_step function finally gives us the uEst and covarEst to be used in the update model

## Update Step
As the measurement model is linear, the update step takes the form:

$$
\begin{aligned}
\mu_t & =\bar{\mu}_t+K_t\left(z_t-C_t \bar{\mu}_t\right) \\
\Sigma_t & =\bar{\Sigma}_t-K_t C_t \bar{\Sigma}_t \\
K_t & =\bar{\Sigma}_t C_t^T\left(C_t \bar{\Sigma}_t C_t^T+R_t\right)^{-1}
\end{aligned}
$$

Where $\mathrm{Kt}$ is the Kalman gain and $z_t$ the measurement.
The calculations for $R_t$ and $C_t$ are the same as discussed above in the measurement models

### Part 1
Rt is the covariance matrix of the measurement noise and is a 6x6 diagonal matrix in part one. Ct was the same as found in measurement model part one

### Part 2
Rt in part two is a 3x3 diagonal matrix. Ct was the same as found in measurement model part two. The update step gives us uCurr and covarCurr.

## Main Loop 
In the main loop, first dt is determined by finding the difference between the sampled time and previous sampled time. The angular velocity and acceleration are read from the sampled data structure which contains the gyroscope and accelerometer data. 

Then the state estimate (u_est) and covariance estimate (covar_est) are found by feeding the previous, previous covariance, angular velocity, acceleration and time step dt into the prediction step function (pred_step).The current measurement, state estimate and covariance estimate are fed into the update step function (upd_step) so we can find the current state(u_curr) and current covariance (cov_curr). The current values, u_curr and covar_curr are saved and used in the next iteration of the loop. All values of u_curr are also saved in the savedStates matrix so it can be plotted later

## Discussions 
The Extended Kalman filter estimated the state of the MAV with high accuracy. When only the position and orientation are used in the measurement model, as in part one, the extended Kalman filter works well and tracks position, orientation velocity, gyroscope and accelerometer biases . But when the measurement model only measures the velocity of the MAV, as done in part two, the accuracy of the filter drops a bit and there is a drift in the yaw / angle_z. This might be because the filter does not have enough information, we only measured the velocity to calculate the Kalman gain which doesn’t correct the yaw enough, which causes its yaw value to drift as the Kalman gain never corrects it completely and the process noise adds up. It’s possible to reduce the error in this case by scaling the process noise covariance matrix Q down. The results of part 2 can be seen by running the KalmanFilt_Part2.m file






