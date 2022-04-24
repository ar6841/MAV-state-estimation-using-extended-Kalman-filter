Project 1

Robot Localization and Navigation

Arjun Raja -- ar6841

1.  **Project overview**

This project entails the design of an Extended Kalman Filter to estimate
the state of a Micro Aerial Vehicle. The state of the MAV contains
position, velocity, and orientation, and gyroscope sensor bias and
accelerometer sensor bias. The body frame acceleration and angular
velocity from the on board IMU are being used as the inputs. The process
and measurement noise follow normal distributions.

2.  **Process model**

The state of the MAV is a vector X:

$$\mathbf{x} = \begin{bmatrix}
\&\mathbf{x}_{1} \\
\&\mathbf{x}_{2} \\
\&\mathbf{x}_{3} \\
\&\mathbf{x}_{4} \\
\&\mathbf{x}_{5} \\
\end{bmatrix} = \begin{bmatrix}
\mathbf{p} \\
\mathbf{q} \\
\dot{\mathbf{p}} \\
\mathbf{b}_{g} \\
\mathbf{b}_{\mathbf{a}} \\
\end{bmatrix} = \begin{bmatrix}
\text{\ position\ } \\
\text{\ orientation\ } \\
\text{\ linear\ velocity\ } \\
\text{\ gyroscope\ bias\ } \\
\text{\ accelerometer\ bias\ } \\
\end{bmatrix} \in \mathbf{R}^{15}$$

In MATLAB this can be represented by a 15x1 vector, with the vectors :
$\mathbf{x}_{1} = \lbrack x,y,z\rbrack^{\top}\ \mathbf{,\ x}_{2} = \lbrack angle\_ x,angle\_ y,angle\_ z\rbrack^{\top}\mathbf{,x}_{3} = \lbrack vx,vy,vz\rbrack^{\top}\mathbf{,x}_{4} = \lbrack bg1,bg2,bg3\rbrack^{\top}\mathbf{,x}_{5} = \lbrack ba1,ba2,ba3\rbrack^{\top}.\ $The
data for the state of the MAV is given in the code by uPrev(i , 1).

**2.1 R matrix**

The rotation order is ZYX by the Euler angles
$\lbrack angle\_ x,angle\_ y,angle\_ z\rbrack = \lbrack\phi,\theta,\psi\rbrack\ $in
this project. So we can find the rotation matrix R

$${}^{W}R_{B} = R_{z}(\psi)R_{y}(\theta)R_{x}(\phi)$$

R is precalculated and implemented in the code, thus the required values
can be substituted in.

**2.2 G matrix calculation**

The G matrix is a mapping from Euler angle rates to angular velocity.
Our input is the **body frame angular velocity** of the robot from the
gyroscope, and we need to map it to the Euler angle rates so we can find
${\dot{\mathbf{x}}}_{\mathbf{2}}$**.**

$${\dot{x}}_{2} = G^{- 1}{}^{B}\omega$$

The G matrix can be found by splitting the angular velocity into the
angular velocity contributions of the Euler angle rates. In other words,
angular velocity contribution is the (Euler angle rate) ×(Projection of
axis of rotation in the body frame).

![A picture containing text, wire, flock, line Description automatically
generated](media/image1.png){width="2.3200021872265966in"
height="2.0543536745406823in"}

$${}^{B}\omega = \begin{bmatrix}
\& 1 \\
\& 0 \\
\& 0 \\
\end{bmatrix}\dot{\phi} + \begin{bmatrix}
0 \\
\cos\phi \\
 - \sin\phi \\
\end{bmatrix}\dot{\theta} + \begin{bmatrix}
 - \sin\theta \\
\cos{\theta\sin\phi} \\
\cos{\theta\cos\phi} \\
\end{bmatrix}\dot{\psi}$$

$${}^{B}\omega = \begin{bmatrix}
1 & 0 & - \sin\theta \\
0 & \cos\phi & \cos\theta\sin\phi \\
0 & - \sin\phi & \cos\theta\cos\phi \\
\end{bmatrix}\begin{bmatrix}
\&\dot{\phi} \\
\&\dot{\theta} \\
\&\dot{\psi} \\
\end{bmatrix}$$

$$\ {{}^{B}\omega = G\dot{x}}_{2}$$

As the projected the axes of rotation are in the body frame, the angular
velocity in the derivation is in the body frame. There is no need to
rotate the angular velocity, so there's no need to multiply G with Rinv.

As $G^{- 1}$ is used in the process model it has been pre calculated and
only the Euler angle values need to be substituted to find $G^{- 1}$

Code Sample:

![](media/image2.png){width="6.844935476815398in"
height="0.5422255030621173in"}

**2.3 Process model**

The process model assumes that $\dot{x} = f(x,u,n)$

$$\dot{\mathbf{x}} = \begin{bmatrix}
\mathbf{x}_{3} \\
G\left( \mathbf{x}_{2} \right)^{- 1}\left( \mathbf{\omega}_{\mathbf{m}} - \mathbf{x}_{4} - \mathbf{n}_{g} \right) \\
\mathbf{g} + R\left( \mathbf{x}_{2} \right)\left( \mathbf{a}_{m} - \mathbf{x}_{5} - \mathbf{n}_{a} \right) \\
\mathbf{n}_{bg} \\
\mathbf{n}_{ba} \\
\end{bmatrix} = f(x,u,n)$$

Where $\mathbf{g}$ is acceleration due to gravity implemented by the
vector $\lbrack 0,0, - 9.81\rbrack^{\top}$, $\mathbf{n}_{bg}$is the
noise due to gyroscope bias and $\mathbf{n}_{ba}$is noise due to
accelerometer bias. The inputs to the system (angular velocity and
acceleration) are represented by the vector U, and the process noise is
represented by the noise vector N. $\mathbf{x}$ is calculated and
implemented in the prediction step around the point
$\left( \mu_{t - 1}u_{t},0 \right)$.

Code Sample:

![Text, letter Description automatically
generated](media/image3.png){width="3.500725065616798in"
height="1.0429254155730534in"}

![](media/image4.png){width="6.268055555555556in"
height="0.47638888888888886in"}

The process noise has a covariance matrix Q \[15x15\] and is randomly
assigned when the program is run using the MATLAB rand() and diag()
functions. It remains constant between iterations so Q was implemented
by persistent data type.

3.  **Measurement model**

The measurement model can be represented by the equation,

$$\begin{matrix}
\& Z_{t} = g(x,v_{t}) = C_{t}x_{t} + v_{t} \\
\& v_{t} \sim N\left( 0,R_{t} \right) \\
\end{matrix}$$

The measurement model is linear.

Where $v_{t}$ is the measurement noise. $R_{t}$ \[6x6\] and is randomly
assigned when the program is run using the MATLAB rand() and diag()
functions. It remains constant between iterations so $R_{t}$ was
implemented by persistent data type.

**3.1 Part 1**

The project is split into two parts, in part one we are measuring the
position $\mathbf{x}_{1}$ and orientation $\mathbf{x}_{2}$.

$C_{t}$ in part one is a \[6x15\] matrix $\begin{bmatrix}
\& I_{3}\ \ \ \ \&\& 0\ \ \ \ \&\& 0\ \ \ \ \&\& 0\ \ \ \ \&\& 0 \\
\& 0\ \ \ \ \&\& I_{3}\ \ \ \ \&\& 0\ \ \ \ \&\& 0\ \ \ \ \&\& 0 \\
\end{bmatrix}$ where $I_{3}$ is a 3x3 identity matrix

![A picture containing text Description automatically
generated](media/image5.png){width="4.269229002624672in"
height="0.34764873140857394in"}

So $g(x,v_{t})$ gives us a 6x1 vector

**3.2 Part 2**

In part two we measure velocity $\mathbf{x}_{3}$ and $C_{t}$ in part two
is a \[3x15\] matrix $\begin{bmatrix}
\& 0\ \ \ \ \&\& I_{3}\ \ \ \ \&\& 0\ \ \ \ \&\& 0\ \ \ \ \&\& 0 \\
\end{bmatrix}$

![](media/image6.png){width="4.390133420822397in"
height="0.3239337270341207in"}

Here $g(x,v_{t})$ gives us a 3x1 vector

4.  **Prediction step**

As the process is non-linear, the Extended Kalman Filter linearizes the
system around $\dot{\mathbf{x}}\left( \mu_{t - 1}u_{t},0 \right)$.

And the discrete prediction step can be found to be

$$\begin{matrix}
\&{\overline{\mu}}_{t} = \mu_{t - 1} + \delta tf\left( \mu_{t - 1},u_{t},0 \right) \\
\&{\overline{\Sigma}}_{t} = F_{t}\Sigma_{t - 1}F_{t}^{T} + V_{t}Q_{d}V_{t}^{T} \\
 \\
\end{matrix}$$

$$\begin{matrix}
\&\dot{x} = f(x,u,n) \\
\& n \sim N(0,Q) \\
\& A_{t} = \left. \ \frac{\partial f}{\partial x} \right|_{\mu_{t - 1},u_{t},0} \\
\& U_{t} = \left. \ \frac{\partial f}{\partial n} \right|_{\mu_{t - 1},u_{t},0} \\
\& F_{t} = I_{15} + \delta tA_{t} \\
\& V_{t} = U_{t} \\
\& Q_{d} = Q\delta t \\
\end{matrix}$$

The step size dt = $\delta t$is given as an input into the pred_step
function, it's calculation is shown in the main loop description below.
The calculation of the process noise has a covariance matrix Q was shown
above in the process model.

The Jacobians $A_{t}$ and $U_{t}$ were pre calculated and are
implemented as persistent symbolic functions using matlabFunction(),
this was done to increase the speed of the program.

Code Sample:

![Text, letter Description automatically
generated](media/image7.png){width="3.8684109798775155in"
height="0.9337543744531933in"}

![A picture containing chart Description automatically
generated](media/image8.png){width="7.093534558180227in"
height="2.3683956692913384in"}

The values for the state (form uPrev), noise (all 0 as we linearize
around N=0) and inputs (angVel and acc) are substituted directly into
the symbolic MATLAB functions to calculate At and Ut. **The prediction
step stays the same for part one and part two.**

The pred_step function finally gives us the uEst and covarEst to be used
in the update model

Code sample:

![](media/image9.png){width="3.8215966754155732in"
height="0.44202865266841646in"}

5.  **Update step**

As the measurement model is linear, the update step takes the form:

$$\begin{matrix}
\mu_{t}\& = {\overline{\mu}}_{t} + K_{t}\left( z_{t} - C_{t}{\overline{\mu}}_{t} \right) \\
\Sigma_{t}\& = {\overline{\Sigma}}_{t} - K_{t}C_{t}{\overline{\Sigma}}_{t} \\
K_{t}\& = {\overline{\Sigma}}_{t}C_{t}^{T}\left( C_{t}{\overline{\Sigma}}_{t}C_{t}^{T} + R_{t} \right)^{- 1} \\
\end{matrix}$$

Where Kt is the Kalman gain and $z_{t}$ the measurement.

The calculations for $R_{t}$ and $C_{t}$ are the same as discussed above
in the measurement models

**5.1 Part 1**

Rt is the covariance matrix of the measurement noise and is a 6x6
diagonal matrix in part one. Ct was the same as found in measurement
model part one.

**5.2 Part 2**

Rt in part two is a 3x3 diagonal matrix. Ct was the same as found in
measurement model part two

The update step gives us uCurr and covarCurr.

![](media/image10.png){width="4.043640638670166in"
height="0.44665463692038493in"}

6.  **Main Loop**

![A screenshot of a computer Description automatically
generated](media/image11.png){width="3.494329615048119in"
height="1.9005304024496938in"}

In the main loop, first dt is determined by finding the difference
between the sampled time and previous sampled time. The angular velocity
and acceleration are read from the sampled data structure which contains
the gyroscope and accelerometer data.

Then the state estimate (u_est) and covariance estimate (covar_est) are
found by feeding the previous state, previous covariance, angular
velocity, acceleration and time step dt into the prediction step
function (pred_step).

The current measurement, state estimate and covariance estimate are fed
into the update step function (upd_step) so we can find the current
state(u_curr) and current covariance (cov_curr). The current values,
u_curr and covar_curr are saved and used in the next iteration of the
loop. All values of u_curr are also saved in the savedStates matrix so
it can be plotted later.

7.  **Results**

Given below are the plots of the MAV state under different data sets and
measurement models.

**7.1 Data set 1**

**Part 1**

![Diagram Description automatically generated with medium
confidence](media/image12.png){width="5.536766185476815in"
height="2.8922944006999125in"}

**Part 2**

![Graphical user interface, diagram Description automatically generated
with medium confidence](media/image13.png){width="5.5903926071741035in"
height="2.8930555555555557in"}

**7.2 Data set 4**

**Part 1**

![A close-up of some writing Description automatically generated with
low confidence](media/image14.png){width="5.636973972003499in"
height="3.04331583552056in"}

**Part 2**

![Chart, line chart Description automatically
generated](media/image15.png){width="5.37750656167979in"
height="2.826379046369204in"}

**7.3 Data set 9**

**Part 1**

![Diagram Description automatically
generated](media/image16.png){width="5.483571741032371in"
height="2.616026902887139in"}

**Part 2**

![Chart, diagram Description automatically
generated](media/image17.png){width="6.168653762029746in"
height="3.225797244094488in"}

8.  **Discussions**

The Extended Kalman filter estimated the state of the MAV with high
accuracy. When only the position and orientation are used in the
measurement model, as in part one, the extended Kalman filter works well
and tracks position, orientation velocity, gyroscope and accelerometer
biases . But when the measurement model only measures the velocity of
the MAV, as done in part two, the accuracy of the filter drops a bit and
there is a drift in the yaw / angle_z. This might be because the filter
does not have enough information, we only measured the velocity to
calculate the Kalman gain which doesn't correct the yaw enough, which
causes its yaw value to drift as the Kalman gain never corrects it
completely and the process noise adds up. It's possible to reduce the
error in this case by scaling the process noise covariance matrix Q
down, but it's not done for the purposes of this report as to make the
phenomenon apparent.
