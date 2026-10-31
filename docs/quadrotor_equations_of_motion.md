# Quadrotor equations of motion

**Further reading.** This note gives the full dynamics derivation. The lectures
and exercise worksheets introduce the reduced models used in the scheduled
practicals; consult this note for details or to explore the modelling extensions.

This note derives the rigid-body equations used in the lectures and explains how
thrust, torque, angular velocity, and orientation are related.

See the [documentation guide](README.md) for the reading sequence. Use the
[Day 1 guide](day_1_control.md) and [controller worksheet](controller_practical.md)
for the outer-loop practical, and the [Day 2 guide](day_2_kalman.md) and
[estimation worksheet](kalman_filter_practical.md) for sensor-frame and process-model
work. These derivations provide background for the practicals.

## Frames and conventions

A [coordinate frame](glossary.md#frames) specifies the origin and axes in which
vector components are expressed.

Use two right-handed coordinate frames:

- world frame \(W\), with \(e_3=(0,0,1)^T\) pointing upward;
- body frame \(B\), fixed to the quadrotor, with its third axis along positive
  rotor thrust.

Let

\[
R\in SO(3)
\]

rotate body-frame coordinates into world-frame coordinates:

\[
v_W=Rv_B.
\]

The body thrust axis expressed in world coordinates is therefore

\[
b_3=Re_3.
\]

The position \(r\), velocity \(\dot r\), and acceleration \(\ddot r\) are
expressed in world coordinates. Angular velocity

\[
\omega=(p,q,r_\omega)^T
\]

is expressed in body coordinates. Its components \(p,q,r_\omega\) give the
angular rates about body X, Y and Z, respectively. Together they specify the
instantaneous axis and rate of rotation. The subscript on \(r_\omega\)
distinguishes this angular-rate component from position \(r\).

Equivalently, write angular velocity as an axis and a magnitude:

\[
\omega=\Omega\hat n_B,\qquad
\Omega=\lVert\omega\rVert=\sqrt{p^2+q^2+r_\omega^2},\qquad
\hat n_B=\frac{\omega}{\Omega}\quad(\Omega>0).
\]

Here \(\Omega\) is the rotation speed in radians per second, and the unit vector
\(\hat n_B\) gives the instantaneous rotation axis in body coordinates. Its
direction has two independent parameters; the magnitude supplies the third.
The right-hand rule gives the sense of rotation. At zero angular velocity,
the axis is undefined. This representation describes the rotation at a given
instant; integrating the orientation kinematics gives its evolution over time.

The cross product appears when angular velocity acts on another vector. For
a body-fixed vector \(a_B\), its world representation \(a_W=Ra_B\) changes as

\[
\dot a_W=\omega_W\times a_W,\qquad \omega_W=R\omega.
\]

Thus \(R\) describes the current orientation, \(\omega\) describes its rate
of change, and \(\omega_W\times a_W\) describes the resulting change of a
body-fixed vector in world coordinates. Both operands of a cross product
must be expressed in the same frame. A vector that also moves relative to
the body has the additional term \(R\dot a_B\) in its world-frame derivative.

Some aerospace conventions take body \(z\) downward or define \(R\) in the
opposite direction. Their thrust signs and rotation kinematics differ. A formula
is meaningful only together with its frame and sign conventions.

## Complete rigid-body model

Under the assumptions below, the state is

\[
(r,v,R,\omega),\qquad v=\dot r,
\]

and the control input is

\[
u=
\begin{bmatrix}
f\\M
\end{bmatrix}
=
\begin{bmatrix}
f&M_x&M_y&M_z
\end{bmatrix}^T.
\]

The equations are

\[
\dot r=v,
\]

\[
m\dot v=-mg e_3+fRe_3+F_{\mathrm{external}},
\]

\[
\dot R=R[\omega]_\times,
\]

\[
J\dot\omega+\omega\times(J\omega)=M+M_{\mathrm{external}}.
\]

The simple lecture model sets the unmodelled external force and torque to zero:

\[
\boxed{\ddot r=-ge_3+\frac{f}{m}Re_3},
\]

\[
\boxed{\dot\omega=J^{-1}\left(M-\omega\times J\omega\right)}.
\]

## Translational equation

Newton's second law for the centre of mass is

\[
m\ddot r=\sum F.
\]

Gravity is

\[
F_g=-mg e_3.
\]

The four rotors produce total thrust magnitude \(f\) along the body vertical
axis. In body coordinates this force is \(f e_3\); in world coordinates it is

\[
F_T=R(fe_3)=fRe_3=fb_3.
\]

Consequently,

\[
m\ddot r=-mg e_3+fRe_3,
\]

and division by mass gives

\[
\ddot r=-ge_3+\frac{f}{m}Re_3.
\]

This equation shows why attitude controls horizontal motion. Rotor thrust has no
independent horizontal component in the body frame. The vehicle must tilt
\(b_3\) to create horizontal force.

### Hover

For level hover,

\[
Re_3=e_3,
\qquad
f=mg.
\]

The forces cancel and \(\ddot r=0\).

### Accelerating flight

For desired world-frame acceleration \(a_d\), the required total force is

\[
F_d=m(a_d+ge_3).
\]

Ignoring drag and actuator limits, choose

\[
b_{3,d}=\frac{a_d+ge_3}{\lVert a_d+ge_3\rVert},
\]

\[
f_d=m\lVert a_d+ge_3\rVert.
\]

Desired yaw supplies the remaining rotation about \(b_{3,d}\), completing the
desired orientation. This relation is central to geometric quadrotor control and
explains why position trajectories imply attitude trajectories.

## Rotational equation

### Angular momentum in the world and body frames

The angular momentum expressed in body coordinates is

\[
H_B=J\omega,
\]

where \(J\) is the inertia tensor about the centre of mass, expressed in body
coordinates. For a rigid body, its mass distribution relative to these axes is
fixed, so \(J\) is constant. Its world-frame representation is

\[
H_W=RH_B.
\]

Angular velocity and inertia can also be expressed in world coordinates:

\[
\omega_W=R\omega,\qquad J_W=RJR^T,\qquad H_W=J_W\omega_W.
\]

The world-frame tensor \(J_W\) generally changes as the drone rotates. Newton's
rotational law uses the time derivative in the inertial world frame:

\[
M_W=\dot H_W.
\]

Here \(M_W\) is the total applied torque in world coordinates; in the simple
model it comes from the rotors. The same torque in body coordinates is
\(M=R^TM_W\).

### Differentiate, then express the result in body coordinates

Differentiating \(H_W=RJ\omega\), with constant body-frame \(J\), gives

\[
M_W=\dot R J\omega+RJ\dot\omega.
\]

The first term accounts for the changing orientation of the body axes; the
second accounts for changes in the body-frame angular-velocity components.
Using \(\dot R=R[\omega]_\times\), where
\([\omega]_\times a=\omega\times a\), gives

\[
M_W=R\left(\omega\times(J\omega)+J\dot\omega\right).
\]

Multiplying by \(R^T\) expresses the whole equation in body coordinates:

\[
M=R^TM_W
=\omega\times(J\omega)+J\dot\omega.
\]

Equivalently, this is the rotating-frame derivative theorem:

\[
R^T\dot H_W=\dot H_B+\omega\times H_B,
\qquad \dot H_B=J\dot\omega.
\]

The dot on \(H_B\) means the derivative of its body-frame components; the
cross-product term accounts for the rotation of those axes. Solving for the
body-rate derivative gives

\[
\boxed{\dot\omega=J^{-1}\left(M-\omega\times(J\omega)\right)}.
\]

The cross-product term describes the inertial coupling associated with expressing
angular momentum in the rotating body frame. With no
applied torque, \(\dot H_W=0\), but
\(\dot H_B=-\omega\times H_B\) can be nonzero. All quantities in the boxed
equation are in body coordinates; \(\dot\omega\) is the derivative of the
body-axis rates. The relation to Euler-angle rates is given below.

## Component form of Euler's equation

When the body axes are principal axes,

\[
J=\operatorname{diag}(J_x,J_y,J_z).
\]

With \(\omega=(p,q,r_\omega)^T\), Euler's equations become

\[
\dot p=\frac{M_x+(J_y-J_z)q r_\omega}{J_x},
\]

\[
\dot q=\frac{M_y+(J_z-J_x)r_\omega p}{J_y},
\]

\[
\dot r_\omega=\frac{M_z+(J_x-J_y)pq}{J_z}.
\]

The first term in each equation is direct angular acceleration from torque. The
second is inertial coupling between axes. Near hover, angular rates are small and
the products of rates can often be neglected:

\[
\dot p\simeq M_x/J_x,
\qquad
\dot q\simeq M_y/J_y,
\qquad
\dot r_\omega\simeq M_z/J_z.
\]

For a nearly symmetric quadrotor \(J_x\simeq J_y\), direct roll-pitch coupling
in the yaw equation is small. Coupling involving \(J_z\) remains during combined
roll, pitch, and yaw motion.

With zero external torque, angular momentum is constant in the world frame. Its
body-frame components can nevertheless change because the body frame itself is
rotating.

## Orientation kinematics

Orientation evolves according to the kinematic equation

\[
\dot R=R[\omega]_\times,
\]

where

\[
[\omega]_\times=
\begin{bmatrix}
0&-r_\omega&q\\
r_\omega&0&-p\\
-q&p&0
\end{bmatrix}
\]

satisfies

\[
[\omega]_\times a=\omega\times a.
\]

To derive this, take a body-fixed vector \(a_B\). Its world representation is
\(a_W=Ra_B\). Since \(a_B\) is constant in the body,

\[
\dot a_W=\dot R a_B.
\]

The same vector rotates in the world at angular velocity \(R\omega\):

\[
\dot a_W=(R\omega)\times(Ra_B)
=R(\omega\times a_B)
=R[\omega]_\times a_B.
\]

Since this holds for every \(a_B\),

\[
\dot R=R[\omega]_\times.
\]

Numerical integration must preserve \(R^TR=I\) and \(\det R=1\). Practical
implementations commonly propagate a unit [quaternion](glossary.md#quaternion-and-so3),
a four-component representation of orientation, or integrate directly on the
rotation group \(SO(3)\): the matrices satisfying those constraints.

For implementation details:

- Joan Sola, [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508),
  Sections 2.3-2.4 and 4.6: rotation matrices, unit quaternions, exponential maps
  and integration of measured angular rates, including quaternion renormalization.
- Taeyoung Lee, Melvin Leok and N. Harris McClamroch,
  [Lie Group Variational Integrators for the Full Body Problem](https://arxiv.org/abs/math/0508365):
  a more advanced treatment of geometric integration that preserves the rigid-body
  configuration space, including the rotation-matrix constraints.

## Body rates versus Euler-angle rates

The relation between body angular velocity and the derivatives of roll, pitch
and yaw depends on the orientation. For the common ZYX convention,

\[
\begin{bmatrix}
p\\q\\r_\omega
\end{bmatrix}
=
\begin{bmatrix}
1&0&-\sin\theta\\
0&\cos\phi&\sin\phi\cos\theta\\
0&-\sin\phi&\cos\phi\cos\theta
\end{bmatrix}
\begin{bmatrix}
\dot\phi\\\dot\theta\\\dot\psi
\end{bmatrix}.
\]

Only for small roll and pitch angles is the approximation

\[
p\simeq\dot\phi,
\qquad q\simeq\dot\theta,
\qquad r_\omega\simeq\dot\psi
\]

reasonable. Euler-angle coordinates also become singular at pitch
\(\theta=\pm90^\circ\), another reason to use rotation matrices or quaternions
internally.

## Rotor thrust and moment allocation

For rotor \(i\), a common static model is

\[
f_i=k_f\Omega_i^2,
\]

where \(\Omega_i\) is rotor speed. Its reaction torque magnitude is often
approximated by

\[
\tau_i=k_m\Omega_i^2.
\]

Let \(\rho_i\) be the rotor position relative to the centre of mass in body
coordinates and let \(s_i=+1\) or \(-1\) encode its spin direction. Then

\[
f=\sum_{i=1}^4 f_i,
\]

\[
M=\sum_{i=1}^4 \rho_i\times(f_i e_3)
+\sum_{i=1}^4 s_i\tau_i e_3.
\]

The first moment produces roll and pitch torque. The rotor reaction torques
produce yaw torque. In matrix form,

\[
\begin{bmatrix}
f\\M_x\\M_y\\M_z
\end{bmatrix}
=B
\begin{bmatrix}
\Omega_1^2\\\Omega_2^2\\\Omega_3^2\\\Omega_4^2
\end{bmatrix},
\]

where the allocation matrix \(B\) depends on arm geometry, motor numbering,
propeller directions, and sign conventions.

Motor limits constrain the achievable combinations of \(f\) and \(M\). At high
collective thrust there may be little remaining authority for attitude control.

## Banked turns

For a level circular turn of radius \(R_c\) and speed \(v\), the required
horizontal acceleration is

\[
a_c=\frac{v^2}{R_c}.
\]

If the thrust vector is banked by angle \(\phi\), constant altitude requires

\[
f\cos\phi=mg,
\]

while the horizontal component supplies centripetal force:

\[
f\sin\phi=m\frac{v^2}{R_c}.
\]

Dividing these equations gives

\[
\tan\phi=\frac{v^2}{gR_c},
\]

and therefore

\[
\boxed{\phi=\tan^{-1}\left(\frac{v^2}{gR_c}\right)}.
\]

For maximum permitted bank \(\phi_{\max}\), the corresponding turn-speed limit
is

\[
\boxed{v_{\max}=\sqrt{gR_c\tan\phi_{\max}}}.
\]

The required thrust increases to

\[
f=\frac{mg}{\cos\phi}.
\]

Thus aggressive banking consumes thrust reserve and can couple horizontal
tracking to altitude error.

For a commanded translational acceleration \(a_d\), including vertical motion,
the bank angle implied by the thrust vector is

\[
\phi=\tan^{-1}\left(
\frac{\lVert(a_{d,x},a_{d,y})\rVert}{g+a_{d,z}}
\right),
\]

provided \(g+a_{d,z}>0\). Consequently, a bank limit must constrain lateral
acceleration using

\[
\lVert(a_{d,x},a_{d,y})\rVert
\leq (g+a_{d,z})\tan\phi_{\max},
\]

The available lateral acceleration therefore depends on the vertical acceleration.

## Relationship to the repository code

The equation above is the full rigid-body plant model. The race software uses a
deliberately reduced interface at the outer-loop boundary:

- planners produce world-frame position, velocity, and translational
  acceleration references;
- the race acceleration controllers output \(a_d\), which excludes gravity;
- in live full-state operation, Crazyflie firmware forms
  \(m(a_d+ge_3)\), obtains its desired thrust direction and magnitude, and uses
  the onboard geometric attitude loop to track them;
- the supplied simulations approximate that closed inner loop as a first-order
  response from commanded to achieved world-frame acceleration;
- the starter and advanced Python Kalman filters estimate translational state.
  Any acceleration passed to the starter filter must already be
  rotated into the world frame and gravity-compensated. The orientation-aware
  error-state EKF is a stretch goal.

The reduced simulations treat the
firmware attitude controller and motor dynamics as an inner-loop plant. The
bank constraint reconnects that reduced model to thrust-vector feasibility.

Additional controller experiments use position-to-velocity and velocity-to-tilt controllers.
Their outputs are m/s and degrees respectively. The toy velocity-lag
and tilt/drag plants approximate different onboard loop boundaries. Follow the
[optional controller tools](controller_cli_examples.md) for their equations, gain units and
world/body-frame mapping; numerical gains cannot be transferred between layers.

## Relation to IMU measurements

The accelerometer measurement model expresses body-frame
[specific force](glossary.md#specific-force), acceleration relative to free fall, with
bias \(b_a\) and noise \(n_a\):

\[
a_m=R^T(\ddot r+ge_3)+b_a+n_a.
\]

For the ideal thrust-only model,

\[
a_m=\frac{f}{m}e_3+b_a+n_a.
\]

The gyroscope measures

\[
\omega_m=\omega+b_g+n_g.
\]

Consequently, dead reckoning requires attitude to rotate specific force into the
world frame, gravity compensation, and estimates of accelerometer and gyro bias.
An orientation error leaks gravity into horizontal acceleration; even a small
tilt error then produces rapidly growing velocity and position drift.

## Model assumptions and omitted effects

The simple model assumes:

- rigid body and constant mass/inertia;
- thrust exactly along one body axis;
- instantaneous commanded thrust and torque;
- no aerodynamic drag or blade flapping;
- no wind, ground effect, or propeller interaction;
- no motor saturation, delay, or battery dependence;
- no rotor gyroscopic moment;
- centre of mass at the modelled origin.

These omissions should enter as explicit model extensions, bounded disturbances,
or process uncertainty. Increasing process noise can acknowledge model error but
does not repair a biased force, frame, mass, or inertia model.
