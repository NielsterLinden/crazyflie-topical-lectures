# Rotations, complex numbers and quaternions

**Further reading.** This is optional mathematical background on orientation.
The optional six-state live helper performs the quaternion-to-world-frame conversion; use the
[recording guide](sensor_recording_contract.md#live-six-state-session-and-replay-fallback)
for its practical frame and calibration checks.

Multiplication can describe a rotation. In a plane, unit complex numbers encode
both the rotation angle and how rotations compose. Unit quaternions extend this
idea to three dimensions, with an axis as well as an angle.

We use right-handed axes, column vectors and active rotations: the vector moves
while the coordinate axes stay fixed. Positive planar angles turn X towards Y.
Angles in the equations are in radians.

## Complex multiplication as a planar rotation

Identify a vector \((x,y)^T\) with the complex number \(z=x+iy\).
Multiplication by \(i\) gives

\[
iz=i(x+iy)=-y+ix,
\qquad
\begin{pmatrix}x\\y\end{pmatrix}
\longmapsto
\begin{pmatrix}-y\\x\end{pmatrix}.
\]

This is a counterclockwise rotation through \(90^\circ\). Applying it twice
gives \(i^2z=-z\), a half-turn; four applications give \(i^4z=z\).

More generally, multiply by

\[
u=\cos\theta+i\sin\theta=e^{i\theta},\qquad |u|=1.
\]

Expanding with the usual complex multiplication rule gives

\[
uz=(x\cos\theta-y\sin\theta)
 +i(x\sin\theta+y\cos\theta).
\]

Thus every planar rotation has a unit-complex representation. A complex number
\(\rho e^{i\theta}\), with \(\rho>0\), additionally scales lengths by \(\rho\).
Choosing \(\rho=1\) preserves lengths.

## The same operation as a matrix

The correspondence is

\[
a+ib\quad\longleftrightarrow\quad
\Phi(a+ib)=\begin{pmatrix}a&-b\\b&a\end{pmatrix}.
\]

For \(a^2+b^2=1\), this becomes

\[
R(\theta)=
\begin{pmatrix}\cos\theta&-\sin\theta\\
\sin\theta&\cos\theta\end{pmatrix},
\qquad R^TR=I,\quad \det R=1.
\]

The multiplication rules agree exactly:

\[
\begin{aligned}
(a+ib)(c+id)&=(ac-bd)+i(ad+bc),\\
\Phi(a+ib)\Phi(c+id)&=
\begin{pmatrix}ac-bd&-(ad+bc)\\ad+bc&ac-bd\end{pmatrix}\\
&=\Phi\big((a+ib)(c+id)\big).
\end{aligned}
\]

In particular,

\[
e^{i\alpha}e^{i\beta}=e^{i(\alpha+\beta)},\qquad
R(\alpha)R(\beta)=R(\alpha+\beta).
\]

## What the group structure expresses

A **group** describes operations that compose associatively, have an identity
and an inverse, and stay within the same set under composition. For rotations:

| Property | Unit complex numbers | Planar rotation matrices |
| --- | --- | --- |
| Composition | Multiplication \(u_2u_1\) | Matrix product \(R_2R_1\) |
| Identity | \(1\) | \(I\) |
| Inverse | \(u^{-1}=\bar u\) | \(R^{-1}=R^T\) |
| Closure | \(|u_2u_1|=1\) | \(R_2R_1\) is a rotation |
| Associativity | \((u_3u_2)u_1=u_3(u_2u_1)\) | Same for matrix products |

The unit complex numbers form the group \(U(1)\); planar rotation matrices form
\(SO(2)\), the special orthogonal group in two dimensions. The map \(\Phi\) is
one-to-one and preserves multiplication: these groups are **isomorphic**.
Angles differing by \(2\pi\) describe the same element in either representation.

Planar rotations also commute: their order can be exchanged. Spatial rotations
have a richer structure because rotations about different axes generally depend
on their order.

## Extending the algebra to three dimensions

A quaternion has one scalar and three imaginary components:

\[
q=w+x\mathbf i+y\mathbf j+z\mathbf k=(w,\mathbf u),
\qquad \mathbf u=(x,y,z)^T.
\]

Hamilton's multiplication rules are

\[
\mathbf i^2=\mathbf j^2=\mathbf k^2=-1,\qquad
\mathbf i\mathbf j=\mathbf k,\quad
\mathbf j\mathbf k=\mathbf i,\quad
\mathbf k\mathbf i=\mathbf j.
\]

Reversing a product of two different imaginary basis elements changes its sign;
for example \(\mathbf j\mathbf i=-\mathbf k\). Equivalently,

\[
(w,\mathbf u)(s,\mathbf v)=
\big(ws-\mathbf u\cdot\mathbf v,\;
 w\mathbf v+s\mathbf u+\mathbf u\times\mathbf v\big).
\]

This multiplication is associative and captures the order dependence needed for
spatial rotations. The conjugate and norm are

\[
q^*=(w,-\mathbf u),\qquad
|q|^2=w^2+\mathbf u\cdot\mathbf u,\qquad
qq^*=|q|^2.
\]

Norms multiply, \(|q_2q_1|=|q_2||q_1|\). Unit quaternions therefore form a group,
with identity \((1,\mathbf0)\) and inverse \(q^{-1}=q^*\).

## Axis, angle and the action on a vector

For a unit rotation axis \(\hat{\mathbf n}\) and angle \(\theta\), choose

\[
q=\left(\cos\frac\theta2,\;
\hat{\mathbf n}\sin\frac\theta2\right).
\]

Represent the vector being rotated by a purely imaginary quaternion
\(V=(0,\mathbf v)\). The rotated vector is obtained by multiplication on both
sides:

\[
V'=qVq^*,\qquad V'=(0,\mathbf v').
\]

Expanding the products gives

\[
\mathbf v'=(w^2-\mathbf u\cdot\mathbf u)\mathbf v
 +2\mathbf u(\mathbf u\cdot\mathbf v)
 +2w(\mathbf u\times\mathbf v).
\]

Substituting the axis-angle expression and using the double-angle identities
produces Rodrigues' rotation formula:

\[
\boxed{\mathbf v'=\cos\theta\,\mathbf v
 +(1-\cos\theta)\hat{\mathbf n}(\hat{\mathbf n}\cdot\mathbf v)
 +\sin\theta\,(\hat{\mathbf n}\times\mathbf v).}
\]

The component along the axis stays fixed, while the perpendicular component
rotates through \(\theta\). The half-angle in \(q\) supplies the full angle
through the two-sided product.

For example, a \(90^\circ\) turn about Z uses
\(q=(1+\mathbf k)/\sqrt2\). It sends \(\mathbf i\) to \(\mathbf j\).
Two such turns have quaternion \(q^2=\mathbf k\) and send \(\mathbf i\) to
\(-\mathbf i\). Four turns give \(q^4=-1\), whose action returns every vector
to itself.

## Mapping quaternions to rotation matrices

The vector formula above is linear in \(\mathbf v\), so it defines a matrix:

\[
R(q)=
\begin{pmatrix}
1-2(y^2+z^2)&2(xy-wz)&2(xz+wy)\\
2(xy+wz)&1-2(x^2+z^2)&2(yz-wx)\\
2(xz-wy)&2(yz+wx)&1-2(x^2+y^2)
\end{pmatrix},\qquad |q|=1.
\]

It satisfies \(R^TR=I\) and \(\det R=1\), placing it in \(SO(3)\).
Every three-dimensional rotation can be represented this way. Composition is
preserved because

\[
q_2(q_1Vq_1^*)q_2^*
 =(q_2q_1)V(q_2q_1)^*,
\qquad
\boxed{R(q_2q_1)=R(q_2)R(q_1).}
\]

With column vectors, the rightmost rotation acts first. Inverse rotations
correspond to \(R(q^*)=R(q)^T\).

Both \(q\) and \(-q\) give the same action:
\((-q)V(-q)^*=qVq^*\). Exactly these two unit quaternions represent each
rotation. This **two-to-one mapping**, or **double cover**, is the key difference
from the one-to-one correspondence between unit complex numbers and planar
rotations. The unit-quaternion group is also called \(SU(2)\) in its equivalent
complex-matrix representation; identifying \(q\) with \(-q\) gives \(SO(3)\).

Around any fixed spatial axis, the half-angles add under quaternion multiplication,
just as complex-number phases add. Allowing the axis to vary introduces the
noncommuting products needed for the full spatial rotation group.

## Connection to the drone measurements

The [optional six-state session](sensor_recording_contract.md#live-six-state-session-and-replay-fallback)
records quaternions in **x,y,z,w** order. The equations here write the scalar
first, **w,x,y,z**. Once placed in the appropriate order, the body-to-world
quaternion gives

\[
\mathbf v_W=R(q)\mathbf v_B.
\]

This rotates accelerometer specific force into world coordinates; adding world
gravity then gives translational acceleration. The [equations of motion
note](quadrotor_equations_of_motion.md#orientation-kinematics) connects orientation
to angular velocity and its integration over time.

For a sequence of recorded orientations, opposite quaternion signs can describe
the same physical attitude. Comparing rotations through \(R(q)\), or choosing
successive signs so that \(q_{k+1}\cdot q_k\geq0\), avoids artificial jumps in
the displayed components. Numerical integration also benefits from restoring
unit norm after each step.

## Further reading

[Joan Sola, *Quaternion kinematics for the error-state Kalman filter*](https://arxiv.org/abs/1711.02508)
develops quaternion algebra, rotation conventions, the rotation group and
orientation integration, with applications to IMU-based estimation.
