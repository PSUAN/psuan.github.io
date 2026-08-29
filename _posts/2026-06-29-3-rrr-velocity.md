---
title: "3-RRR velocity kinematics"
short: "Developing velocity kinematics for the spherical parallel manipulator. Plain and simple."
math: true
---

# 3-RRR velocity kinematics

## Preamble: position kinematics

The 3-RRR spherical parallel manipulator has three coaxial drive joints that move three arms, each with one passive joint.
The arms hold the orientation platform, which has three passive joint axes meet at the central point - the point acting as the origin of the platform rotation.
The platform's passive joint axes lie in the same plane and the angle between pairs of axes is 120 degrees.

It is considered a common knowledge that the forward kinematics problem for that type of parallel kinematics can not be solved.
The inverse kinematics problem solution is rather simple though.

The solution is the same for every active joint and the corresponding arm.

Let's assign the following indices to the joints:

1. The active joint.
   It is aligned with the global $Z$ axis;
2. Passive joint between arm segments.
   It is rotated 45 degrees and points at the central point;
3. Passive joint that connects the arm and the platform.
   In the home position it corresponds to the local $X$ axis.

We know the target orientation of the platform $R_\text{target}$.
Let us introduce $\bar{w}$ - the direction of the joint 3 in the global space:

$$
  \bar{w} = R \times \begin{bmatrix}
    1 \\ 0 \\ 0
  \end{bmatrix}
$$

Then our active joint angle is

$$
  g = \arccos\left(\frac{w_z}{\sqrt{1 - w_z^2}}\right) + \arctan\left(w_y, w_x\right)
$$

## Velocity kinematics

Upon closer examination we might notice that the value of $g$ depends on two parts:

1. Angle to the horizontal plane (the property encoded in $w_z$);
2. Angle of the rotation around the $Z$ axis (encoded in the values of $w_x$ and $w_y$).

So, the joint velocity $\dot{g}$ may be represented as

$$
  \dot{g} = \dot{g}_1 + \dot{g}_2
$$

Let us introduce the $\beta$ angle - the signed angle between the horizontal plane and the axis $\bar{w}$ of the third passive joint axis.

Then, the second part of $\dot{g}$ would be:

$$
  \dot{g}_2 = \frac{\dot{\beta}}{\sqrt{1 - \tan^2\left({\beta}\right)} \cos^{\frac{3}{2}}\left(\beta\right)}
$$

To compute the $\beta$ one would have to solve the inverse position kinematics and find the axis that is orthogonal to both global $Z$ and $\bar{w}$.
We will leave this as an exercise for the reader.

And the $\dot{g}_1$ value is simply the velocity around the global $Z$ axis.

$$
  \dot{g}_1 = \bar{\omega} \cdot \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}
$$
