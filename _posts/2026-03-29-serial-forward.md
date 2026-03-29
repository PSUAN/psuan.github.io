---
title: "Serial forward kinematics solutions"
short: "Solving forward kinematics problems for serial chains."
math: true
---

# Serial forward kinematics solutions

## Building blocks: 3D Frames

### Description

Let there be a spatial frame $A$.
Its properties in relation to the origin frame $O$ can be represented as:

$$
  \newcommand{\q}{\textbf{q}}
  T_{O, A} = \left[ \bar{p}, \q, \bar{V}, \bar{\omega}, \bar{a}, \bar{\epsilon} \right]
$$

It has the following properties:

| Symbol           | Type                          | Quantity             | Meaning                                  |
| ---------------- | ----------------------------- | -------------------- | ---------------------------------------- |
| $\bar{p}$        | 3D vector                     | length               | position in the origin space             |
| $\q$             | quaternion or rotation matrix | ratio                | orientation in the origin space          |
| $\bar{V}$        | 3D vector                     | velocity             | velocity in the origin space             |
| $\bar{\omega}$   | 3D pseudovector               | angular velocity     | angular velocity in the origin space     |
| $\bar{a}$        | 3D vector                     | acceleration         | acceleration in the origin space         |
| $\bar{\epsilon}$ | 3D pseudovector               | angular acceleration | angular acceleration in the origin space |

### Transform combination

Let there are two frames: $A$ (given in relation to the origin $O$) and $B$ (given in relation to $A$).
Where is $B$ in relation to the origin $O$?

Let's name $A$'s properties $T_{O, A}$ and $B$'s properties $T_{A, B}$.
Then, $T_{O, B}$ can be computed using the following approach:

$$
  T_{O, A} \times T_{A, B} = T_1 \times T_2 = T_r =  T_{O, B}
$$

Resulting position is a sum of the first position vector and the rotated second position vector:

$$
  \bar{p}_r = \bar{p}_1 + \q_1 \circlearrowleft \bar{p}_2
$$

Resulting rotation is just the rotation combination:

$$
  \q_r = \q_1 \times \q_2
$$

Resulting velocity is the sum of velocities in origin space and the cross product of the first angular velocity and the second arm:

$$
  \bar{V}_r = \bar{V}_1
    + \q_1 \circlearrowleft \bar{V}_2
    + \bar{\omega}_1 \times \left( \q_1 \circlearrowleft \bar{p}_2 \right)
$$

Resulting angular velocity is just the sum of angular velocities in the origin space:

$$
  \bar{\omega}_r = \bar{\omega}_1 + \q_1 \circlearrowleft \bar{\omega}_2
$$

Resulting acceleration is the sum of accelerations, taking into account the Coriolis acceleration:

$$
  \bar{a}_r = \bar{a}_1
    + \bar{\epsilon}_1 \times (\q_1 \circlearrowleft \bar{p}_2)
    + \bar{\omega}_1 \times (\bar{\omega}_1 \times (\q_1 \circlearrowleft \bar{p}_2))
    + 2 \bar{\omega}_1 \times (\q_1 \circlearrowleft \bar{V}_2)
    + \q_1 \circlearrowleft \bar{a}_2
$$

Resulting angular acceleration is the sum of angular accelerations in the origin space plus angular velocities cross product:

$$
  \bar{\epsilon}_r = \bar{\epsilon}_1
    + \bar{\omega}_1 \times (\q_1 \circlearrowleft \bar{\omega}_2)
    + \q_1 \circlearrowleft \bar{\epsilon}_2
$$

## Using the building blocks

Let's solve forward kinematics problems for a simple 3-DoM spatial robot.
Let the first revolution joint be aligned with the $Z$ axis and the second and third joints be aligned with the $Y$ axis when the generalized values are set to $0$.
The robot's generalized coordinates are $g_1$, $g_2$ and $g_3$.

Let's introduce some transforms:

From the origin to the first segment start:

$$
  \newcommand{\R}{\text{R}}
  T_1 = \left[
    \bar{0},\
    \R_Z({g_1}),\
    \bar{0},\
    \dot{g}_1 \cdot \bar{Z},\
    \bar{0},\
    \ddot{g}_1 \cdot \bar{Z}
  \right]
$$

From the first segment start to the first segment end:

$$
  T_2 = \left[
    \left[0, 0, l_1\right],\
    \R_Y({g_2}),\
    \bar{0},\
    \dot{g}_2 \cdot \bar{Y},\
    \bar{0},\
    \ddot{g}_2 \bar{Y}
  \right]
$$

From the first segment end to the second segment end:

$$
  T_3 = \left[
    \left[0, 0, l_2\right],\
    \R_Y({g_3}),\
    \bar{0},\
    \dot{g}_3 \cdot \bar{Y},\
    \bar{0},\
    \ddot{g}_3 \bar{Y}
  \right]
$$

From the second segment end to the flange:

$$
  T_4 = \left[
    \left[0, 0, l_2\right],\
    \textbf{1},\
    \bar{0},\
    \bar{0},\
    \bar{0},\
    \bar{0}
  \right]
$$

Upon combining those transforms we get solution for our robot:

$$
  T_r = T_1 \times T_2 \times T_3 \times T_4
$$

For example, robot's position is:

$$
  \bar{p}_r = \begin{bmatrix}
    \cos\left(g_1\right) \left(
      l_2 \sin\left(g_2\right) + l_3 \sin\left(g_2 + g_3\right)
    \right) \\
    \sin\left(g_1\right) \left(
      l_2 \sin\left(g_2\right) + l_3 \sin\left(g_2 + g_3\right)
    \right) \\
    l_1 + l_2 \cos\left(g_2\right) + l_3 \cos\left(g_2 + g_3\right)
  \end{bmatrix}
$$

While its angular velocity is:

$$
  \bar{\omega}_r = \begin{bmatrix}
    -\left(\dot{g}_2 + \dot{g}_3\right) \sin\left(g_{1} \right)\\
    \left(\dot{g}_2 + \dot{g}_3\right) \cos\left(g_{1} \right)\\
    \dot{g}_1
  \end{bmatrix}
$$

And the angular acceleration is:

$$
  \bar{\epsilon}_r = \begin{bmatrix}
    - \ddot{g}_2 {\sin}{\left(g_{1} \right)} - \ddot{g}_3 {\sin}{\left(g_{1} \right)} - \dot{g}_1 \dot{g}_2 {\cos}{\left(g_{1} \right)} - \dot{g}_1 \dot{g}_3 {\cos}{\left(g_{1} \right)}\\
    \ddot{g}_2 {\cos}{\left(g_{1} \right)} + \ddot{g}_3 {\cos}{\left(g_{1} \right)} - \dot{g}_1 \dot{g}_2 {\sin}{\left(g_{1} \right)} - \dot{g}_1 \dot{g}_3 {\sin}{\left(g_{1} \right)}\\
    \ddot{g}_1
  \end{bmatrix}
$$

## Conclusion

Provided approach allows us to compute any serial (or tree-like) mechanism forward kinematics for position, velocity and acceleration.
One does not have to analytically differentiate the forward position kinematics problem or to compute the Jacobian matrix to achieve this result.

The solution is $\mathcal{O}(n)$ for $n$ segments and is easily implemented in code.
