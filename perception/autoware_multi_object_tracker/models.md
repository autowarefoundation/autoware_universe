# Models used in this module

## Tracking model

<!-- cspell:ignore CTRV -->

### CTRV model [1]

CTRV model is a model that assumes constant turn rate and velocity magnitude.

- state transition equation

$$
\begin{aligned}
x_{k+1} & = x_{k} + v_{k} \cos(\psi_k) \cdot {d t} \\
y_{k+1} & = y_{k} + v_{k} \sin(\psi_k) \cdot {d t} \\
\psi_{k+1} & = \psi_k + \dot\psi_{k} \cdot {d t} \\
v_{k+1} & = v_{k} \\
\dot\psi_{k+1} & = \dot\psi_{k} \\
\end{aligned}
$$

- jacobian

$$
A = \begin{bmatrix}
1 & 0 & -v \sin(\psi) \cdot dt & \cos(\psi) \cdot dt & 0 \\
0 & 1 & v \cos(\psi) \cdot dt & \sin(\psi) \cdot dt & 0 \\
0 & 0 & 1 & 0 & dt \\
0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

### Kinematic bicycle model [2]

Static bicycle model uses two wheel positions (front and rear) with longitudinal and lateral velocities to represent vehicle motion.
The merit of using this model is that it can handle both longitudinal and lateral motion while maintaining vehicle orientation through wheel base geometry.

![kinematic_bicycle_model](image/kinematic_bicycle_model.png)

- **state variable**
  - rear wheel position( $x_1=x-l_r\cos\psi$, $y_1=y-l\sin\psi$ ), front wheel position( $x_2=x+l_f\cos\psi$, $y_2=y+l_f\sin\psi$ ), longitudinal velocity( $v_{long} = v\cos\beta$ ), and lateral velocity of the front wheel ( $v_{lat}=\frac{l_r+l_f}{l_r}v\sin\beta$ )
  - $[x_{1k}, y_{1k}, x_{2k}, y_{2k}, v_{long,k}, v_{lat,k} ]^\mathrm{T}$
- **Prediction Equation**
  - $dt$: sampling time
  - $l_{wheelbase} = l_r + l_f = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}$ : distance between front and rear wheels
  - $\cos\psi = \frac{x_2 - x_1}{l_{wheelbase}}$, $\sin\psi = \frac{y_2 - y_1}{l_{wheelbase}}$ : vehicle orientation components
  - $\gamma = \frac{\ln(2)}{t_{halflife}}$ : decay constant for lateral velocity

$$
\begin{aligned}
x_{1,k+1} & = x_{1,k} + v_{long,k} \cos\psi \cdot dt \\
y_{1,k+1} & = y_{1,k} + v_{long,k} \sin\psi \cdot dt \\
x_{2,k+1} & = x_{2,k} + v_{long,k} \cos\psi \cdot dt - v_{lat,k} \sin\psi \cdot dt \\
y_{2,k+1} & = y_{2,k} + v_{long,k} \sin\psi \cdot dt + v_{lat,k} \cos\psi \cdot dt \\
v_{long,k+1} & = v_{long,k} \\
v_{lat,k+1} & = v_{lat,k} \cdot e^{-\gamma \cdot dt}
\end{aligned}
$$

- **Jacobian Matrix**

$$
A = \begin{bmatrix}
1 - \frac{v_{long} }{l_{wheelbase}}\cdot dt & 0 & \frac{v_{long}}{l_{wheelbase}}\cdot dt & 0 & \cos\psi \cdot dt & 0 \\
0 & 1 - \frac{v_{long}}{l_{wheelbase}}\cdot dt & 0 & \frac{v_{long}}{l_{wheelbase}}\cdot dt & \sin\psi \cdot dt & 0 \\
-\frac{v_{long}}{l_{wheelbase}}\cdot dt & \frac{v_{lat}}{l_{wheelbase}}\cdot dt & 1 + \frac{v_{long}}{l_{wheelbase}}\cdot dt & -\frac{v_{lat}}{l_{wheelbase}}\cdot dt & \cos\psi \cdot dt & -\sin\psi \cdot dt \\
-\frac{v_{lat}}{l_{wheelbase}}\cdot dt & \frac{v_{long}}{l_{wheelbase}}\cdot dt & \frac{v_{lat}}{l_{wheelbase}}\cdot dt & 1 + \frac{v_{long}}{l_{wheelbase}}\cdot dt & \sin\psi \cdot dt & \cos\psi \cdot dt \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & e^{-\gamma \cdot dt}
\end{bmatrix}
$$

#### remarks on the output

The output twist in the vehicle coordinate system is calculated from the state variables:

- Vehicle center position: $x = \frac{x_1 + x_2}{2}$, $y = \frac{y_1 + y_2}{2}$
- Vehicle yaw: $\psi = tan^{-1}{(\frac{y_2 - y_1}{x_2 - x_1})}$
- Longitudinal velocity: $v_x = v_{long}$
- Lateral velocity: $v_y = v_{lat}\frac{ {l_{wheelbase}}}{l_{f}}$
- Angular velocity: $\omega_z = \frac{v_{lat}}{l_{wheelbase}}$

The lateral velocity decays exponentially over time to model the natural stabilization of vehicle slip motion.

## References

<!-- cspell:ignore Wanielik, Gerd, ICIF -->

[1] Schubert, Robin & Richter, Eric & Wanielik, Gerd. (2008). Comparison and evaluation of advanced motion models for vehicle tracking. 1 - 6. 10.1109/ICIF.2008.4632283.

<!-- cspell:ignore Pfeiffer, Schildbach, Georg, Borrelli, Francesco -->

[2] Kong, Jason & Pfeiffer, Mark & Schildbach, Georg & Borrelli, Francesco. (2015). Kinematic and dynamic vehicle models for autonomous driving control design. 1094-1099. 10.1109/IVS.2015.7225830.
