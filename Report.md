# FRA532 Lab Report

## Authors 
- 
- Wasupol Hengsritawat 64340500049

To run the command provided in this repository, first clone this repository, then run
```
cd {your_ros_workspace}
colcon build && source install/setup.bash
```

# Part 1: Kinematics of Mobile Robot
## Background Knowledge
### Drive Types
#### Basic Drive (Bicycle Model)
A fundamental method for controlling four-wheeled robots is the **bicycle model**, which approximates the robot as a two-wheeled system with one rear driving wheel and one front steering wheel. This simplified model controls the robot’s **linear velocity** $v_{Rx}$ and **angular velocity** $\omega_{Rz}$ using two key parameters:

 - Rear wheel velocity $v_{Wr}$
 - Front wheel steering angle $\delta$

![alt text](images/part1/Bicycle_Model.jpg)

Given the desired robot linear and angular velocities, the steering angle $\delta$ can be computed using the following relation:

$$
\begin{equation}
    \delta = \arctan{\left(\frac{\omega_{Rz}L}{v_{Rx}}\right)}
\end{equation}
$$

where $L$ is the wheelbase—the distance between the front and rear axles. The angular velocity of the rear wheel is given by:
$$
\begin{equation}
    \omega_{Wr} = \frac{v_{Rx}}{r}
\end{equation}
$$

where $r$ is the radius of the wheel. The linear velocity of the front wheel can be expressed as:

$$
\begin{align*}
    v_{Wf} &= \frac{v_{Rx}}{\lvert{}v_{Rx}\rvert{}}\lvert{\omega_{Rz}} \rvert{} \sqrt{L^2+R^2}
\end{align*}
$$

This can be reformulated to compute the angular velocity of the front wheel as:

$$
\begin{equation}
    \omega_{Wf}=\frac{v_{Rx}}{r\lvert{}v_{Rx}\rvert{}} \sqrt{L^2\omega_{Rz}^2+v_{Rx}^2}
\end{equation}
$$

To apply the bicycle model to a four-wheeled mobile robot, the same steering angle $\delta$ is typically applied to both front wheels. However, this simplification leads to slippage, since the front wheels rotate around parallel axes and do not share a common instantaneous center of rotation. This misalignment causes lateral forces that can reduce motion accuracy, especially during tight turns.

#### Ackermann Drive

**Ackermann Drive** (or *Ackermann Steering Geometry*) is a steering mechanism commonly used in four-wheeled vehicles to ensure that all wheels follow circular paths **without slipping** during a turn.

Unlike differential or bicycle models, Ackermann steering more accurately reflects real car-like motion, where:

- Only the **front wheels** are used for steering
- Each front wheel is turned at a **different angle** during a turn:
  - $\delta_L$: steering angle of the front-left wheel
  - $\delta_R$: steering angle of the front-right wheel

![Ackermann Model](images/part1/Ackermann_Model.jpg)

Given the desired robot linear and angular velocities, the **Ackermann steering angle** $\delta_{\text{Ack}}$ can be calculated using the same relation as in the bicycle model:

$$
\begin{equation}
    \delta_{\text{Ack}} = \arctan\left(\frac{\omega_{Rz}L}{v_{Rx}}\right)
\end{equation}
$$

To maintain a common **instantaneous center of rotation**, the steering angles for the left and right front wheels are derived geometrically as:

$$
\begin{equation}
\delta_L = \arctan\left( \frac{L \tan(\delta_{\text{Ack}})}{L + 0.5B \tan(\delta_{\text{Ack}})} \right), \quad
\delta_R = \arctan\left( \frac{L \tan(\delta_{\text{Ack}})}{L - 0.5B \tan(\delta_{\text{Ack}})} \right)
\end{equation}
$$


Given the actual steering angles of both front wheels, the **effective Ackermann steering angle** can be recovered using:

$$
\begin{equation}
\delta_{\text{Ack}} = \arctan\left( \frac{2 \tan(\delta_L) \tan(\delta_R)}{\tan(\delta_L) + \tan(\delta_R)} \right)
\end{equation}
$$

The angular velocity of the rear wheel follows the same equation as in the bicycle model:

$$
\begin{equation}
\omega_{Wr} = \frac{v_{Rx}}{r}
\end{equation}
$$

## Experiments
### Objectives
1. To develop the system with bicycle model drive and Ackermann drive.
2. To develop the system with single track, double track, and yaw rate odometry methods.
3. To compare the accuracy of robot position estimation using different odometry methods: single-track, double-track, and yaw-rate odometry.
4. To analyze the impact of different drive types—namely, basic drive and Ackermann drive—on the accuracy of robot pose estimation.
5. To identify the optimal combination of drive type and odometry method for accurate robot localization.

### Methodology

1. **Develop a mobile robot simulation** using **Gazebo**. The robot steering angle limit is 30 degrees (0.52 rad) for each front wheel.

2. **Implement a robot drive node** that converts robot twist commands (in the global frame) into individual wheel angular velocities based on the specified drive type:
   - **Basic drive**
   - **Ackermann drive**

   Then, verify the correctness and functionality of the developed node.

3. **Implement an odometry node** to estimate the robot's pose using joint state and IMU data, supporting the following odometry methods:
   - **Single-track**
   - **Double-track**
   - **Yaw-rate**

4. **Run simulation experiments** for each robot drive type:
   - Set the **linear velocity** to **0.5 m/s**.
   - Vary the **angular velocity** for each run: −2.0, −1.0, −0.5, 0.0, 0.5, 1.0, and 2.0 rad/s.
   - For each run, record:
     - The **actual robot pose**
     - The **estimated pose** using each odometry method
     - The **robot joint states**

    Each run can be execute using the command:
    ```
    ros2 launch limo_controller limo_experiment type:={drive_type} v_x:={v_x} w_z:={w_z} time:={time}
    ```
    where `drive_type` can be either "basic" or "ackermann", `v_x` is the desired robot linear velocity, `w_z` is the desired robot angular velocity, and `time` is time in seconds to record experiment data. If `v_x` and `w_z` are both zero, the simulation will endlessly record the data in real-time.

5. **Analyze and compare** the recorded results to evaluate the performance of each **drive–odometry combination**.

### Results
#### Basic Drive
Robot Positions (positions are record in meters)
![Basic Drive Positions](images/part1/full_results/1-Bicycle_Positions.png)
Robot Orientations (orientation are record in radians)
![Basic Drive Orientations](images/part1/full_results/2-Bicycle_Orientations.png)
Robot Joint States (velocities are record in m/s)
![Basic Drive Joint States](images/part1/full_results/3-Bicycle_JointStates.png)

Based on the robot joint states and the resulting paths, the data confirm that the basic drive is correctly implemented. The steering angles of both front wheels are equal, as expected, and the robot follows a trajectory consistent with the given twist command. However, the recorded wheel angular velocities reveal noticeable noise spikes, which indicate **slippage**—a known issue in the bicycle model due to unaligned wheel axes. This slippage is caused by friction forces, especially during turns. Moreover, as the steering angle increases, the degree of slippage also increases, since the distance between the instantaneous centers of rotation of the two front wheels becomes larger.

The **double-track** odometry method demonstrated the **worst performance** among all evaluated methods. This result is expected, as double-track odometry relies on the angular velocities of both front wheels to estimate the robot’s pose. However, due to friction and slippage—particularly in high steering angle scenarios—the front wheel velocities are not reliable. In contrast, the **single-track** and **yaw-rate** odometry methods do not depend on the front wheel angular velocities, making them more robust under these conditions.

#### Ackermann Drive
Robot Positions
![Ackermann Drive Positions](images/part1/full_results/4-Ackermann_Positions.png)
Robot Orientations
![Ackermann Drive Orientations](images/part1/full_results/5-Ackermann_Orientations.png)
Robot Joint States
![Ackermann Drive Joint States](images/part1/full_results/6-Ackermann_JointStates.png)

Compared to the basic drive that uses the bicycle model, the Ackermann drive results in less wheel friction, as evidenced by smoother wheel angular velocity profiles. However, noticeable spike noise appears in the wheel angular velocities when the desired robot angular velocity is −2.0 rad/s or 2.0 rad/s. These cases represent edge conditions, where one of the front wheel steering angles exceeds the predefined steering limit. As a result, the system violates Equation $(5)$, leading to the absence of a unique instantaneous center of rotation. This misalignment causes the robot to slip, introducing noise and instability into the motion.

The **double-track** odometry method continues to perform the **worst** even when using the Ackermann drive, despite the reduction in spike noise in the wheel angular velocities. A possible reason lies in the inherent nature of double-track odometry, which **heavily depends on front wheel velocities and steering angles**—both of which are susceptible to disturbances from friction between the wheels and the ground. Additionally, estimation errors in this method tend to accumulate over time, further reducing its reliability.

These observations suggest that even with Ackermann steering, **slippage cannot be fully eliminated**. Friction and the robot’s inertia—especially during turns—still introduce **centrifugal effects**, which contribute to deviations from the ideal path and reduce odometry accuracy.

#### Comparisons
Paths comparison between basic drive using bicycle model and ackermann drive
![](images/part1/3A-Bicycle_vs_Ackermann_Paths.png)
Positions comparison between basic drive using bicycle model and ackermann drive
![](images/part1/3B-Bicycle_vs_Ackermann_Positions.png)
Orientations comparison between basic drive using bicycle model and ackermann drive
![](images/part1/3C-Bicycle_vs_Ackermann_Orientations.png)
Overall, all odometry methods performed **better with the Ackermann drive** than with the basic drive. This is primarily because Ackermann steering provides **more reliable wheel velocity measurements**, while the basic drive suffers from increased slippage.

Interestingly, however, the double-track odometry method produced lower errors with the basic drive than with the Ackermann drive when the angular velocity command was −0.5 rad/s or 0.5 rad/s. This may be attributed to the simplicity of the bicycle model, which can be more effective at low angular velocities. In such cases, the simplified assumptions of the basic drive may lead to better performance, especially for methods like double-track odometry that heavily rely on accurate wheel velocities and steering angles. Thus, the limitations of the Ackermann model may become more pronounced in low-turn-rate scenarios.

### Conclusions
The symmetricity of the data between the positive robot angular velocity command and negative robot angular velocity command for both basic drive and Ackermann drive suggest that both drive nodes are correctly implemented. While the basic drive—based on the bicycle model—successfully produces the desired motion, it suffers from significant slippage due to equal steering angles on the front wheels and the lack of a shared instantaneous center of rotation. This slippage is reflected in noisy wheel angular velocities, especially at higher steering angles.

Among the three odometry methods evaluated, double-track odometry consistently performed the worst, regardless of the drive type. Its reliance on front wheel velocities and steering angles—both of which are sensitive to friction and slippage—makes it less robust than single-track and yaw-rate odometry, which performed more reliably by avoiding these dependencies.

Switching to the Ackermann drive reduces friction-related noise and improves overall odometry accuracy due to more realistic steering mechanics. However, extreme angular velocity commands (±2.0 rad/s) lead to steering saturation and violation of the geometric model, resulting in residual slippage and instability.

Interestingly, the double-track method performed slightly better under the basic drive for low angular velocities (±0.5 rad/s), likely due to the model’s simplicity and minimal steering distortion under these conditions. This highlights that, in low-turn-rate scenarios, the basic model can occasionally outperform more complex models due to its reduced susceptibility to geometric constraints.

In summary, Ackermann drive combined with single-track or yaw-rate odometry provides the most reliable performance for general cases, but the choice of odometry and drive model should consider expected motion characteristics—especially angular velocity—to balance complexity, reliability, and noise sensitivity.