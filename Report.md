# FRA532 Lab Report

## Authors 
- Waritthon Kongnoo 65340500050
- Wasupol Hengsritawat 64340500049

To run the command provided in this repository, first clone this repository into your source folder of your ROS2 workspace, then run
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

### Kinematic Models
#### Single-Track Model
The single-track model, also known as the bicycle model, simplifies a four-wheel vehicle by treating it as if it has a single front wheel and a single rear wheel aligned on a central axis—similar to a bicycle.

![Single Track](images/part1/SingleTrack.png)

According to the figure and by applying the cosine law, we obtain:
$$r^2_{FM} = r_b^2+r_{RM}^2-2r_br_{RM}\cos\left( \frac{\pi}{2} + \beta_R\right)$$
which leads to the non-trivial solution:
$$
\begin{equation}
    r_{RM} = r_b\cos\left( \frac{\pi}{2} + \beta_R\right)+r_{FM}\cos\left(\beta_F-\beta_R\right)
\end{equation}
$$
Next, applying the sine law:
$$r_{FM}=\frac{\sin\left(\frac{\pi}{2}+\beta_R\right)}{\sin\left(\beta_F-\beta_R\right)}r_b$$
Using equation $(8)$, the expression simplifies to:
$$\begin{equation}
r_{FM}=\frac{r_b}{\cos\beta_R(\tan\beta_F-\tan\beta_R)}
\end{equation}$$

Since the angular velocity is given by $v = \omega r_{RM}$​, and assuming $\beta_R=0$, the equation $(9)$ becomes:
$$\begin{equation}
\omega=\frac{v}{r_b}\tan{\beta_F}
\end{equation}$$
The linear velocity of the vehicle is simple obtain by averaging the linear velocity of rear wheels which can be calculated using the angular velocity of each rear wheel

$$\begin{equation}
v=\frac{(\omega_{WrL}+\omega_{WrR})r}{2}
\end{equation}$$

where $r$ is the wheel radius, and $\omega_{WrL}$ and $\omega_{WrR}$ are angular velocity of left and right rear wheels, respectively.

### Double-Track Model
The double-track model extends the single-track (bicycle) model by considering each of the four wheels (front-left, front-right, rear-left, rear-right) individually. This model captures the effect of separate wheel velocities, steering angles, and slip angles, providing a more accurate kinematic representation—especially during turning or dynamic maneuvers.

![Double Track](images/part1/DoubleTrack.png)

According to the figure, given the vehicle twist $[\vec{v},\vec{\omega}]^T$ and the position vector of each wheel contact point $\vec{r}_i$, the velocity at the contact point of each wheel 
𝑖
i is given by:
$$\begin{equation}
    \begin{bmatrix}
        \bar{v}_{i,x}\\
        \bar{v}_{i,y}\\
        0
    \end{bmatrix} = 
    \begin{bmatrix}
        v\cos\beta\\
        v\sin\beta\\
        0
    \end{bmatrix} +
    \begin{bmatrix}
        0\\
        0\\
        \omega
    \end{bmatrix} \times
    \begin{bmatrix}
        r_{i,x}\\
        r_{i,y}\\
        0
    \end{bmatrix}
\end{equation}$$

where $\beta$ is a slip angle. However, only the velocity component aligned with the actual wheel rolling direction—defined by the steering angle $\delta_i$—is relevant. Therefore, the effective rolling velocity at each wheel is:

$$\begin{equation}
    \tilde{v}_i=\bar{v}_{i,x}\cos\delta_i+\bar{v}_{i,y}\sin\delta_i
\end{equation}$$

Substituting Equation $(13)$ into $(12)$, we get:

$$\begin{equation}
    \tilde{v}_i=v\cos(\delta_i-\beta)+\omega\left(r_{i,x}\sin\delta_i-r_{i,y}\cos\delta_i\right)
\end{equation}$$

Given the angular velocity $\omega_i$, of each wheel, the linear velocity of that wheel can be approximated as $v_i=\omega_ir\approx\tilde{v}_i$ where $r$ is a wheel radius. Using the angular velocities and steering angles of the front wheels, the vehicle's angular velocity can be computed as:

$$\begin{equation}
    \omega = \frac{v_1\cos(\delta_2-\beta)-v_2\cos(\delta_1-\beta)}
    {r_{1,x}\sin\delta_1\cos(\delta_2-\beta) - r_{1,y}\cos\delta_1\cos(\delta_2-\beta) - 
     r_{2,x}\sin\delta_2\cos(\delta_1-\beta) + r_{2,y}\cos\delta_2\cos(\delta_1-\beta)}
\end{equation}$$

The vehicle's linear velocity can then be calculated as:

$$\begin{equation}
    v = \frac{r_{1,x}v_2\sin\delta_1 - r_{1,y}v_2\cos\delta_1 - 
              r_{2,x}v_1\sin\delta_2 + r_{2,y}v_1\cos\delta_2}
    {r_{1,x}\sin\delta_1\cos(\delta_2-\beta) - r_{1,y}\cos\delta_1\cos(\delta_2-\beta) - 
     r_{2,x}\sin\delta_2\cos(\delta_1-\beta) + r_{2,y}\cos\delta_2\cos(\delta_1-\beta)}
\end{equation}$$

**Note**: Since the double-track model considers the full state of all four wheels, the linear velocity can alternatively be approximated using the average wheel velocities, as described previously in Equation $(11)$.

### Yaw Rate
The yaw rate kinematic model refers to a formulation where the vehicle's yaw rate can either be directly computed (as in differential drive robots) or measured through onboard perception systems. This yaw rate information is then used for estimating the vehicle's pose and twist.

In this work, the yaw rate of the four-wheeled robot is obtained from an Inertial Measurement Unit (IMU), while the linear velocity is computed using Equation $(11)$.

## Experiments
### Objectives
1. To develop the system with bicycle model drive and Ackermann drive.
2. To develop the system with single track, double track, and yaw rate odometry methods which varied by kinematics models.
3. To compare the accuracy of robot position estimation using different odometry methods: single-track, double-track, and yaw-rate odometry.
4. To analyze the impact of different drive types—namely, basic drive and Ackermann drive—on the accuracy of robot pose estimation.
5. To identify the optimal combination of drive type and odometry method for accurate robot localization.

### Methodology

1. **Develop a mobile robot simulation** using **Gazebo**. The robot steering angle limit is 30 degrees (0.52 rad) for each front wheel.

2. **Implement a robot drive node** that converts robot twist commands (in the global frame) into individual wheel angular velocities based on the specified drive type:
   - **Basic drive**
   - **Ackermann drive**

   Then, verify the correctness and functionality of the developed node.
   The implementation of drive node is located in `limo_controller/scripts/controller/basic` for basic drive and `limo_controller/scripts/controller/ackermann` for Ackermann drive. Note that both kinematic classes are inherited from kinematic class in `limo_controller/scripts/kinematics.py`.

3. **Implement an odometry node** to estimate the robot's pose using joint state and IMU data, supporting the following odometry methods:
   - **Single-track**
   - **Double-track**
   - **Yaw-rate**

   The implementation of odometry node is located in `limo_controller/scripts/odometry.py`. Note that the kinematic class used in the code is located in `limo_controller/scripts/kinematics.py`.

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
    where `drive_type` can be either "basic" or "ackermann", `v_x` is the desired robot linear velocity, `w_z` is the desired robot angular velocity, and `time` is time in seconds to record experiment data. If `v_x` and `w_z` are both zero, the simulation will endlessly record the data in real-time which can be use with ROS2 teleop twist keyboard.

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

**In summary**, Ackermann drive combined with single-track or yaw-rate odometry provides the most reliable performance for general cases, but the choice of odometry and drive model should consider expected motion characteristics—especially angular velocity—to balance complexity, reliability, and noise sensitivity.

# Part 2: Path Tracking Controller
## Background Knowledge

## Experiments 1
### Objectives
1. To implemented the pure pursuit path tracking algorithm
2. To study how the look ahead distance effects the tracking performance

### Methodology
1. Implement pure pursuit controller to be fully compatible with the system developed in Part 1.
2.

## Experiments 2

## Experiments 3
### Objectives
To study and compare the performance of each tracking control technique.

### Methodology
1. Implement each path tracking controller to be fully compatible with the system developed in Part 1. The source code for each controller is located in: `limo_controller/scripts/path_tracking`. The hyperparameters of each controller are manually tuned to yield the most idealistic results (i.e. be able to complete the path smoothly with minimum error) as followed
    - **Pure Pursuit**: 
        - look ahead distance = 0.5 m
        - Linear velocity P Controller: Kp = 1.5
        - Angular velocity P Controller: Kp = 3.0
    - **PID**:
        - Linear velocity PID Controller: Kp = 5.0, Ki = 0.0, Kd = 0.5
        - Angular velocity P Controller: Kp = 2.5
    - **Stanley**
        - Linear velocity = 1.0 m/s
        - k = 1.0

2. Run each tracking algorithm to make the robot—configured with Ackermann steering—follow the reference path defined in `limo_controller/config/path.yaml`. During execution, record:
    - The robot's actual trajectory compared to the reference trajectory
    - The tracking error at each timestep

    The execution of each path tracking algorithm can be performed first using the command to launch the simulation:
    ```
    ros2 launch limo_controller limo_drive.launch.py type:=ackermann
    ```
    Once the simulation initializes successfully, start the path tracking node:
    ```
    ros2 launch limo_controller limo_pathtrack.launch.py tracking:={tracking_method}
    ```
    where `tracking_method` can be one of the following: "pure_pursuit", "pid", or "stanley_control"

3. Analyse the results.

### Results
![Results](images/part2/PathTrackingControllerResults.png)

The pure pursuit, PID, and stanley controller are all capable of tracking the given path. The tracking error distribution among all three controller are the same with $\mu\approx3$ cm and the variance $\sigma^2\approx0.00026$ cm2. The small number of tracking error and its variance suggest that the hyperparameters are tuned and the controller algorithm are implemented correctly. 

### Conclusions
