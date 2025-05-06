# Lab 1 Mobile Ackermann

- call ackermann controller and manual move by teleop_twist_keyboard
```bash
ros2 launch limo_controller limo_drive.launch.py
```

- call ackermann controller and path tracking by pid
```bash
ros2 launch limo_controller limo_drive.launch.py tracking:='pid'
```

- call ackermann controller and path tracking by pure_pursuit
```bash
ros2 launch limo_controller limo_drive.launch.py tracking:='pure_pursuit'
```

- call ackermann controller and path tracking by stanley_control
```bash
ros2 launch limo_controller limo_drive.launch.py tracking:='stanley_control'
```

- call basic controller
```bash
ros2 launch limo_controller limo_drive.launch.py type:='basic'
```

