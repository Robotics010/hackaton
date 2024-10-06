Настройте среду ROS2 с Gazebo.

Соберите модули
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Запустите симулятор и программное обеспечение робота вместе с `goal_selector`
```
ros2 launch hackathon_bringup hackathon_launch.py
```

Робот должен выполнить задачи, реализованные в `goal_selector`

![simulator window](images/simulator_window.png)
