# Пример решения хакатона "На автопилоте"

ROS2 пакет [goal_selector](modules/navigation/goal_selector_example) показывает на примере формат решения [хакатона "На автопилоте"](https://navtopilote.dev/ros2-hackathon).

Для запуска примера локально необходимо:

1. Настроить среду
1. Скомпилировать ROS2 пакеты
1. Скомпилировать Gazebo плагины
1. Запустить launch примера

[![Video of Eurobot 2025 in gazebo simulation example](https://img.youtube.com/vi/0wdEeBCH4MU/0.jpg)](https://www.youtube.com/watch?v=0wdEeBCH4MU)

## Настроить среду

Для настройки среды установите [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) и [Gazebo (Ignition) Fortress](https://gazebosim.org/docs/fortress/install_ubuntu/#binary-installation-on-ubuntu).

## Скомпилировать ROS2 пакеты

Соберите модули
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

## Скомпилировать Gazebo плагины

Перейдите в каталог плагина и скомпилируйте.
```
cd ~/software/modules/simulator/plugins/retachable_joint
cmake -B build -S .
cmake --build build

cd ~/software/modules/simulator/plugins/game_manager
cmake -B build -S .
cmake --build build
```

Добавьте их к соответствующему пути Gazebo.
```
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH}":~/software/modules/simulator/plugins/retachable_joint/build
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH}":~/software/modules/simulator/plugins/game_manager/build
```

## Запустить launch примера

```
ros2 launch hackathon_bringup hackathon_launch.py
```

Должны открыться три окна: Rviz2 одного робота, Rviz2 второго робота и Gazebo, где оба робота демонстрируют свою работу.

### Команды launch

* `blue_robot:=False` - не добавлять blue_robot
* `blue_pose_x:=0.275` - задать начальный x blue_robot
* `blue_pose_y:=-0.75` - задать начальный y blue_robot
* `blue_pose_yaw:=1.57` - задать начальный угол blue_robot
* `yellow_robot:=False` - не добавлять yellow_robot
* `yellow_pose_x:=-0.275` - задать начальный x yellow_robot
* `yellow_pose_y:=-0.75` - задать начальный y yellow_robot
* `yellow_pose_yaw:=1.57` - задать начальный угол yellow_robot

