# Webots_ros2_epuck_mapping-navigation-ArUco

Проект посвящен разработке системы управления мобильным робота с использованием фреймворка ROS2

#Мобильный_робот_e-puck2 #Движение_в_складском_помещении #Распознавание_ArUco-маркеров #Картографирование #Навигация
---

тех.стек **ROS2 (humble)**, **Python**, **Linux 22.04**, **Nav2**, **OpenCV 4.09.0**, **tf-transformation**, **Transforms3d**, **teleop_twist_keyboard**.

Проект был реализован в рамках курсовой работы. Пояснительная записка: [Курсовая Цыгикало.pdf](https://github.com/Gelendwvwvwvein/Webots_ros2_epuck_mapping-navigation-ArUco/files/14027271/default.pdf)

Основные команды:
=====================
1. Запуск основного пакета в режиме картографирования (сцена в webots + визуализация картографии в rviz):
   `ros2 launch webots_ros2_epuck robot_launch.py rviz:=true mapper:=true`
2. Сохранение построенной карты (мой путь из рабочего пространства src/webots_ros2_epuck/resource/map_min): 
    `ros2 run nav2_map_server map_saver_cli -f src/webots_ros2_epuck/resource/map_min`
3. Управление роботом посредством использования телеуправления:
    `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
4. Запуск основного пакета в режиме навигации по посторееной карте (у меня webots_ros2_epuck//resource/map_min.yaml):
   `ros2 launch webots_ros2_epuck robot_launch.py rviz:=true nav:=true map:=src/webots_ros2_epuck/resource/map_min.yaml`
5. Запуск ноды для распознавая ArUco-маркеров:
   `ros2 run ros2_aruco aruco_node`
6. Запуск ноды для навигации по станциям:
    `ros2 run webots_ros2_epuck path`



