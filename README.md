# projekt-ubuntu
Projekt został wykonanty w środowisku ROS2 celem stworzenia prostego interfejsu sterowania robotem mobilmym TurtleBot3 poprzez kliknięcie w punkt na obrazie kamery.

Uruchamianie:
- uruchomić node kamery (ros2 run image_tools cam2image --ros-args  -p burger_mode:=true)
- uruchomić click_node
- uruchomić symulacje TurtleBot3 (ros2 run turtlesim turtlesim_node)

Alternatywny sposob uruchomienia:
- uruchomic plik start.launch.py, ktory uruchamia wszystkie potrzbne wezly i pliki

Wymagana instalacja:
- pakiet ros2 humble
- dodatkowe zależności (git python3-colcon-common-extensions python3-rosdep python3-opencv ros-humble-cv-bridge ros-humble-image-transport ros-humble-usb-cam ros-humble-image-tools)
- repozytorium github.com/pzyla16/projekt-ubuntu
- paczka TurtleBot3

##Aktualny ROS2
![ROS2](https://www.therobotreport.com/wp-content/uploads/2022/05/ros-humble-hawksbill-featured.jpg)
