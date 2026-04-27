
## connect
`ssh artem@192.168.0.104`

## password

`Simka200231`

---

## Запуск ВСЬОГО робота 

`cd ~/ros2_ws
ros2 launch robot_bringup robot_auto.launch.py`

---

## ОКРЕМО запуск тільки лідара

`cd ~/ros2_ws
ros2 run robot_bringup ld06_node`

---

## Якщо порт інший

ros2 run robot_bringup ld06_node --ros-args -p port:=/dev/ttyUSB1

---

##Перевірка лідара 

ros2 topic echo /scan (потрібна 3 консоль) 

---
