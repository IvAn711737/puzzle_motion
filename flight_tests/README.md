# flight_tests  
Пакет для тестирования качества отработки различных полетных ситуаций:  
1. взлет-посадка (takeoff_test.launch)  
2. зависание в воздухе заданное время (position_hold_test.launch)  
3. полет по точкам (flight_through_points_test.launch)  
4. поворот на заданный угол (rotate_angle_test.launch)  
5. вращение на заданное число кругов (spin_laps_test.launch)

### Взлет-посадка  
```bash  
roslaunch flight_tests takeoff_test.launch altitude:=1.5 ap_type:=0
```  
### Зависание в воздухе заданное время  
```bash  
roslaunch flight_tests position_hold_test.launch hover_time:=3
```  
Время *hover_time* зависания указывается в секундах целым числом.  

### Полет по точкам  
```bash  
roslaunch flight_tests flight_through_points_test.launch tolerance:=0.1 num_points:=0 altitude:=1.5
```  
*tolerance* - точность достижения точки в метрах.  
*num_points* - количество точек, по которым летит дрон. Сами точки задаются в launch-файле. Если *num_points* = 0, то дрон летит по траектории по умолчанию (квадрат со стороной 1м).  
### Поворот на заданный угол  
```bash  
roslaunch flight_tests rotate_angle_test.launch angle:=1.57 ang_vel:=0.8
```  
*angle* - относительный угол поворота - положительный, в радианах.  
*ang_vel* - угловая скорость, может быть больше или меньше нуля.  
### Вращение на заданное число кругов
```bash  
roslaunch flight_tests spin_laps_test.launch laps:=3 ang_vel:=0,8
```  
*laps* - количество кругов - положительное число.  
*ang_vel* - угловая скорость, может быть больше или меньше нуля.  
