# fast_planner_server
Позволяет использовать fast_planner как ActionServer.

### Запуск 

Способы запуска:  

1. fast-planner + fast_planner_server:  
```bash  
roslaunch fast_planner_server fast_planner_server.launch
```  
2. Чтобы подавать **2D Nav Goal** из RViz надо:  
```bash  
roslaunch fast_planner_server fast_planner_server.launch
```  
И еще запустить клиент, который точки из *move_base_simple/goal* отправляет в fast_planner_server:  
```bash  
rosrun fast_planner_server fast_planner_client_node.py
```  

### Конфигурация

#### **Удержание высоты**  
fast_planner_server может удерживать высоту при отсутствие точек траектории. Удержание высоты и сама высота задаются ROS-параметрами в fast_planner_server.launch:  
```xml  
    <param name="enable_althold" value="True"/>  
    <param name="altitude" value="1.5"/>
```  

#### **Режим пересылки сообщений**  
fast_planner_server может работать в режиме пересылки сообщений - игнорируя цели, их достижимость и ошибки, он будет просто пересылать сообщения из топика *planning/pos_cmd* в топик */mavros/setpoint_position/local*.  
Для установки режима пересылки необходимо отправить сообщение в соответствующий топик:  
```bash  
rostopic pub /fast_planner_server_node/set_remapper_mode std_msgs/Bool "data: true"
```  
*fast_planner_server_node* - имя узла (у вас оно может быть другим!).  
Режим пересылки отключается сообщением со значением *false*.  