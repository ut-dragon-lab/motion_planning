# grasp planning:
## 1. object config publishment
```
$ roslaunch aerial_transportation object.launch (object:=cylinder)
```
## 2. grasp planning
```
$ roslaunch aerial_transportation grasp_planning.launch
```
(please check the grasp_planning.yaml, file record option)
## 3. robot model
```
$ roslaunch aerial_robot_model aerial_robot_model.launch simulation:=false (model:=xxxxx)
```

e.g. model:= hydrusx/hydrux_quad 

# grasp control(test):
## 1. object config publishment
```
$ roslaunch aerial_transportation object.launch (object:=cylinder)
```

## 2. grasp control(test)
```
$ roslaunch aerial_transportation grasp_planning.launch control_test:=true (object:=cylinder)
```

## 3. robot model
```
$ roslaunch aerial_robot_model aerial_robot_model.launch simulation:=false (model:=xxxxx)
```

e.g. model:= hydrusx/hydrux_quad 

# grasp demonstration:

## 1. hydrus aerial_robot bringup launch
```
$ roslaunch hydrus_transform_control hydrusx_bringup.launch (model:=quad)
```

## 2. hydrus grasp planning&control launch

### box: 
```
$ roslaunch aerial_transportation aerial_transportation.launch plugin_name:=hydrus debug:=true
```

### cylinder: 
```
$ roslaunch aerial_transportation aerial_transportation.launch plugin_name:=hydrus debug:=true object:=cylinder
```

### local pc: 

#### 1) mocap for hydrus and object
```
$ roslaunch aerial_transportation mocap_assistant.launch 
```
P.S. please swith the mocap project in your windows PC according to the object(box or cylinder)

#### 2) joy stick

```
$ roslaunch aerial_robot_base joy_stick.launch
```
