# best grasp form searching :

1. searching launch :
```
$  roslaunch aerial_transportation grasp_form_search.launch 
```
If add ``` test_grasp_flag:= true```, then no search execute, instead, kinematics and statics will start once. And ```file_result_flag:= true ``` willl record the search result which will be used later. Set ```search_file_log_flag``` in /config/grasp_form_search.yaml to true will start result the full search result map.


2. object configuration :
```
$ roslaunch aerial_transportation test_object.launch (object:=cylinder)
```

3. use rviz to cehck 


# play the already search result:
```
$  roslaunch aerial_transportation grasp_form_search.launch object:=convex(cylinder)  play_file_flag:=true
```

# plot full search result:
```
$ roslaunch aerial_transportation grasp_form_search_result_plot.launch object:=cylinder
```
