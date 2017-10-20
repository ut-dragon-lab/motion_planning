# The planning for gap passing using RRT*, along with pure transformation
## Hydrusx
### pure tranformation
#### quad
$ roslaunch  gap_passing aerial_transformation.launch type:=quad
#### hex
$ roslaunch  gap_passing aerial_transformation.launch type:=hex

### gap passing
#### without transformation
$ roslaunch gap_passing gap_passing_planning.launch transformation:=false
#### with transformation
$ roslaunch gap_passing gap_passing_planning.launch transformation:=true
