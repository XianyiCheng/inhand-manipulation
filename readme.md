
### Dependencies:
```
sudo apt-get update
sudo apt-get install libglpk-dev
```
Install eigen3 library. Follow the instructions here: http://eigen.tuxfamily.org/


### How to complie mex file:

If eigen3 is not under "/usr/include/eigen3/" (command below), replace it to be the path where eigen3 is installed. Use *-lglpk* to indicate that glpk library need to be included.

```
mex -g -I"/usr/include/eigen3/" -lglpk cpp/planner.cpp cpp/tree.cpp cpp/sample.cpp cpp/utilities.cpp cpp/contact_kinematics.cpp
```
