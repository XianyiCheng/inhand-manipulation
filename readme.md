
How to complie mex file:

You need to first install eigen3 library. If eigen3 is not under "/usr/include/eigen3/" (command below), replace it to be the path where eigen3 is installed.

```
mex -g -I"/usr/include/eigen3/" cpp/planner.cpp cpp/tree.cpp cpp/sample.cpp cpp/utilities.cpp cpp/contact_kinematics.cpp
```
