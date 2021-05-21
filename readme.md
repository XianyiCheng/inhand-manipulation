
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

### Mex compile problems

If you have `GLIBCXX_3.4.26' not found issue. 

First make sure you can find `GLIBCXX_3.4.26' when you run 
```
strings /usr/lib/x86_64-linux-gnu/libstdc++.so.6 | grep GLIBCXX
```

In the linux terminal, try to 

```
sudo ln -s /usr/lib/x86_64-linux-gnu/libstdc++.so.6  /usr/local/MATLAB/R2020b/bin/glnxa64/../../sys/os/glnxa64/libstdc++.so.6
```

To run always start MATLAB with OpenGL, Type the following at the MATLAB command prompt: 
```
opengl('save','software')
```

Otherwise every time you start MATLAB, you do 

```
matlab -softwareopengl
```




