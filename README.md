Assemblability-Analysis
========

A framework of assemblability analysis of 3D Fabrication objects. Users only need to load their pins and sockets, the tool can detect how and where to assemble automatically. If the pair of pin and socket can be assembled, framework will create interactive GUI and a inserting path for users adjusting their objects. If users move objects to much and cause collision of pin and socket, framework will warn users. Also the tool would automatically add screw on the surface of pin and socket when every is done. 

Finally, happly 3D printing!

Requirements
----
* CMake
* libigl
* cgal
* pcl
* Eigen

That's all, just some basic Graphic opensource library

Installation
----
    git ...
    cd path_to_folder
    mkdir build
    cd build
    cmake ..
    make
    
Quick Start
----
There are some parameters should be mentioned.
1. number of females(pins)
2. your workspace(where is your models)
3. number of hard clusters
4. mesh sampling density
5. the smooth indexes of soft clusters for each females.

And there is a Simple test. Plese use them as below
    ./part_match 4 ../models/Simpletest 5 2 0.5 0.08 0.2 0.07 0.15

Architecture
----
1. Calculting convex hull for female and convex hull minus original female to get key surface.
2. Do convex segment on each male. There two step hard clustering based on SDF and soft clustering to smooth the segmentation.
3. Do mesh sampling on female and male basing on the sampling density given by user.
4. Do ICP on female surfaces and males and use Hungarian algoritm to match cooresponding female surface and male.
5. Do PCA to find maximum inertia of matched female and male and align male to female
6. Create interactive GUI for user adjusting.
7. Add screw when user exit GUI and save final objects.

Screenshots
----
![image](http://github.com/Youcunl/Assemblability-Analysis/raw/master/image/input.png)

