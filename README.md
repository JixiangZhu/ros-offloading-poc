# ros-offloading-poc
A proof of concept implementation of dynamic service offloading using Robot Operating System.

python files in app folder contains the offloading client, offloading manager, publisher and listener. It also contains the two scheduling algorithms, HEFT and CPOP.

tgff folder contains input files for generating task DAGs (Directed acyclic graph).

c++ classes in src folder simulates robots by using ROS nodelets.