cd ..
cd ..
cd ..

pwd

catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -ltbb

source devel/setup.bash

roslaunch osk run_osk.launch