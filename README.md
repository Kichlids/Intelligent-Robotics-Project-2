# Intelligent-Robotics-Project-2
CS 4023 Intro to Intelligent Robotics Project 2 Repository
   
# Members   
Kichang Song   
Ethan Vong

## Steps to Run The Project 

## #1 Clone the Github repository in a suitable location with
`git clone https://github.com/Kichlids/Intelligent-Robotics-Project-2.git`   
   
Or unpack the zipped project submitted into a suitable location

## #2 CD into the project workspace
`cd /<Path to project>/Intelligent-Robotics-Project-2/robot-ws`

## #3 Execute Catkin make
`catkin_make`

## #4 Source the setup.bash file
`source devel/setup.bash`

## #5 Execute the main launch file
`roslaunch robot_control robot_control.launch`   
   
When prompted, input coordinates for the robot in the format   
((x,y), (x,y))   
((x,y), (x,y))   
(etc...)   
Press ENTER on an empty line to finish inputing coordinates for the task
      
After the robot finsihes with one task you can enter more coordinates for another task
