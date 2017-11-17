# CountingTargetsWithDrone

Drones are most emergine technology noa a days. I have created an algorithm which count the number of the targets on the floor using drone. 
The drone is counting the targets in real time. So, the biggest challange was how to count the same target once while it is available in more than one picture. 
The drone is taking the pictures while flying and also it's counting the targets simoultaneaously.
# Algorithm

The Drone will take the first picture and count the number of the targets in the picture. It finds the last target of that picture, Drone will take another picture. In this picture last target of the prvious image must be present. Using vector algorithm, I will identify last target of the previous image in the new Image. Using that target I can conclude which target are the new ones. 

# CODE

I have two codes, one for raspberry Pi and another for the drone.
I have plugged the raspberry pi with the drone. USing raspberry pi camera, I took the pictures.
Basic Procedure of The code is Following.

Step 1 => Drone Takes Off
Step 2 => Drone go for 3 meters and Stop.
Step 3 => Raspberry Pi will take the pic and Apply the algorithm on it.
Step 4 => After Counting the targets, it tells the drone to move forward. 
Step 5 => Drone moves forward with the same amount of the distance. 
Step 6 => Drone tells the pi to take the pic again.
Repeat



