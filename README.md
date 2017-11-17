# Requirements 
Hardware

1.) 3DR Solo Drone
2.) 3DR Solo Drone Controller
3.) Laptop Computer
4.) Raspberry Pi
5.) Raspberry Pi Camera
6.) Targets (Red Coloured Objects)
7.) Measurement Tape
8.) Mobile Phone

Software

1.) Python 2.7
2.) Libraries : Dronekit, OS, Vectors, Numpy, Math
3.) Solo App

Licenses

FAA Unmanned Aircraft Remote Pilot License 107

# Configuratrion

> Here, 3DR Solo Drone Controller provides wifi Network called "SoloLink".
> First of all, Turn On the Controller (3DR Solo Controller). Then Turn on the Laptop Computer.
> Connect Laptop Computer to the Controller's Sololink network.
> Connect and Attach the Raspberry Pi with the drone in a way that the camera pointing downside of the drone carefully. 
> Log In to the Raspberry using SSH. It will ask for the password of th Raspberry Pi. 
SSH pi_name@pi_address
password:*******
> Use SSH Keygen to get rid of the password. Here is the link to do that.
https://help.github.com/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent/
> Now, Log in to the drone the same way we logged in to the Pi.
SSH drone_name@drone_address
password:*******
> Use SSH Keygen to get rid of the password.
https://help.github.com/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent/
> We have two terminals in the Laptop Computer. One is for Pi and another is for the drone.
> we need to connect the raspberry pi with the sololink network. 
> We should set up the wifi configuration of the raspberrry pi in a way that it will connect to the sololink everytime automatically.
Here are the instruction:
https://learn.adafruit.com/adafruits-raspberry-pi-lesson-3-network-setup/setting-up-wifi-with-occidentalis

# Set Up

> Fight is fully Autonomous. So, it totally depends on the GPS.
> Longitude and Latitude of the location must be determined in the script. 
> Ex : If you want to go the East for 3 meters, then 
Current_Lon = Current_Lon + 3 meters(in degrees)
Current_Lat = Current_Lat
> It also affects by wind as well as GPS accuracy. 
> So, to include all the targets in the frame, we need to decide certain height of the drone. 
> To determine the height of the drone, we have to consider following parameters.
1.) Field of View of the Camera
2.) Frame Size
3.) GPS_ERROR
4.) OVERLAP between two consecutive Images
Height = (Frame_Size+GPS_ERROR+OVERLAP)/2 * tan(field of view/2)
> We also need to get the direction of the drone. The direction will be useful in our algorithm.
> The direction can be find using the Yaw parameter of the Drone.

# Procedure

> Get all the required Equipments with you. Turn on the the Controller.
> Connect the laptop computer with the sololink. connect the Raspberry pi with the sololink. The Drone is already connected with the sololink network.
> Log in to the Raspberry Pi
> Log in to the Drone
> Calibrate the drone before flying.
Calibration :
1.) Download the Solo App on the phone.
2.) Connect your phone with the sololink network.
3.) Open the App -> Settings -> Solo > Level Calibration
Follow the instructions on the screen of the phone
4.) Open the App -> Settings -> Solo > Surface Calibration
Follow the instructions on the screen of the phone
5.) now, The drone is ready to go. 
6.) The targets are of Red Colored. They has approximately 7 inch Radius.
7.) Distribute the Targets on the floor. Target distribution should be in the direction where the drone is going to fly. We sshould distribute in such a way that drone can locate the targets on it's way.
8.) We can distribute the targets in several ways. We should make it simpler. Distribute them in one line.
9.) Now, Put the drone Before the Targets.
10.) Run the raspberry.py in RaspberryPi
11.) Run the drone.py in Drone.
