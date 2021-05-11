# Robotics

## Introduction 

Robotics is a very interesting word in itself. When you google Robot it says "Machine resembling human or replicating some human functions". We human's are ourselves very complex both biologically and psychologically. Yet we want to replicate something we don't fully understand yet. For me this is where Robotics and AI stands out . With an aspiration of unknown magnitude and a limited understanding, researchers set out to unravel the Human form into Robots. 

So I think that should be quite a good intro. I will now move from philosophical things into the technical things. I want these posts to cover a lot of things like *Electronics, ML/ DL, Robotics*. I will start off with ROS(Robot Operating System) and some small simulations, then move on to some other stuff. So let's get going.

## ROS

ROS is a pseudo-operating system created to handle complex robots with many functions. ROS provides us with an interface to design modular systems with different functions. ROS is essentially a communication framework. Why ROS? This was the question I asked myself when I couldn't find my way around ROS. Why? cause ROS lets us communicate between different nodes or codes written in different language for different devices. ROS unifies this process of communication. So let's start with installing ROS. 

![logo_ros](https://github.com/wally1002/tutorials/blob/main/media/logo_ros.jpg?raw=true)

### Installing ROS

These instructions are only for Ubuntu 18.04 LTS operating system. You can dual-boot your system or run a virtual machine but prefer the former. We will be using ROS Melodic-Morenia. All the instructions can be found in official [ROS website](http://wiki.ros.org/melodic/Installation/Ubuntu). 

Open a terminal and copy paste these sequentially and enter the password when needed. You will get to know everything as we move forward.

`sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`

`sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`

`sudo apt update`

`sudo apt install ros-melodic-desktop-full`

`echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc`

`source ~/.bashrc`

`sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential`

`sudo apt install python-rosdep`

`sudo rosdep init`

`rosdep update`

So as the installation is out of the way let's get into core of the ROS. 

### Building Blocks of ROS

ROS is about sending and receiving data. So it is maintained by three main structures, Nodes, Topics and Messages. The below figure illustrates how these are connected. 

![image-20210509212618969](https://github.com/wally1002/tutorials/blob/main/media/core_ros.png?raw=true)

1. ROS NODES : Nodes are simply some code which needs data from other sources or generates data which needs to be sent to other nodes. For example camera is source of images, the raw data from sensor is processed on the camera itself and the image is formed. We now need to send the image to the processor so it can save and modify it. Here camera acts as a Node which sends(publishes) the data(image). 

2. ROS MESSAGES : Messages are nothing but data containers. In ROS we have many different types of messages like int, float, image etc. The image data which we need to send from camera acts as a Message of type Image.

3. ROS TOPICS : Topics are message carriers. So every message we send must go through a topic. So the Image message we need to send must go to a topic of some name(let's say camera_data). So if we need that image we should write down a Node which takes data from this topic. 

This brings us to Packages. Packages are simply a self contained project. Packages can be imported into another package and can be used. So the Nodes you write will be inside a package. Packages are self contained so you cannot nest them or add two packages with same name. Collection of different packages is in turn known as Workspace. So all the packages which we need to be working together should be in same workspace. 

There are many other important terms like Services, Parameters which will come later into the picture, so they can be introduced later. This essentially forms the core of ROS. 

### Creating Workspace and Packages

Let's start by creating a directory . `mkdir` creates a directory and `-p` flag specifies that the directory we create is a parent directory.

`mkdir -p catkin_ws`
Then move inside that directory is using `cd`. Here `cd` means change directory. 

`cd catkin_ws`

Before creating a package we need a directory named `src` in the workspace. This is where our packages are managed. 
`cd catkin_ws`

`mkdir src`

We use `catkin build` to manage our packages. This initialises our workspaces and creates a build and devel space for our packages and build them individually if present. 

`catkin build`

Then we need to source(setup) our workspace and this needs to be done every time we open the terminal. So to decrease our work we but the command into the bashrc file which runs every time we open a terminal

`echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`

![output](https://github.com/wally1002/tutorials/blob/main/media/ws.gif?raw=true)



Now let's create a package with name **tutorials**. For that we use the following command where `catkin_create_pkg` creates the package named `tutorials` with dependencies on `geometry_msgs` and `rospy`. So to create a package we should add the dependencies we require to run the code. Although they can be added and deleted after creating the package by editing `CMakeLists.txt` and `package.xml`.
`cd catkin_ws/src`
`catkin_create_pkg tutorials geometry_msgs rospy`

Now we need a `src` folder in the package created. Then we need to build the package using the `catkin build` similar to the workspace. 
`mkdir src`
`cd ~/catkin_ws`
`catkin build`

So we have our package ready to go. We can create a scripts folder inside your package to hold our codes. 
`cd ~/catkin_ws/src`
`mkdir tutorials/scripts`

![package](https://github.com/wally1002/tutorials/blob/main/media/package.gif?raw=true)

I think this will be it for this post and let's look into a simulator called Turtlesim in the next post. 

