## Turtlesim

We now  understand the core ideas discussed in the previous post using a 2D simulator called Turtlesim. We will explore the ideas of topics, messages and nodes. Let's get started. 

![startTsim](https://github.com/wally1002/tutorials/blob/main/media/turtlesim.png?raw=true)

###  Getting started with Turtlesim

Turtlesim is a package which is included in ROS. So to use the Turtlesim we first need to start ROS. To start ROS we use the command `roscore` which starts all necessary processes for ROS. So open a new terminal and type in `roscore`. Then to start the the turtlesim node we use `rosrun turtlesim turtlesim_node` in a new terminal. The structure of this command is such that `rosrun <package name> <node name>`. Here our package is `turtlesim` and node is `turtlesim_node`.  

Run the following commands.

`roscore`

In a new terminal

`rosrun turtlesim turtlesim_node`

![startTsim](https://github.com/wally1002/tutorials/blob/main/media/startTsim.gif?raw=true)

### Move the Turtle!

Now that we have our turtle ready, next step will be to move it. We can control the turtle using our keyboard keys by starting another rosnode named  `turtle_teleop_key`. We need to have two terminals from the previous section running to control the turtle. 

In a new terminal

`rosrun turtlesim turtle_teleop_key`

![teleop](https://github.com/wally1002/tutorials/blob/main/media/teleop.gif?raw=true)

As we can see we can move the turtle seamlessly in the simulator. We could also see that whenever turtle hits a wall it sends a message in the other terminal. And to control the turtle the cursor must be on the 3rd terminal that is where we ran our control node. So what's happening under the hood? How is the turtle moving? Here is where we use RQT graph to know what's going on.

### Understanding RQT Graphs 

RQT graphs give an overview of our nodes, topics and how they are connected. You can start a rqt_graph using simple rosrun command like the previous cases. `rosrun rqt_graph rqt_graph` starts a window where we can see the nodes and topics. 

![rqt_graph](https://github.com/wally1002/tutorials/blob/main/media/rqt_graph.png?raw=true)

Here oval shape means a node and box means a topic. so we used two nodes `turtlesim` and `teleop_turtle`(these we ran using `rosrun` in previous sections). Here blue means it's sending messages to a topic and green meaning it is receiving messages from a topic. the intermediary in the red is our topic. It is taking commands from the `teleop_turtle`(our keyboard keys) and sending it out to the `turtlesim`(simulator). Nodes which send data to a topic are called **Publishers** and which take data are called **Subscribers**. So what is this weird `turtle1/cmd_vel` name for the topic? how to look into the data published into this topic? Let's dive into rostopics...

### Into the rostopics 

So first we may want to list all the rostopics which are running. For this we use `rostopic list`, this gives us a list of rostopics running currently. So now we got the list we may want to see from where a specific topic(here `/turtle1/cmd_vel`) is receiving data and where is it sending. Here we use `rostopic info <topic name>` . For our case this command would be `rostopic info /turtle1/cmd_vel`. 

![rostopic_info](https://github.com/wally1002/tutorials/blob/main/media/rostopic_info.gif?raw=true)

So we now see some weird names in the list of rostopics. `rosout` and `rosout_agg` are created when we run `roscore`. They handle some functionality of ROS which is not very useful for us. The other 3 topics are of importance for us. As we were looking at `/turtle1/cmd_vel` we find out it's info. In the info we can see 3 main sections. Type, Publishers and Subscribers. Here type is the message type of the topic. As we already noted from the `rqt_graph` the publisher is `teleop_turtle` and the subscriber is `turtlesim`.

Now we may want to inspect the data itself which is being published into the topics. For this we have `rostopic echo <topic name>` and in our case it is `rostopic echo /turtle1/cmd_vel`. You can note from the GIF that after we give an input through our keyboard we get ourselves data in `rostopic echo`. The data we see is the velocity of the turtle in 6DOF(Degrees Of Freedom). We gave a command to move the turtle forward we can see it only has linear velocity in x-direction. We will discuss this more later. 

![rostopic_echo](https://github.com/wally1002/tutorials/blob/main/media/rostopic_echo.gif?raw=true)



You may have observed a pattern here. To get anything related to a rostopic we are just using list, info, echo after it. This also applies for nodes, messages and others also. 

In the next post we will look into `rostopic pub`, `/turtle1/pose` topic and write a script to move the turtle to a given position from a python code..............s
