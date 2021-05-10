## Move the turtle

In the previous post we controlled the turtle using our keys, but we may want to automate this process by writing a piece of code which tells the turtle to move in a specified manner. There are 2 ways of controlling it using code. One is through command line using `rostopic pub` and other is using a python/c++ code. We will start with the command line one.

### Rostopic pub through command line

We have already seen `rostopic list`, `rostopic info`, `rostopic echo`. Adding to that stack of commands will be `rostopic pub`. This command helps us publish the data onto a rostopic from terminal itself. Remember the rostopic which the `teleop` node published in the previous tutorial? It's `/turtle1/cmd_vel`. We will now publish a message directly to this topic from command line. 

`roscore`

New terminal - 

`rosrun turtlesim turtlesim_node`

A small trick is to use tab auto complete in terminal. Type something incompletely and press tab key once, If an unambiguous choice is present it will fill it otherwise it doesn't change anything. If you press the tab key twice you can see all the suggestions. This will be helpful in filling out the `rostopic pub` command. Whenever you see things filling out without me typing I'm using tab key to auto complete. 

`rostopic pub <topic name> <message type> <message>` 

Usually after filling out topic name press tab key twice. It will auto fill message type and message. Then edit the message to your requirements. 

![rostopic_pub_single](/home/wally1002/Downloads/Robotics/media/rostopic_pub_single.gif)

Edit the linear velocities in x, y directions and observe. As you can see the turtle stopped after sometime, it's because we only sent a single message. Some times we need to send messages continuously. This is where `-r` flag helps us. It means repeatedly. The command structure will be

`rostopic pub -r <frequency of messages> <topic name> <message type> <message>`

![rostopic_pub_rec](/home/wally1002/Downloads/Robotics/media/rostopic_pub_rec.gif) 

In the GIF we are sending 1 message per second. We can increase the frequency if needed. This concludes discussion on rostopics. Now let's move turtle to a given location. 

### Move through Code

Now we will write a python code encompassing all the ideas we discussed. I'm assuming little knowledge of python. First we need to create a file in the scripts directory of our tutorials package. 

`cd ~/catkin_ws/src/tutorials/scripts`

`touch control.py`

`chmod +x control.py`

Now you can open the file and edit in your favourite editor. The last command makes your file executable so we can run it as a rosnode. 

