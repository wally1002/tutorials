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

![rostopic_pub_single](/home/wally1002/Downloads/Robotics/tutorials/media/rostopic_pub_single.gif)

Edit the linear velocities in x, y directions and observe. As you can see the turtle stopped after sometime, it's because we only sent a single message. Some times we need to send messages continuously. This is where `-r` flag helps us. It means repeatedly. The command structure will be

`rostopic pub -r <frequency of messages> <topic name> <message type> <message>`

![rostopic_pub_rec](/home/wally1002/Downloads/Robotics/tutorials/media/rostopic_pub_rec.gif) 

In the GIF we are sending 1 message per second. We can increase the frequency if needed. This concludes discussion on rostopics. Now let's move turtle to a given location. 

### Move through Code

Now we will write a python code encompassing all the ideas we discussed. I'm assuming little knowledge of python. First we need to create a file in the scripts directory of our tutorials package. 

`cd ~/catkin_ws/src/tutorials/scripts`

`touch control.py`

`chmod +x control.py`

Now you can open the file and edit in your favourite editor. The last command makes your file executable so we can run it as a rosnode. I will paste the whole code here then dissect it to the bits. 

```python
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

class TurtleBot:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is received by the 				 	   subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

    def steering_angle(self, goal_pose):
        """Angle which the turtle needs to rotate to align with the goal position."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
    
    def linear_vel(self, goal_pose, constant=1.5):
        """Velocity as a linear function of the distance to be moved. Velocity = K * Distance"""
        return constant * self.euclidean_distance(goal_pose)

    def angular_vel(self, goal_pose, constant=6):
        """Angular velocity at which the turtle has to rotate to align with goal position"""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move2goal(self):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        goal_pose.x = input("Set your x goal: ")
        goal_pose.y = input("Set your y goal: ")

        distance_tolerance = input("Set your tolerance: ")

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)
            
            self.velocity_publisher.publish(vel_msg)
            
            self.rate.sleep()
            
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.spin()

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass
```

Now let's breakdown the code. 

``` python
#!/usr/bin/env python
```

Above line specifies the python interpreter version for the code.

```python
#import libraries
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
```

rospy is python package for writing code we can connect to ros. geometry_msgs are msg class which contain several msgs. Twist is a velocity msg type containing 6 values which are 3 Linear velocities, 3 angular velocities. Pose message has x, y, theta with x axis, linear velocity, angular velocity. Rest are math functions for calculations. 

```python
class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a unique node (using 						anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel' of type Twist with a queue of 10 msgs.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose' with msg type Pose. self.update_pose is called when a 			    message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)
        # set the pose variable
        self.pose = Pose()
        # set rospy rate which is used for specifing the publishing rate
        self.rate = rospy.Rate(10)
```

Define a class for a turtle with all the functions needed. In the constructor(`__init__`) of the class we define our publishers and subscribers. First we initialise our node using `rospy.init_node` which gives our node a name and unique identifier using `anonymous` flag. Then we define our velocity publisher using `rospy.Publisher(<topic name>, <msg type>, queue_size=)`. Here our topic name is `/turtle1/cmd_vel` which has a msg type `Twist`(try to find out how?). They we define a pose subscriber using `rospy.Subscriber(<topic name>, <msg type>, <callback function>)`. Subscribers have a callback function which is called whenever we receive a new msg to the topic. We define a pose variable to store our pose value. rate stores the frequency at which we will publish our msg, this can be set using `rospy.Rate(<value>)`. 

```python
def update_pose(self, data):
    """Callback function which is called when a new message of type Pose is received by the 				 	   subscriber."""
    # store the pose msg in pose variable
    self.pose = data
    # round of x, y to 4 decimal places
    self.pose.x = round(self.pose.x, 4)
    self.pose.y = round(self.pose.y, 4)
```

The above function is the callback function for the subscriber we have written. It takes the data from the topic, It stores the data in pose variable, then rounds off x, y to 4 decimal places. 

```python
def euclidean_distance(self, goal_pose):
    """Euclidean distance between current pose and the goal."""
    return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

def steering_angle(self, goal_pose):
    """Angle which the turtle needs to rotate to align with the goal position."""
    return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

def linear_vel(self, goal_pose, constant=1.5):
    """Velocity as a linear function of the distance to be moved. Velocity = K * Distance"""
    return constant * self.euclidean_distance(goal_pose)

def angular_vel(self, goal_pose, constant=6):
    """Angular velocity at which the turtle has to rotate to align with goal position"""
    return constant * (self.steering_angle(goal_pose) - self.pose.theta)
```

Let's look at an image first. In the image we can see a goal position in  a X-Y coordinate system. $\theta$ is the steering angle we calculate using the `steering_angle` function. The distance between turtle and goal position is calculated using the `euclidean_distance` function. It's very simple math. 

![goal_turtle](/home/wally1002/Downloads/Robotics/tutorials/media/goal_turtle.png)

Now that we have established the variables we want to use to steer the turtle to the goal let's use them. To control the turtle we used what's called as a Proportional Controller. It will be of form $y = K_p * x$  where $y$ is the variable we want to control and $x$ is the variable we have. $K_p$ is the called the proportional gain. So here $x$ is $\theta$, $d$ for two $y$'s which are $V_x$ and $\omega_z$. This is really intuitive. Let's say if we have a large steering angle then we may want to adjust it quickly, this is what the proportional controller does. If input is large then output is large and vice-versa. So the equations we have are 

$$
\begin{align*}
	V_x = K_{p1} * d \\
	\omega_x = K_{p2} * \theta \\
\end{align*}
$$
The above $V_x$ and $\omega_z$ are calculated using `linear_vel` and `angular_vel` functions. Now that we have these values the only remaining step is publishing these values to the topic `/turtle1/cmd_vel`. 

```python
def move2goal(self):
    """Moves the turtle to the goal."""
    # define a goal position msg
    goal_pose = Pose()

    # Get the input from the user.
    goal_pose.x = input("Set your x goal: ")
    goal_pose.y = input("Set your y goal: ")

    # Please, insert a number slightly greater than 0 (e.g. 0.01).
    distance_tolerance = input("Set your tolerance: ")

    # define a velocity msg which needs to be published
    vel_msg = Twist()

    # If we are not within the distance tolerance limit then apply the controller. 
    while self.euclidean_distance(goal_pose) >= distance_tolerance:

        # Porportional controller.
        # https://en.wikipedia.org/wiki/Proportional_control
        
        # Set the velocities calculated in the above functions. 

        # Linear velocity in the x-axis.
        vel_msg.linear.x = self.linear_vel(goal_pose)
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = self.angular_vel(goal_pose)

        # Publishing our vel_msg 
        self.velocity_publisher.publish(vel_msg)

        # Publish at the desired rate.
        self.rate.sleep()

    # Stopping our robot after the movement is over.
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    self.velocity_publisher.publish(vel_msg)

    # If we press control + C, the node will stop.
    rospy.spin()
```

The above function `move2goal` is the main piece of code which calls all the other functions and steers it to the goal. We take 3 inputs which are `x goal, y goal, tolerance`. Tolerance is the error which we can tolerate, so keep it's value `<0.3`(try other values and see what happens) . If our distance to the goal position is greater than tolerance we have to move, otherwise we break out of the while loop and stop our turtle. Now we will set the velocities $V_x$ and $\omega_z$ to the velocity msg. Then we will publish it using  `velocity_publisher.publish(vel_msg)`. The rate of publish is using `rate.sleep()` using the `rate` variable defined. After breaking out of the while loop we need to stop the turtle so we publish a zero velocity msg. `rospy.spin()` lets us terminate the program using `ctrl + C` in the terminal.  

```python
# this is our main function which will call the other functions. 
if __name__ == '__main__':
    try:
        # Instantiate our Turtle class
        x = TurtleBot()
        # Give it a move goal command.
        x.move2goal()
    # If there is an interruption pass it and continue. 
    except rospy.ROSInterruptException:
        pass
```

This above piece of code is where we call everything we written. So `if __name__ == '__main__':` executes our program. If you want to know more on this watch this [video](https://www.youtube.com/watch?v=sugvnHA7ElY). We instantiate our `TurtleBot` class then call the `move2goal` function. 

![move2goal](/home/wally1002/Downloads/Robotics/tutorials/media/move2goal.gif)

So that is it. This is how we can control a robot in simulator using code. I tried to split everything into modules and asses them individually. So if you have any doubts you can ping me anytime!.



