# West
A remote interface to ros made with html + js + css.
The project was implemented for the *[ROS Indigo Igloo](http://wiki.ros.org/indigo)* distro of ros,
and developed on *[Ubuntu 14.04](http://releases.ubuntu.com/14.04/)*

# Tools
- To easily interact with the Ros environment we are helped by *[rosbridge](http://wiki.ros.org/rosbridge_suite)*, then install it. Our rosdistro is `indigo`
```
> sudo apt-get install ros-<rosdistro>-rosbridge-server
```

## Setup

- Create a new folder and fork the project
   ```
    > mkdir west
    > cd west/
    > git clone https://github.com/barbosa23/west.git
    ```

- Now create a relative link from your catkin workspace to ros package of west
    ```
    > ln -s west/rosnode/west_tools CATKIN_WORKSPACE_PATH/src/west_tools
    ```

- Finally compile new node
    ```
    > catkin_make
    ```
## Launch

- In ros environment run in three different shell the follw command
	```
    > roscore
    > roslaunch rosbridge_server rosbridge_websocket.launch
    > rosrun west_tools west_tools_node.py
    ```

- And from west directory run, **port_number** is a number between 1024 and 49151
	```
    > python server.py <port_number>
    ```
    

## Usage
- Open your favorite broswer from laptop or smartphone and connect to host that provide your server python, at the port number you chose, so West is running
- West require ip of ros host, and a port number which is usually **9090**, port on which **rosbridge** is listening
- Now have fun launching and killing nodes, or calling services!
