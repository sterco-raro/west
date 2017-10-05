# West
An easy to use remote interface to ros.  
The gui is made with html + js + css and talks to ros using [roslibjs](http://wiki.ros.org/roslibjs). It is served through a [simple python http server](http://blog.wachowicz.eu).  
Communications with the remote ros machine are done using [rosbridge](http://wiki.ros.org/rosbridge_suite) by launching the websocket server node.  
The project was implemented for ros *[Indigo Igloo](http://wiki.ros.org/indigo)* version and developed on *[Ubuntu 14.04](http://releases.ubuntu.com/14.04/)*

### Setup
- First install rosbridge as it's the main dependency for this project
```
sudo apt-get install ros-<rosdistro>-rosbridge-server
```
- Get the project from git
```
git clone https://github.com/barbosa23/west.git
```
- Create a symbolic link for west as a package in your catkin workspace
```
ln -s /abs/path/to/west/rosnode/west_tools CATKIN_WS_PATH/src/west_tools
```
- Lastly compile sources with catkin
```
catkin_make
```

### Quick Start

##### server
These commands run inside the machine we just set up, it must run a working ros environment.  

- Launch the main ros process
```
roscore
```
- There's a handy script to launch both rosbridge_websocket and west_tools, all output will be in *logs/*
```
bash west-cli start
```
- From the root west folder run the python webserver giving a *port* number as argument (between 1024 and 49151)
```
python server.py <port>
```

##### client
Open your favorite web browser, then

- Visit the url **ip**:**port** where
    + *ip* is the address of the python server and
    + *port* is the port number assigned before
    + example on localhost: 127.0.0.1:9999
- Enter rosbridge ip to connect to ros
- Now have fun launching and killing nodes, or calling services!

#### close
- 
```
bash west-cli check
```
- 
```
bash west-cli stop
```

### Troubleshooting

##### rosnode cannot find west_tools
This is a common problem, after linking west_tools in your workspace ros may not be aware of possible changes, to update the packages list you should run
```
rospack profile
```
