#!/usr/bin/env python
# license removed for brevity

import os
import rospy
from time import sleep
from west_tools.srv import *
from subprocess import Popen, PIPE

# ----- Globals ----------------------------------------------------------------

# Package list cache
ROS_PKGS_CACHE = "/tmp/ros-pkgs-list.west"

# Set of nodes launched by West
wnodes = {}

# ----- Utils ------------------------------------------------------------------

def get_active_nodes ():
	"""Query ROS and return a Set of currently active nodes (key: node name, value: ?)"""

	# Get a list of active ROS nodes
	cmd = [ "rosnode", "list" ]
	nodes = launch_subprocess(cmd).split('\n')

	# TODO Later on we'll do (newset - oldset) after launching a new node,
	# FIXME I think we skip packages that spawn multiple nodes (later, inside handle_node_list)
	return set(nodes)

def launch_subprocess (cmd):
	"""Launch a child process and return its (string) output"""
	return Popen( cmd, stdout = PIPE, stderr = PIPE ).communicate()[0].strip()

# ----- Services handlers ------------------------------------------------------

def handle_pack_list (req):
	"""Return a list of all ROS packages in the system"""
	packages = None
	try:
		with open(ROS_PKGS_CACHE, "r") as cache:
			# Try to load cached list
			packages = cache.read()
	except Exception as err:
		with open(ROS_PKGS_CACHE, "w") as cache:
			# Get packages list
			cmd = [ "rospack", "list-names" ]
			packages = launch_subprocess(cmd)
			# Cache a copy of the list
			cache.write(packages)
	return PackListResponse(packages.split('\n'))

def handle_node_list (req):
	"""Return a list of all nodes available in the given package"""
    # FIXME must be called multiple times when in web UI, why?
	# Find package-related folders
	cmd = [ "rospack", "find", req.pack ]
	path_rospack = launch_subprocess(cmd).strip()

	cmd = [ "catkin_find", "--first-only", "--without-underlays", "--libexec", req.pack ]
	path_catkin = launch_subprocess(cmd).strip()

	if path_rospack + path_catkin == "":
		return NodeListResponse()

	# Find node files in folders
	cmd = [ "find", "-L", path_rospack, path_catkin, "-type", "f", "-perm", "/a+x", "-not", "-path", "'*/\.*'" ]
	filepaths = launch_subprocess(cmd).strip().split('\n')

	if len(filepaths) == 0:
		return NodeListResponse([])

	# Extract node names from filepaths
	nodes = []
	splitted = None
	for name in filepaths:
		# Split one time using '/' as separator
		splitted = name.rsplit('/', 1)
		if len(splitted) == 0: continue
		# Store last element
		nodes.append( splitted[ len(splitted) - 1 ] )

	return NodeListResponse(nodes)

def handle_service_list (req):
	# TODO docstring

	# Get services list for the given node
	cmd = [ "rosnode", "info", req.node ]
	output = launch_subprocess(cmd)

	with open("/tmp/culo", "w") as fout:
		fout.write(output)

	# Empty services list
	if "Services: None" in output:
		return ServiceListResponse([])

	# Extract services list from verbose rosnode output
	output = output.split("Services: \n")[1] 		# Discard everything before the line "Services:"
	output = output.split("\n\n")[0].split('\n') 	# Discard everything after the services list (always ends with a '\n\n')

	# Remove the '*' prefix from every service name
	services = map( lambda name: name.rsplit("* ")[1], output )
	# Remove automatically generated services (get_logger_level, set_loggers)
	services = filter( lambda name: True if "logger" not in name else False, services )
	return ServiceListResponse(services)









def handle_run_node (req):
	# get a set of running nodes
	nodes = get_active_nodes()

	# use 'rosrun' command with package and nodes given in service request
	cmd = ['rosrun', req.pack, req.node]
	process = Popen (cmd, stdin = PIPE, stdout = PIPE, stderr = PIPE)
	
	# after new node launch, we require again a set of running nodes
	# and make the difference (set difference) with this and the previous one
	# to get the new entry, this is a new key for wnode
	s = (get_active_nodes() - nodes)
	key = None
	if len(s) == 0 :
		for k in wnodes :
			if wnodes[k].poll() != None :
				key = k
	else :
		key = s.pop()
	# create entry on wnodes with 
	# key : key 	node name
	# value : process 	relative process 
	if key != None :
		wnodes[key] = process

	# return true if process is alive, false otherwise
	return RunNodeResponse(process.poll() == None)

def handle_kill_node (req):
	# we make sure the node name begins with the backslash
	if req.node[0] != '/':
		req.node = '/' + req.node
	# use 'rosnode kill' command to kill node, ros takes care to do this dirty job
	cmd = ['rosnode', 'kill', req.node]
	kill = Popen (cmd, stdout = PIPE, stderr = PIPE)
	# wait util kill process expire
	kill.wait()
	# sleep is necessary because wait doesn't guarantee subprocess join
	sleep(2)
	# check if node killed is a wnode, if it is remove this from wnodes
	# check also if relative process still alive, in this case kill it too
	if req.node in wnodes :
		if wnodes[req.node].poll() == None:
			wnodes[req.node].kill()
			# refresh running node list
			cmd = ['rosnode', 'cleanup']
			Popen (cmd,stdin = PIPE, stdout = PIPE, stderr = PIPE).communicate('y')

		del wnodes[req.node]

	return KillNodeResponse(True)

def handle_wnode_list (req):
	# simply return a list of wnodes keys
	return WNodeListResponse(wnodes.keys())

def handle_wnode_input (req):
	# we make sure the node name begins with the backslash
	if req.node[0] != '/':
		req.node = '/' + req.node
	# check if node is present on wnodes and if it is still alive
	# then is possible to write into its standard input
	if req.node in wnodes :
		if wnodes[req.node].poll() == None :
			wnodes[req.node].stdin.write(req.text)
			return WNodeInputResponse(True)

	return WNodeInputResponse(False)

if __name__ == '__main__':
	
	print '\n--- Welcome to West tools server ---\n'
	rospy.init_node('west_tools_server')
	
	# expose west services
	rospy.Service('pack_list', PackList, handle_pack_list)
	rospy.Service('node_list', NodeList, handle_node_list)
	rospy.Service('service_list', ServiceList, handle_service_list)
	rospy.Service('run_node', RunNode, handle_run_node)
	rospy.Service('kill_node', KillNode, handle_kill_node)
	rospy.Service('wnode_list', WNodeList, handle_wnode_list)
	rospy.Service('wnode_input', WNodeInput, handle_wnode_input)

	print '''Available services :
		\t /pack_list 
		\t /node_list <package>
		\t /service_list <node>
		\t /run_node <node>
		\t /kill_node <node>
		\t /wnode_list
		\t /wnode_input <node> <text>
		'''
	rospy.spin()
