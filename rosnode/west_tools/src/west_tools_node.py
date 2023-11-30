#!/usr/bin/env python
# license removed for brevity

import rospy
from time import sleep
from west_tools.srv import *
from subprocess import Popen, PIPE

# ----- Globals ----------------------------------------------------------------

# This node's name
NODE_NAME = "west_tools_server"

# Package list cache
ROS_PKGS_CACHE = "/tmp/ros-pkgs-list.west"

# Set of nodes launched by West
# { node_name: { node_exec_name: <str>, package: <str>, process: <class Popen> } }
wnodes = {}

# ----- Utils ------------------------------------------------------------------

def get_active_nodes ():
	"""Query ROS and return a Set of currently active nodes"""
	# Get a list of active ROS nodes
	cmd = [ "rosnode", "list" ]
	nodes = launch_and_communicate_subprocess(cmd).split('\n')

	# We use sets because they support arithmetic difference by simply doing (set1 - set2)
	return set(nodes)

def launch_subprocess (cmd, stdin = None, stdout = PIPE, stderr = PIPE):
	"""Launch a child process and return its (string) output"""
	# Popen executes a child program in a new process
	return Popen( cmd, stdin = stdin, stdout = stdout, stderr = stderr )

def launch_and_communicate_subprocess (cmd, stdin = None, stdout = PIPE, stderr = PIPE):
	"""Launch a child process and return its (string) output"""
	# Popen executes a child program in a new process
	# .communicate() outputs a tuple with (stdout, stderr), avoiding possible buffer deadlocks
	return Popen( cmd, stdin = stdin, stdout = stdout, stderr = stderr ).communicate()[0].strip()

# ----- Services handlers ------------------------------------------------------

def handle_kill_node (req):
	"""Terminate a node and and remove its instance from tracked nodes. Return False when something raises an exception"""
	try:
		# Prepend backslash when missing
		if req.node[0] != '/':
			req.node = '/' + req.node

		# Kill given node
		cmd = [ "rosnode", "kill", req.node ]
		process_kill = launch_subprocess(cmd)

		# TODO necessary?
		# Wait for process termination
		process_kill.wait()
		# Sleep a couple of seconds to guarantee subprocess "join"
		sleep(2)

		# Update wnodes when necessary
		for key in wnodes:
			if key == req.node or wnodes[key]["node_exec_name"] == req.node:
				# Assert node termination
				if wnodes[key]["process"].poll() == None:
					# Kill node if it's still running
					wnodes[key]["process"].kill()
					# Refresh active nodes list
					cmd = [ "rosnode", "cleanup" ]
					launch_subprocess(cmd, stdin = PIPE).communicate("y")
				# Delete dead node
				del wnodes[key]

	# Return False on error
	except Exception as err:
		return KillNodeResponse(False)

	return KillNodeResponse(True)

def handle_node_list (req):
	"""Return a list of all nodes available in the given package"""
    # TODO FIXME must be called multiple times when in web UI, why?
	# Find package-related folders
	cmd = [ "rospack", "find", req.pack ]
	path_rospack = launch_and_communicate_subprocess(cmd)

	cmd = [ "catkin_find", "--first-only", "--without-underlays", "--libexec", req.pack ]
	path_catkin = launch_and_communicate_subprocess(cmd)

	if path_rospack + path_catkin == "":
		return NodeListResponse()

	# Find node files in folders
	cmd = [ "find", "-L", path_rospack, path_catkin, "-type", "f", "-perm", "/a+x", "-not", "-path", "'*/\.*'" ]
	filepaths = launch_and_communicate_subprocess(cmd).split('\n')

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
			packages = launch_and_communicate_subprocess(cmd)
			# Cache a copy of the list
			cache.write(packages)

	return PackListResponse(packages.split('\n'))

def handle_run_node (req):
	"""Launch a new node or try to replace an active node instance. Return True if the process is alive, False otherwise"""

	# Currently active nodes
	nodes_before = get_active_nodes()

	# Launch requested node
	cmd = [ "rosrun", req.pack, req.node ]
	process = launch_subprocess(cmd, stdin = PIPE)

	# Get the newly launched node(s)
	diff_nodes = get_active_nodes() - nodes_before

	# The diff is empty: we just replaced a running instance.
	if len(diff_nodes) == 0:
		# Try to find the instance inside wnodes
		for key in wnodes:
			# TODO possible BUG?
			# TODO when a process replaces a running instance (e.g. turtlesim_node) we check the instance status via poll().
			# TODO is it possible for poll to be called too early, thus preventing the process substitution and leaving us with an untracked process?
			# TODO the new instance, the one we just launched, will be lost in the wind?
			if wnodes[key]["package"] == req.pack and (key == req.node or wnodes[key]["node_exec_name"] == req.node):
				# Check if the process has terminated
				if wnodes[key]["process"].poll() != None:
					# Replace terminated node with new process
					wnodes[key]["process"] = process

	# The diff is not empty: we launched a new node
	else:
		# NOT thread-safe. Other processes may have spawned new nodes in the meantime. These new nodes will end up having the wrong package and node_exec_name.
		# Start tracking newly created nodes
		for key in diff_nodes:
			wnodes[key] = {}
			wnodes[key]["node_exec_name"] = req.node
			wnodes[key]["package"] = req.pack
			wnodes[key]["process"] = process

	return RunNodeResponse( process.poll() == None )

def handle_service_list (req):
	"""Return a list of available services"""

	# Get services list for the given node
	cmd = [ "rosnode", "info", req.node ]
	output = launch_and_communicate_subprocess(cmd)

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

def handle_wnode_input (req):
	"""Send input to node via stdin"""
	# Prepend backslash when missing
	if req.node[0] != '/':
		req.node = '/' + req.node

	# Find target node, if it's stored inside wnode
	for key in wnodes:
		if key == req.node or wnodes[key]["node_exec_name"] == req.node:
			# Check whether the target node is still alive
			if wnodes[key]["process"].poll() == None:
				# Send input
				wnodes[key]["process"].communicate(req.text)
				return WNodeInputResponse(True)
	# No nodes available
	return WNodeInputResponse(False)

def handle_wnode_list (req):
	"""Return a list of nodes spawned by west"""
	# Update wnodes status
	for key in wnodes:
		if wnodes[key]["process"].poll() != None:
			# Refresh nodes list, purging unreachable nodes
			cmd = [ "rosnode", "cleanup" ]
			launch_subprocess(cmd, stdin = PIPE).communicate("y")
			# Delete dead node
			del wnodes[key]
	# Return wnodes list
	return WNodeListResponse(wnodes.keys())

# ----- Main entry point  ------------------------------------------------------

if __name__ == '__main__':

	# Setup this node with a custom name
	rospy.init_node(NODE_NAME)
	
	# Register available services and related callbacks
	rospy.Service('kill_node', KillNode, handle_kill_node)
	rospy.Service('node_list', NodeList, handle_node_list)
	rospy.Service('pack_list', PackList, handle_pack_list)
	rospy.Service('run_node', RunNode, handle_run_node)
	rospy.Service('service_list', ServiceList, handle_service_list)
	rospy.Service('wnode_input', WNodeInput, handle_wnode_input)
	rospy.Service('wnode_list', WNodeList, handle_wnode_list)

	# Spin simply sleeps until shutdown
	rospy.spin()
