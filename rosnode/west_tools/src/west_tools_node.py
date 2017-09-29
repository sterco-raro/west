#!/usr/bin/env python
# license removed for brevity

import rospy
from time import sleep
from west_tools.srv import *
from subprocess import Popen, PIPE

# dictionary that keeps all nodes lauched by West, to which
# is possible forward string as standard input
wnodes = {}

def running_nodes (): # utility
	# obtain list of running nodes
	cmd = ['rosnode', 'list']
	nodes = (Popen (cmd, stdout = PIPE, stderr = PIPE).communicate ()[0]).strip ()
	nodes = nodes.split('\n')

	# now @nodes is a list of running nodes on ros, and we trasform it in a set
	return set(nodes)

def handle_pack_list (req):
	# spawn rospack process to retrive all packages in ros and put the result in a list
	cmd = ['rospack', 'list-names']
	packs = (Popen (cmd, stdout = PIPE, stderr = PIPE).communicate ()[0]).strip ()
	packs = packs.split('\n')

	# now @packs is a list of available package on ros workspace
	return PackListResponse(packs)

def handle_node_list (req):
	# obtain path where to look for nodes
	cmd = ['rospack', 'find', req.pack]
	ros_path = (Popen (cmd, stdout = PIPE, stderr = PIPE).communicate ()[0]).strip ()

	cmd = ['catkin_find', '--first-only', '--without-underlays', '--libexec', req.pack]
	catkin_path = (Popen (cmd, stdout = PIPE, stderr = PIPE).communicate ()[0]).strip ()

	# check if we found at least one valid path, otherwise return a empty list
	if (ros_path + catkin_path) == '':
		return NodeListResponse()

	# spawn find process to retrive a nodes list
	cmd = ['find', '-L', ros_path, catkin_path, '-type', 'f', '-perm', '/a+x', '-not', '-path', '\'*/\.*\'']
	nodes = (Popen (cmd, stdout = PIPE, stderr = PIPE).communicate ()[0]).strip ()
	nodes = (nodes.split('\n')); # obtain an array of string
	# only check on actual lists, not the empty set
	if len(nodes) > 0:
		# take the last string that follow last '/' char
		nodes = map(lambda x: x.rsplit('/', 1)[1], nodes)
		
	# now @nodes is a list that contains the ros nodes names
	return NodeListResponse(nodes)

def handle_service_list (req):
	# obtain relative node service list by 'rosnode info' command
	cmd = ['rosnode', 'info', req.node]
	services = (Popen (cmd, stdout = PIPE, stderr = PIPE).communicate ()[0]).strip ()
	services = ((services.split('Services: \n')[1]).split('\n\n')[0]).split('\n')
	
	# only check on actual lists, not the empty set
	if len(services) > 0:
		# take the string that follow each '* ' char
		services = map(lambda x: x.rsplit('* ')[1], services)
	
	# now @services is a list that contains node related services
	return ServiceListResponse(services)

def handle_run_node (req):
	# get a set of running nodes
	nodes = running_nodes()

	# use 'rosrun' command with package and nodes given in service request
	cmd = ['rosrun', req.pack, req.node]
	process = Popen (cmd, stdin = PIPE, stdout = PIPE, stderr = PIPE)
	
	# after new node launch, we require again a set of running nodes
	# and make the difference (set difference) with this and the previous one
	# to get the new entry, this is a new key for wnode
	s = (running_nodes() - nodes)
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