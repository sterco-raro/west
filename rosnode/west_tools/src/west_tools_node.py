#!/usr/bin/env python
# license removed for brevity

import rospy
from west_tools.srv import *
from subprocess import Popen, PIPE

wnodes = []

class WNode():
	"""docstring for WNode"""
	def __init__(self, name, process):
		super(WNode, self).__init__()
		self.name = name
		self.process = process

		

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
	if ((ros_path + catkin_path) == ''):
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
	# obtain relative node service list
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
	# use 'rosrun' command with package and nodes given in service request
	cmd = ['rosrun', req.pack, req.node]
	process = Popen (cmd, stdout = PIPE, stderr = PIPE)
	
	# if is present a node with the same name we delete it
	for e in wnodes:
		if (e.name == req.node)
			wnodes.remove(e)

	# create new WNode and add it to list
	node = WNode(req.node, process)
	wnodes.append(node)

	return node.poll() == None

def handle_kill_node (req):
	# use 'rosnode kill' command to kill requested node, ros takes care to do this dirty job
	cmd = ['rosnode', 'kill', req.node]
	kill = Popen (cmd, stdout = PIPE, stderr = PIPE)
	# wait util kill process exspire
	kill.wait()

	# check if node's process is normally die, else kill it
	for e in wnodes:
		if (e.name == req.node)
			if (e.process.poll() == None)
				e.process.kill()
		# finally we remove this WNode from list
		wnodes.remove()
		return True

	return False

def handle_wnode_list (req):
	return WNodeListResponse(wnodes)

def handle_wnode_input (req):
	# check if node requested is present, and if it is write given text to its standard input
	for e in wnodes:
		if (e.name == req.node)
			if (e.process.poll() == None)
				e.stdinput.write(req.text)
				return True

	return False

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

	print '''Available services
			\n\t /pack_list 
			\n\t /node_list <package>
			\n\t /service_list <node>
			\n\t /run_node <node>
			\n\t /kill_node <node>
			\n\t /wnode_list
			\n\t /wnode_input <node> <text>
		'''
	rospy.spin()