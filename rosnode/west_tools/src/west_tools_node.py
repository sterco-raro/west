#!/usr/bin/env python
# license removed for brevity

import rospy
from west_tools.srv import *
from subprocess import Popen, PIPE

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
	if len(nodes) > 1:
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
	if len(services) > 1:
		# take the string that follow each '* ' char
		services = map(lambda x: x.rsplit('* ')[1], services)
	
	# now @services is a list that contains node related services
	return ServiceListResponse(services)

if __name__ == '__main__':
	
	print '\n--- Welcome to West tools server ---\n'
	rospy.init_node('west_tools_server')
	
	# expose west services
	rospy.Service('pack_list', PackList, handle_pack_list)
	rospy.Service('node_list', NodeList, handle_node_list)
	rospy.Service('service_list', ServiceList, handle_service_list)

	print 'Available services \n\t /pack_list \n\t /node_list <package> \n\t /service_list <node>'
	rospy.spin()