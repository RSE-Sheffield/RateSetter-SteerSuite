#Takes a standard steersuite testcase and adds bags close to the people

import argparse
import os
import random

import xml.etree.ElementTree as ET

#the xml namespaces used (https://stackoverflow.com/questions/14853243/parsing-xml-with-namespace-in-python-via-elementtree)
namespaces = {'ns0': 'http://www.magix.ucla.edu/steerbench'}


def indent(elem, level=0):
	"""
	Prettify the xml for dumping
	"""
	i = "\n" + level*"  "
	j = "\n" + (level-1)*"  "
	if len(elem):
		if not elem.text or not elem.text.strip():
			elem.text = i + "  "
		if not elem.tail or not elem.tail.strip():
			elem.tail = i
		for subelem in elem:
			indent(subelem, level+1)
		if not elem.tail or not elem.tail.strip():
			elem.tail = j
	else:
		if level and (not elem.tail or not elem.tail.strip()):
			elem.tail = j
	return elem        

def remove_namespace(doc):
	"""Remove namespace (ns0) in the passed document in place."""

	for elem in doc.getiterator():
		elem.tag = elem.tag[38:]

def read_in_testcase(infile):
	"""
	Converts an xml testcase into a python element tree object

	:param infile: existing xml steersuite testcase
	:type infile: str

	:return: python class storing the details of the xml testcase
	:rtype: [ReturnType]
	"""
	ET.register_namespace('', "http://www.magix.ucla.edu/steerbench")
	tree = ET.parse(infile)
	root = tree.getroot()
	return root

def add_bags(root, add_bag_proba):
	"""
	Adds bags next to each person in.

	:param root: etree description of the testcase
	:type root: ...
	:param add_bag_proba: Probability of adding a bag to a person. Must be between 0 and 1
	:type add_bag_proba: float

	:return: etree of testcase with additional bags
	:rtype: etree
	"""
	bag_roots = []
	
	for (id, elem) in enumerate(root.findall('ns0:agent', namespaces)):

		if(random.uniform(0, 1) < add_bag_proba):

			#bag agent
			bag_root = ET.Element('agent')

			#name tag
			agent_name = elem.find('ns0:name', namespaces).text
			bag_name = agent_name + '_bag'
			ET.SubElement(bag_root, 'name').text = bag_name

			#initial conditions
			location_xyz = elem.find('ns0:initialConditions/ns0:position', namespaces)
			x, y, z = [location_xyz[i].text for i in range(3)]
			# print(x, y, z)
			initial_conditions_elem = ET.SubElement(bag_root, 'initialConditions')
			ET.SubElement(initial_conditions_elem, 'radius').text ="0.2"
			position_elem = ET.SubElement(initial_conditions_elem, 'position')
			ET.SubElement(position_elem, 'x').text = str(x)
			ET.SubElement(position_elem, 'y').text = str(y)
			ET.SubElement(position_elem, 'z').text = str(float(z) + 0.4)
			direction_elem = ET.SubElement(initial_conditions_elem, 'direction')
			ET.SubElement(direction_elem, 'x').text = "0"
			ET.SubElement(direction_elem, 'y').text = "0"
			ET.SubElement(direction_elem, 'z').text = "0"
			ET.SubElement(initial_conditions_elem, 'speed').text = "0"

			#goal target
			# agent_speed = elem.find('ns0:desiredSpeed', namespaces)
			# print(agent_speed)
			goal_root = create_dynamic_goal(id, 1.3)
			# indent(goal_root)
			# ET.dump(goal_root)
			bag_root.append(goal_root)

			#bag tag
			ET.SubElement(bag_root, 'bag').text = 'true'

			# indent(bag_root)
			# ET.dump(bag_root)

			bag_roots.append(bag_root)

			#increment id counter
			# id = id + 1

	for br in bag_roots:
		root.append(br)

	return root

def create_dynamic_goal(target_id, target_speed):
	"""
	generates the xml tree info for a dynamic goal target

	:target_id: the id of the agent to follow
	:return: The xmlTree root of the information
	"""
	goal_root = ET.Element('goalSequence')
	dynamic_elem = ET.SubElement(goal_root, 'seekDynamicTarget')
	ET.SubElement(dynamic_elem, 'targetName').text = str(target_id)
	ET.SubElement(dynamic_elem, 'desiredSpeed').text = str(target_speed + 1)
	ET.SubElement(dynamic_elem, 'timeDuration').text = '1000.0'
	return goal_root

def change_agents(root, radius):
	"""
	Alter values of already existing agents
	"""
	for elem in root.findall('ns0:agent', namespaces):
		initialConditions_elem = elem.find('ns0:initialConditions', namespaces)
		# print(initialConditions_elem)

		#change radius
		if radius:
			# print(initialConditions_elem.find('ns0:radius', namespaces).text)
			initialConditions_elem.find('ns0:radius', namespaces).text = str(radius)
			# print(initialConditions_elem.find('ns0:radius', namespaces).text)
			# print(elem.find('ns0:initialConditions/ns0:radius', namespaces).text)

	return root