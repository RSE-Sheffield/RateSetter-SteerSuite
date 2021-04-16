#Takes a standard steersuite testcase and adds bags close to the people

import argparse

import xml.etree.ElementTree as ET
import numpy as np

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

def remove_namespace(doc, namespace):
	"""Remove namespace in the passed document in place."""
	ns = u'{%s}' % namespace
	nsl = len(ns)
	for elem in doc.getiterator():
		if elem.tag.startswith(ns):
			elem.tag = elem.tag[nsl:]


def read_in_testcase(infile):
	"""
	Converts an xml testcase into a python element tree object

	:param infile: existing xml steersuite testcase
	:type infile: str

	:return: python class storing the details of the xml testcase
	:rtype: [ReturnType]
	"""

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
	id = 0
	bag_roots = []
	
	for elem in root.findall('ns0:agent', namespaces):

		#add bag tag to all agents
		ET.SubElement(elem, 'ns0:bag').text = 'false'

		#bag agent
		bag_root = ET.Element('ns0:agent')

		#name tag
		agent_name = elem.find('ns0:name', namespaces).text
		bag_name = agent_name + '_bag'
		ET.SubElement(bag_root, 'ns0:name').text = bag_name

		#initial conditions
		location_xyz = elem.find('ns0:initialConditions/ns0:position', namespaces)
		x, y, z = [location_xyz[i].text for i in range(3)]
		# print(x, y, z)
		initial_conditions_elem = ET.SubElement(bag_root, 'ns0:initialConditions')
		ET.SubElement(bag_root, 'ns0:radius').text ="0.2"
		position_elem = ET.SubElement(initial_conditions_elem, 'ns0:position')
		ET.SubElement(position_elem, 'ns0:x').text = str(x)
		ET.SubElement(position_elem, 'ns0:y').text = str(y)
		ET.SubElement(position_elem, 'ns0:z').text = str(float(z) + 0.4)
		direction_elem = ET.SubElement(initial_conditions_elem, 'ns0:direction')
		ET.SubElement(direction_elem, 'ns0:x').text = "0"
		ET.SubElement(direction_elem, 'ns0:y').text = "0"
		ET.SubElement(direction_elem, 'ns0:z').text = "0"
		ET.SubElement(initial_conditions_elem, 'ns0:speed').text = "0"

		#goal target
		goal_root = create_dynamic_goal(id)
		# indent(goal_root)
		# ET.dump(goal_root)
		bag_root.append(goal_root)

		#bag tag
		ET.SubElement(bag_root, 'ns0:bag').text = 'true'

		# indent(bag_root)
		# ET.dump(bag_root)

		bag_roots.append(bag_root)

		#increment id counter
		id = id + 1

	for br in bag_roots:
		root.append(br)

	return root

def create_dynamic_goal(target_id):
	"""
	generates the xml tree info for a dynamic goal target

	:target_id: the id of the agent to follow
	:return: The xmlTree root of the information
	"""
	goal_root = ET.Element('ns0:goalSequence')
	dynamic_elem = ET.SubElement(goal_root, 'ns0:seekDynamicTarget')
	ET.SubElement(dynamic_elem, 'ns0:targetName').text = str(target_id)
	ET.SubElement(dynamic_elem, 'ns0:desiredSpeed').text = '1.33'
	ET.SubElement(dynamic_elem, 'ns0:timeDuration').text = '1000.0'
	return goal_root



if __name__ == "__main__":
	default_agent_radius = 0.9
	default_agents_per_region = 10
	default_agents_per_carriage = 10
	default_platform_depth = 6

	parser = argparse.ArgumentParser(description='Generate new Steersuite xml testcases from originals with bags')
	parser.add_argument('-i','--inputFile', default="../crossing-1.xml",
						help='Test case file to add bags to')
	parser.add_argument('-o','--outputFile', default="",
						help='Name of the output testcase file. Defaults to name $(inputFile)-bag.xml')
	parser.add_argument('-r','--bagProb', type = float, default=1,
						help='Probability of adding a bag to an agent')
	args = parser.parse_args()

	#other handling of the arguements
	if args.outputFile == "":
		args.outputFile = "crossing-1-bag.xml"

	root = add_bags(read_in_testcase(args.inputFile), args.bagProb)
	remove_namespace(root, 'ns0')
	# indent(root)
	# ET.dump(root)
	with open(args.outputFile, 'wb') as f:
		ET.ElementTree(root).write(f)