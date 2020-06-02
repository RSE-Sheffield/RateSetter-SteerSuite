#Creates an .xml file scenario of people boarding and alighting a train
#for info on the merseyrail train layout see en.wikipedia.org/wiki/British_Rail_Class_507

import xml.etree.ElementTree as ET
import argparse
import numpy as np

# parser = argparse.ArgumentParser()
# parser.add_argument("-r", "--radius", help="radius of agents to generate")
# parser.add_argument("-o", "--outputName", default = "merseyrail.xml", help="name of generated file ")
# parser.add_argument("-oh", "--obstacleHeight", default = 1, help="visual height of obstacles")
# args = parser.parse_args()

#prettify output
def indent(elem, level=0):
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

#generate a steersuite xml obstacle, given 2 opposite points on a cuboid	
def make_obstacle(xmlparent, point1, point2):
	obstacle = ET.SubElement(xmlparent, 'obstacle')
	xmin = ET.SubElement(obstacle, 'xmin').text= str(point1[0])
	xmax = ET.SubElement(obstacle, 'xmax').text= str(point2[0])
	ymin = ET.SubElement(obstacle, 'ymin').text= str(point1[1])
	ymax = ET.SubElement(obstacle, 'ymax').text= str(point2[1])
	zmin = ET.SubElement(obstacle, 'zmin').text= str(point1[2])
	zmax = ET.SubElement(obstacle, 'zmax').text= str(point2[2])
	
#create a singlular xml steersuite agent	
def make_agent(xmlparent,):
	print("Todo")
	
#Create an xml steersuite agent region that populates a region with similar parameters	
#regionbounds is a 2-element 2d array of opposite ends of the rectangle
def make_agent_region(xmlparent, num_agents, region_bounds, goal_location, agent_radius):
	agentRegion= ET.SubElement(xmlparent, 'agentRegion')
	numAgents= ET.SubElement(agentRegion, 'numAgents').text = str(num_agents)
	
	#region bounds
	regionBounds= ET.SubElement(agentRegion, 'regionBounds')
	xmin = ET.SubElement(regionBounds, 'xmin').text= str(region_bounds[0][0])
	xmax = ET.SubElement(regionBounds, 'xmax').text= str(region_bounds[1][0])
	ymin = ET.SubElement(regionBounds, 'ymin').text= "0"
	ymax = ET.SubElement(regionBounds, 'ymax').text= "0"
	zmin = ET.SubElement(regionBounds, 'zmin').text= str(region_bounds[0][1])
	zmax = ET.SubElement(regionBounds, 'zmax').text= str(region_bounds[1][1])

	#initial conditions
	initialConditions= ET.SubElement(agentRegion, 'initialConditions')
	direction = ET.SubElement(initialConditions, 'direction')
	random = ET.SubElement(direction, 'random').text= "true"
	radius = ET.SubElement(initialConditions, 'radius').text= str(agent_radius)
	speed = ET.SubElement(initialConditions, 'speed').text= "0"
	
	#goal sequence
	goalSequence= ET.SubElement(agentRegion, 'goalSequence')
	seekStaticTarget = ET.SubElement(goalSequence, 'seekStaticTarget')
	targetLocation = ET.SubElement(seekStaticTarget, 'targetLocation')
	x = ET.SubElement(targetLocation, 'x').text= str(goal_location[0])
	y = ET.SubElement(targetLocation, 'y').text= "0"
	z = ET.SubElement(targetLocation, 'z').text= str(goal_location[1])
	desiredSpeed = ET.SubElement(seekStaticTarget, 'desiredSpeed').text = "1.3"
	timeDuration = ET.SubElement(seekStaticTarget, 'timeDuration').text = "1000.0"

#creates obstacles corresponding to a merseyrail train carriage
#door_locations are the center of the door
#train_origin is the front left corner of the train ( e.g. (0,0) where train extends in the +xyz direction
def make_train_obstacles(xmlroot, train_origin, length_x, length_z, wall_thickness, door_locations, door_width, obstacle_height):
	#make the non-door walls
	train_origin = np.asarray(train_origin)
	wall_1 = np.array([train_origin, train_origin + np.array([wall_thickness,obstacle_height,length_z])]) #left carriage wall
	wall_3 = np.array([train_origin + np.array([length_x,0,0]), train_origin + np.array([length_x + wall_thickness,obstacle_height,length_z])]) # right carriage wall
	wall_2 = np.array([train_origin + np.array([0,0,length_z]), train_origin + np.array([length_x,obstacle_height,length_z+wall_thickness])]) #back side of carrage
	
	# make_obstacle(xmlroot, wall_1[0], wall_1[1])
	make_obstacle(xmlroot, wall_2[0], wall_2[1])
	# make_obstacle(xmlroot, wall_3[0], wall_3[1])
	
	#Make the front wall with door gaps
	left_part = train_origin
	for i in door_locations:
		right_part = train_origin + np.array([i-door_width/2,obstacle_height, wall_thickness])
		make_obstacle(xmlroot, left_part, right_part)
		
		left_part = train_origin + np.array([i+door_width/2,0, 0])
	right_part = train_origin + np.array([length_x,obstacle_height, wall_thickness])
	make_obstacle(xmlroot, left_part, right_part)

def make_hollow_square_obstacle(xmlroot, origin, length_x, length_z, wall_thickness=0.1, obstacle_height=0.3):
	bottom_left = np.array([0,0,0])
	bottom_r = np.array([length_x,0,0])
	top_l = np.array([0,0,length_z])
	top_r = np.array([length_x,0,length_z])
	wall_x = np.array([wall_thickness,obstacle_height,0])
	wall_z = np.array([0,obstacle_height,wall_thickness])

	wall_0 = np.array([ origin + bottom_left, origin + bottom_r + wall_z])
	wall_1 = np.array([ origin + bottom_left, origin + top_l + wall_x])
	wall_2 = np.array([ origin + top_l, origin + top_r + wall_z])
	wall_3 = np.array([ origin + bottom_r, origin + top_r + wall_x])
	make_obstacle(xmlroot, wall_0[0], wall_0[1])
	make_obstacle(xmlroot, wall_1[0], wall_1[1])
	make_obstacle(xmlroot, wall_2[0], wall_2[1])
	make_obstacle(xmlroot, wall_3[0], wall_3[1])


# Creates an xml root and populates it in accordance with steersuite	
def initialize_xml():
	outroot = ET.Element('SteerBenchTestCase')
	header = ET.SubElement(outroot, 'header')
	version = ET.SubElement(header, 'version').text = "1.0"
	name = ET.SubElement(header, 'name').text = "Merseyrail-test"
	worldBounds = ET.SubElement(header, 'worldBounds')
	xmin = ET.SubElement(worldBounds, 'xmin').text= "-100"
	xmax = ET.SubElement(worldBounds, 'xmax').text= "100"
	ymin = ET.SubElement(worldBounds, 'ymin').text= "0"
	ymax = ET.SubElement(worldBounds, 'ymax').text= "0"
	zmin = ET.SubElement(worldBounds, 'zmin').text= "-100"
	zmax = ET.SubElement(worldBounds, 'zmax').text= "100"

	return outroot



if __name__ == "__main__":

	parser = argparse.ArgumentParser(description='Generate Steersuite xml input file for train PTI')
	parser.add_argument('-n','--numPerDoor', type=int, default=5,
                		help='number of people per door to spawn (default: 5')
	parser.add_argument('-r','--radius', type=float, default=0.4,
                    	help='radius of agents in meters (default: 0.4m)')
	parser.add_argument("-oh", "--obstacleHeight", default = 1, help="visual height of obstacles")
	parser.add_argument("-o", "--outputName", default = "merseyrail.xml", help="name of generated file ")

	args = parser.parse_args()

	agent_radius = args.radius
	agents_per_region= args.numPerDoor

	# platform_depth: 5
	# platform_length: 150
	# num_carriages: 3
	# carriage_length: 20
	# carriage_depth: 5

	door_width =  1.5
	train_dims = (20,5) #x,z
	train_wall_thickness = 0.1 
	plat_dims = (100,20+train_dims[1]) #x,z
	plat_origin = np.array([-40,0,-20])


	args = parser.parse_args()

	outroot = initialize_xml()
	
	# station+ rails
	make_hollow_square_obstacle(outroot, plat_origin, plat_dims[0], plat_dims[1])
	#block off rails
	make_obstacle(outroot, np.array([-40,0,0]), np.array([-20,0.1,0]))
	make_obstacle(outroot, np.array([40,0,0]), np.array([60,0.1,0]))


	#tile for multiple trains
	tiling = [-1,0,1]
	for i in tiling:
		offset = np.array([i*train_dims[0],0,0])
		offset2d = np.array([i*train_dims[0],0])
		make_train_obstacles(outroot, offset, train_dims[0], train_dims[1], 0.1, [7, 13], door_width, args.obstacleHeight)
		
		#1 large group
#		make_agent_region(outroot, 20, offset2d + np.array([[0,0],[train_dims[0],-10]]), offset2d+np.array([10,2]), 0.5)
		
		#1 group for each door
		make_agent_region(outroot, agents_per_region, offset2d + np.array([[0,0],[train_dims[0]/2,-20]]), offset2d+np.array([7,0]), agent_radius)
		make_agent_region(outroot, agents_per_region, offset2d + np.array([[train_dims[0]/2,0],[train_dims[0],-20]]), offset2d+np.array([13,0]), agent_radius)

		#alighting
#		make_agent_region(outroot, 10, offset2d + np.array([[0,0],[train_dims[0],train_dims[1]]]), offset2d+np.array([10,-10]), 0.5)
		
	#write xml to file
	outtree = ET.ElementTree(outroot)
	print("writing to " + args.outputName)
	indent(outroot)
	outtree.write(args.outputName)