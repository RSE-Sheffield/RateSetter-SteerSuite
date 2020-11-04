#Creates an .xml file scenario of people boarding and alighting a train
#for info on the merseyrail train layout see en.wikipedia.org/wiki/British_Rail_Class_507
import random
import argparse
import math
import sys

import xml.etree.ElementTree as ET
import numpy as np

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
	if(point1[0] > point2[0] or point1[1] > point2[1]):
		print("obstacle args are incorrect way round")

	obstacle = ET.SubElement(xmlparent, 'obstacle')
	xmin = ET.SubElement(obstacle, 'xmin').text= str(point1[0])
	xmax = ET.SubElement(obstacle, 'xmax').text= str(point2[0])
	ymin = ET.SubElement(obstacle, 'ymin').text= str(point1[1])
	ymax = ET.SubElement(obstacle, 'ymax').text= str(point2[1])
	zmin = ET.SubElement(obstacle, 'zmin').text= str(point1[2])
	zmax = ET.SubElement(obstacle, 'zmax').text= str(point2[2])

def make_obstacle_2d(xmlparent, point1, point2):
	xmin = point1[0] if (point1[0] < point2[0]) else point2[0]
	zmin = point1[1] if (point1[1] < point2[1]) else point2[1]
	xmax = point2[0] if (point1[0] < point2[0]) else point1[0]
	zmax = point2[1] if (point1[1] < point2[1]) else point1[1]

	if(point1[0] > point2[0] or point1[1] > point2[1]):
		print("obstacle args are incorrect way round")
	obstacle = ET.SubElement(xmlparent, 'obstacle')
	xmin = ET.SubElement(obstacle, 'xmin').text= str(point1[0])
	xmax = ET.SubElement(obstacle, 'xmax').text= str(point2[0])
	ymin = ET.SubElement(obstacle, 'ymin').text= "0"
	ymax = ET.SubElement(obstacle, 'ymax').text= "0.5"
	zmin = ET.SubElement(obstacle, 'zmin').text= str(point1[1])
	zmax = ET.SubElement(obstacle, 'zmax').text= str(point2[1])

	
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
	
	make_obstacle(xmlroot, wall_1[0], wall_1[1])
	make_obstacle(xmlroot, wall_2[0], wall_2[1])
	make_obstacle(xmlroot, wall_3[0], wall_3[1])
	
	#Make the front wall with door gaps
	left_part = train_origin
	for i in door_locations:
		right_part = train_origin + np.array([i-door_width/2,obstacle_height, wall_thickness])
		make_obstacle(xmlroot, left_part, right_part)
		
		left_part = train_origin + np.array([i+door_width/2,0, 0])
	right_part = train_origin + np.array([length_x,obstacle_height, wall_thickness])
	make_obstacle(xmlroot, left_part, right_part)


# valid goal types are: "statictarget" nad "boxregion"
def add_goal(goal_sequence_xml, goal_dict ):
	goal_type = goal_dict["goal_type"]
	if(goal_type == "statictarget"):
		goal_location = goal_dict["goal_location"]
		seekStaticTarget = ET.SubElement(goal_sequence_xml, 'seekStaticTarget')
		targetLocation = ET.SubElement(seekStaticTarget, 'targetLocation')
		x = ET.SubElement(targetLocation, 'x').text= str(goal_location[0])
		y = ET.SubElement(targetLocation, 'y').text= "0"
		z = ET.SubElement(targetLocation, 'z').text= str(goal_location[1])
		desiredSpeed = ET.SubElement(seekStaticTarget, 'desiredSpeed').text = "1.3"
		timeDuration = ET.SubElement(seekStaticTarget, 'timeDuration').text = "1000.0"
	elif(goal_type == "boxregion"):
		goal_region = goal_dict["goal_region"]
		goal_location = [0,0]
		seekAxisAlignedBoxRegion = ET.SubElement(goal_sequence_xml, 'seekAxisAlignedBoxRegion')
		targetLocation = ET.SubElement(seekAxisAlignedBoxRegion, 'targetLocation')
		x = ET.SubElement(targetLocation, 'x').text= str(goal_location[0])
		y = ET.SubElement(targetLocation, 'y').text= "0"
		z = ET.SubElement(targetLocation, 'z').text= str(goal_location[1])
		desiredSpeed = ET.SubElement(seekAxisAlignedBoxRegion, 'desiredSpeed').text = "1.3"
		timeDuration = ET.SubElement(seekAxisAlignedBoxRegion, 'timeDuration').text = "1000.0"
		goalRegionBounds = ET.SubElement(seekAxisAlignedBoxRegion, 'goalRegionBounds')
		xmin = ET.SubElement(goalRegionBounds, 'xmin').text= str(goal_region[0])
		xmax = ET.SubElement(goalRegionBounds, 'xmax').text= str(goal_region[1])
		ymin = ET.SubElement(goalRegionBounds, 'ymin').text= "0"
		ymax = ET.SubElement(goalRegionBounds, 'ymax').text= "0"
		zmin = ET.SubElement(goalRegionBounds, 'zmin').text= str(goal_region[2])
		zmax = ET.SubElement(goalRegionBounds, 'zmax').text= str(goal_region[3])
	else:
		print("unknown goal type attempting to be added")


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
	# make_obstacle(xmlroot, wall_0[0], wall_0[1])
	make_obstacle(xmlroot, wall_1[0], wall_1[1])
	make_obstacle(xmlroot, wall_2[0], wall_2[1])
	make_obstacle(xmlroot, wall_3[0], wall_3[1])

#create a singlular xml steersuite agent	
def make_agent(xmlparent, radius, location, goals_arr):
	agent= ET.SubElement(xmlparent, 'agent')

	#initial conditions
	initialConditions= ET.SubElement(agent, 'initialConditions')
	direction = ET.SubElement(initialConditions, 'direction')
	direction_x = ET.SubElement(direction, 'x').text = "0"
	direction_y = ET.SubElement(direction, 'y').text = "0"
	direction_z = ET.SubElement(direction, 'z').text = "0"
	position = ET.SubElement(initialConditions, 'position')
	position_x = ET.SubElement(position, 'x').text = str(location[0])
	position_y = ET.SubElement(position, 'y').text = "0"
	position_z = ET.SubElement(position, 'z').text = str(location[1])
	radius = ET.SubElement(initialConditions, 'radius').text= str(radius)
	speed = ET.SubElement(initialConditions, 'speed').text= "0"

	#goal sequence
	goalSequence= ET.SubElement(agent, 'goalSequence')
	for g in goals_arr:
		add_goal(goalSequence, g)

# each agent is given a "box" within the larger box, where it can spawn at some point within.
# This helps maintain randomness each run
# returns number of agents that couldnt fit
def make_manual_agents_in_square(xmlroot, origin, lengths, num_agents, radius, goals):
	# Rows in x dir, columns across z
	
	# Can we fit all the agents?
	nums_per_side = [math.floor(abs(lengths[0]/(radius*2))), math.floor(abs(lengths[1]/(radius*2)))]
	total_num = int(nums_per_side[0] * nums_per_side[1])
	leftover = num_agents - total_num
	num_agents = total_num if leftover > 0 else num_agents

	# calculate the boxes available
	box_lengths = lengths / nums_per_side
	box_occupancy = list(range(0,total_num))
	box_size = box_lengths
	box_center = box_size / 2
	extra_space = np.abs(box_size - (2*radius)) / 2
	# print("nums per side")
	# print(nums_per_side)
	# print("box_size")
	# print(box_size)
	# print("origin: ")
	# print(origin)

	for i in range(num_agents):
		# select a free box
		boxId = random.choice(box_occupancy)

		# choose some space in the box
		offset_range = 0.0
		x_off = random.uniform(0, offset_range) * 2 - offset_range
		y_off = random.uniform(0, offset_range) * 2 - offset_range

		# convert boxId to real location in x-y and create
		row_id = math.floor(boxId / nums_per_side[0])
		colum_id = boxId % nums_per_side[0]
		col_loc = origin[0] + (colum_id * box_size[0]) + box_center[0] + x_off#+ (-extra_space[1] + random.uniform(0,2) * extra_space[1])
		row_loc = origin[1] + (row_id   * box_size[1]) + box_center[1] + y_off#+ (-extra_space[0] + random.uniform(0,2) * extra_space[0])
		make_agent(xmlroot, radius, [col_loc, row_loc], goals)

		# Set id as occupied
		box_occupancy.remove(boxId)

	return leftover

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

def add_corridor(xmlparent, origin, width, height):
	# Add obstacles
	make_obstacle_2d(xmlparent, origin + np.array([-0.1,-height]), origin)
	make_obstacle_2d(xmlparent, origin + np.array([width,-height]), origin + np.array([width+0.1,0]))
	make_obstacle_2d(xmlparent, origin + np.array([0,-height]), origin + np.array([width,-height+0.1]))


def generate_xml(radius, agents_per_region, agents_in_carriage, platform_depth, outputName):
	door_width =  1.5
	train_dims = (20,5) #x,z
	train_wall_thickness = 0.1 
	plat_dims = ([100,platform_depth]) #x,z
	plat_origin = np.array([-40,0,-platform_depth])
	

	outroot = initialize_xml()

	# Location of doors (for goal purposes)
	# door_goals = []
	
	# station+ rails
	make_hollow_square_obstacle(outroot, plat_origin, plat_dims[0], plat_dims[1] + train_dims[1])
	make_obstacle(outroot, np.array([-40,0,-platform_depth]), np.array([-20,0.1,-platform_depth]))
	make_obstacle(outroot, np.array([40,0,-platform_depth]), np.array([60,0.1,-platform_depth]))

	#block off rails
	make_obstacle(outroot, np.array([-40,0,0]), np.array([-20,0.1,0]))
	make_obstacle(outroot, np.array([40,0,0]), np.array([60,0.1,0]))


	#tile for multiple trains
	# train sides:
	# make_obstacle(outroot, np.array([-20,0,0]), np.array([-20,0.1,5]))
	# make_obstacle(outroot, np.array([40,0,0]), np.array([40,0.1,5]))
	bAddingCorridors = False
	tiling = [-1, 0, 1]
	for i in tiling:
		offset = np.array([i*train_dims[0],0,0])
		offset2d = np.array([i*train_dims[0],0])
		make_train_obstacles(outroot, offset, train_dims[0], train_dims[1], train_wall_thickness, [7, 13], door_width, args.obstacleHeight)

		goal_in_train = {
			"goal_type": "boxregion",
			"targetLocation": [4,5],
			"goal_region": [-18,38, 4,5]
		}
		goal_door = {
			"goal_type": "statictarget",
			"goal_location": [0,0]
		}
		goal_alight = {
			"goal_type": "boxregion",
			"targetLocation": [-3,-5],
			"goal_region": [-18,38, -platform_depth+1,-platform_depth]
		}


		lengths = np.array([train_dims[0] / 2, -platform_depth])
		leftover = 0
		make_manual_agents_in_square(outroot, offset2d- 0.3, lengths, agents_per_region, agent_radius, [ goal_door, goal_in_train])
		leftover = make_manual_agents_in_square(outroot, offset2d + np.array([train_dims[0]/2,0]) -0.3, lengths, agents_per_region, agent_radius, [ goal_door, goal_in_train])

		lengths_carriage = np.array([train_dims[0] / 2, 5])
		make_manual_agents_in_square(outroot, offset2d, lengths_carriage, agents_in_carriage, agent_radius, [goal_door, goal_alight])
		make_manual_agents_in_square(outroot, offset2d + np.array([train_dims[0]/2,0]), lengths_carriage, agents_in_carriage, agent_radius, [goal_door, goal_alight])

		if(leftover > 0):
			bAddingCorridors = True
			width = 6
			height = (2 * leftover / math.floor(width / (2*agent_radius)) + 1)* (2*agent_radius)
			20
			corridor_origin = offset2d + np.array([(train_dims[0] - width) / 2, -platform_depth])
			add_corridor(outroot, corridor_origin, width, height)
			# print("issue making agents", file=sys.stderr)
			# add walls opposite carriage
			make_obstacle_2d( outroot, offset2d + np.array([0,-platform_depth - train_wall_thickness]), corridor_origin)
			make_obstacle_2d( outroot, corridor_origin + np.array([width,-train_wall_thickness]), offset2d + np.array([ train_dims[0],-platform_depth]))

			# Add remaining agents
			goal_corridor = {
				"goal_type": "boxregion",
				"targetLocation": [0,0],
				"goal_region": [corridor_origin[0] + 1.5, corridor_origin[0] + width - 1.5, -platform_depth+1.5, -platform_depth + 2]
			}
			goals = [ goal_door, goal_in_train]

			corridor_leftover = make_manual_agents_in_square(outroot, corridor_origin, np.array([width, -height]), leftover*2, agent_radius, goals)
			if corridor_leftover > 0:
				print("corridor too small")

			# raise Exception("issue making agents")
	if not(bAddingCorridors):
		make_obstacle(outroot, np.array([-20,0,-platform_depth]), np.array([40,0.1,-platform_depth]))


	#write xml to file
	outtree = ET.ElementTree(outroot)
	print("writing to " + args.outputName)
	indent(outroot)
	outtree.write(args.outputName)


def generate_xml_intercity(radius, agents_per_region, agents_in_carriage, platform_depth, outputName):
	door_width =  1.3
	train_dims = (24,5) #x,z
	train_wall_thickness = 0.1 
	plat_dims = ([100,platform_depth]) #x,z
	plat_origin = np.array([-40,0,-platform_depth])
	

	outroot = initialize_xml()

	# Location of doors (for goal purposes)
	# door_goals = []
	
	# station+ rails
	make_hollow_square_obstacle(outroot, plat_origin, plat_dims[0], plat_dims[1] + train_dims[1])
	make_obstacle(outroot, np.array([-40,0,-platform_depth]), np.array([-20,0.1,-platform_depth]))
	make_obstacle(outroot, np.array([40,0,-platform_depth]), np.array([60,0.1,-platform_depth]))

	#block off rails
	make_obstacle(outroot, np.array([-40,0,0]), np.array([-train_dims[0],0.1,0]))
	make_obstacle(outroot, np.array([2*train_dims[0],0,0]), np.array([60,0.1,0]))


	#tile for multiple trains
	# train sides:
	# make_obstacle(outroot, np.array([-20,0,0]), np.array([-20,0.1,5]))
	# make_obstacle(outroot, np.array([40,0,0]), np.array([40,0.1,5]))
	bAddingCorridors = False
	tiling = [-1, 0, 1]
	for i in tiling:
		offset = np.array([i*train_dims[0],0,0])
		offset2d = np.array([i*train_dims[0],0])
		make_train_obstacles(outroot, offset, train_dims[0], train_dims[1], train_wall_thickness, [2,22], door_width, args.obstacleHeight)

		goal_in_train = {
			"goal_type": "boxregion",
			"targetLocation": [4,5],
			"goal_region": [-24,48, 4,5]
		}
		goal_door = {
			"goal_type": "statictarget",
			"goal_location": [0,0]
		}
		goal_alight = {
			"goal_type": "boxregion",
			"targetLocation": [-3,-5],
			"goal_region": [-24,48, -platform_depth+1,-platform_depth]
		}


		lengths = np.array([train_dims[0] / 2, -platform_depth])
		leftover = 0
		make_manual_agents_in_square(outroot, offset2d- 0.3, lengths, agents_per_region, agent_radius, [ goal_door, goal_in_train])
		leftover = make_manual_agents_in_square(outroot, offset2d + np.array([train_dims[0]/2,0]) -0.3, lengths, agents_per_region, agent_radius, [ goal_door, goal_in_train])

		lengths_carriage = np.array([train_dims[0] / 2, 5])
		make_manual_agents_in_square(outroot, offset2d, lengths_carriage, agents_in_carriage, agent_radius, [goal_door, goal_alight])
		make_manual_agents_in_square(outroot, offset2d + np.array([train_dims[0]/2,0]), lengths_carriage, agents_in_carriage, agent_radius, [goal_door, goal_alight])

		if(leftover > 0):
			bAddingCorridors = True
			width = 6
			height = (2 * leftover / math.floor(width / (2*agent_radius)) + 1)* (2*agent_radius)
			20
			corridor_origin = offset2d + np.array([(train_dims[0] - width) / 2, -platform_depth])
			add_corridor(outroot, corridor_origin, width, height)
			# print("issue making agents", file=sys.stderr)
			# add walls opposite carriage
			make_obstacle_2d( outroot, offset2d + np.array([0,-platform_depth - train_wall_thickness]), corridor_origin)
			make_obstacle_2d( outroot, corridor_origin + np.array([width,-train_wall_thickness]), offset2d + np.array([ train_dims[0],-platform_depth]))

			# Add remaining agents
			goal_corridor = {
				"goal_type": "boxregion",
				"targetLocation": [0,0],
				"goal_region": [corridor_origin[0] + 1.5, corridor_origin[0] + width - 1.5, -platform_depth+1.5, -platform_depth + 2]
			}
			goals = [ goal_door, goal_in_train]

			corridor_leftover = make_manual_agents_in_square(outroot, corridor_origin, np.array([width, -height]), leftover*2, agent_radius, goals)
			if corridor_leftover > 0:
				print("corridor too small")

			# raise Exception("issue making agents")
	if not(bAddingCorridors):
		make_obstacle(outroot, np.array([-20,0,-platform_depth]), np.array([40,0.1,-platform_depth]))


	#write xml to file
	outtree = ET.ElementTree(outroot)
	print("writing to " + args.outputName)
	indent(outroot)
	outtree.write(args.outputName)


if __name__ == "__main__":
	default_agent_radius = 0.9
	default_agents_per_region = 10
	default_agents_per_carriage = 10
	default_platform_depth = 6

	parser = argparse.ArgumentParser(description='Generate Steersuite xml input file for train PTI')
	parser.add_argument('-n','--numPerDoor', type=int, default=default_agents_per_region,
                		help='number of people per door to spawn (default: 10')
	parser.add_argument('-nc','--numPerCarriage', type=int, default=default_agents_per_region,
                		help='number of people per door to spawn starting within the carriage (default: 10')
	parser.add_argument('-r','--radius', type=float, default=default_agent_radius,
                    	help='radius of agents social distance in meters (default: 0.9m)')
	parser.add_argument("-oh", "--obstacleHeight", default = 1, help="visual height of obstacles")
	parser.add_argument("-o", "--outputName", default = "merseyrail.xml", help="name of generated file ")
	parser.add_argument("-d", "--depthPlatform", type = float, default = default_platform_depth, help="depth of the platform (z)")
	parser.add_argument("-rand", "--randomSeed", type = int, default = -1, help="random seed for simulation. -1 for no seeding")
	parser.add_argument("-c", "--carriage", type = int, default = 0, help="suburban (0) or intercity (1) train type", metavar="[0,1]")

	args = parser.parse_args()

	agent_radius = args.radius
	agents_per_region= args.numPerDoor
	agents_per_carriage= args.numPerCarriage
	outputName = args.outputName
	platform_depth = args.depthPlatform

	if(args.randomSeed != -1):
		random.seed(args.randomSeed)

	if args.carriage == 0:
		generate_xml(agent_radius, agents_per_region, agents_per_carriage, platform_depth, outputName)
	elif args.carriage == 1:
		generate_xml_intercity(agent_radius, agents_per_region, agents_per_carriage, platform_depth, outputName)
	else:
		print("invalid argument for carriage")
