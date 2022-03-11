from typing import List, Set, TypedDict
import math
import random
import xml.etree.ElementTree as ET
# import numpy as np

###############
# Typing Hints
class TrainInfo(TypedDict):
	"""
	Typing hint of train info dictionary
	"""
	carriages: int # how many carriages per train
	c_length: float # length of the carriage
	c_width: float # width of the carriage
	doors: List[float] # array of centred door positions
	door_size: float # in meters from one end to the other door

class AgentGoal(TypedDict):
	"""
	Typing hint of agent goal dictionary
	"""
	goal_type: str

class AgentGoalBox(AgentGoal):
	"""
	Typing hint of BoxRegion agent goal
	"""
	# goal_type = "boxregion"
	target_locations: List[float] # positions
	goal_region: List[float] # (x1,y1, x2,y2) 2 opposite corners of the box goal

class AgentGoalTargetSet(AgentGoal):
	"""
	Typing hint of TargetSet agent goal
	"""
	# goal_type = "targetSet"
	goal_location: List[List[float]] # list of (x,y/z) goal locations
	parameters: Set[str] # {"low priority", "boarding"}

class AgentInfo(TypedDict):
	"""
	Typing hint of agent info dictionary
	"""
	radius: float # physical radius of person
	sdradius: float # social distancing radius
	goals: List[AgentGoal]


class GenTestcase():
	# default y/height of all obstacles
	obstacle_heights = 0.5
	xmlparent = ET.Element

	def __init__(self):
		self.xmlparent = self.gen_head()

	def dump(self):
		self.indent(self.xmlparent)
		return ET.dump(self.xmlparent)

	def write(self, location):
		outtree = ET.ElementTree(tc.xmlparent)
		self.indent(self.xmlparent)
		outtree.write(location)

	def gen_head(self, testcase_name: str = "Testcase") -> ET.Element:
		"""
		Create xml head
		"""
		outroot = ET.Element('SteerBenchTestCase')
		header = ET.SubElement(outroot, 'header')
		ET.SubElement(header, 'version').text = "1.0"
		ET.SubElement(header, 'name').text = testcase_name
		worldBounds = ET.SubElement(header, 'worldBounds')
		ET.SubElement(worldBounds, 'xmin').text= "-100"
		ET.SubElement(worldBounds, 'xmax').text= "100"
		ET.SubElement(worldBounds, 'ymin').text= "0"
		ET.SubElement(worldBounds, 'ymax').text= "0"
		ET.SubElement(worldBounds, 'zmin').text= "-100"
		ET.SubElement(worldBounds, 'zmax').text= "100"

		return outroot

	def gen_train(self, **train):
		"""
		Create train obejects from multiple carriages. 
		Aligned so doors open along the x-axis.
		The train start at the origin(0,0), and takes up space in (+x,-z) space (people on the train are in -z position)
		"""
		for i in range(train['carriages']):
			train['origin'] = [i*train['c_length'], 0]
			self.gen_carriage(**train)

	def gen_carriage(self, **carriage):
		"""
		Create a single train carraige
		"""
		o = carriage['origin']

		# Left and right walls
		self.create_thin_wall(o[0], 					   [-carriage['c_width'], 0])
		self.create_thin_wall(o[0] + carriage['c_length'], [-carriage['c_width'], 0])
		# back wall (z=carriage:c_length)
		self.create_thin_wall([o[0], o[0] + carriage['c_length']], -carriage['c_width'])
		# Front parts (z=0)
		self.create_thin_wall(
			[
				o[0], 
				o[0] + carriage['doors'][0] - 0.5 * carriage['door_size']
			], 0)
		for i, d in enumerate(carriage['doors'][:-1]):
			self.create_thin_wall(
				[
					o[0] + carriage['doors'][i] + 0.5 * carriage['door_size'], 
					o[0] + carriage['doors'][i+1] - 0.5 * carriage['door_size']
				], 0)
		self.create_thin_wall(
				[
					o[0] + carriage['doors'][-1] + 0.5 * carriage['door_size'], 
					o[0] + carriage['c_length']
				], 0)


	def create_thin_wall(self, x, z, y: float = None):
		"""
		Creates an x or z aligned object with very thin dimension.
		One of x or z should be a single value, while the other is a double value from min to max
		"""
		if y == None:
			y = self.obstacle_heights

		# how thin is the wall?
		thin_width = 0.1

		if type(x) is not list: x = [ x ]
		if type(z) is not list: z = [ z ]
		if not( (len(x) == 1 and len (z) == 2) or (len(x) == 2 and len(z) == 1)):
			raise Exception("incorrect number of parameters")  
		
		if len(x) == 2:
			self.create_object(x = x, z = [z[0], z[0]+thin_width], y=y)
		else:
			self.create_object(x = [x[0], x[0]+thin_width], z = z, y=y)

	def create_object(self, x: List[float], z: List[float], y: float = None):
		if (not (len(x) == 2 and len (z) == 2)):
			raise Exception("incorrect number of parameters")  

		if y == None:
			y = self.obstacle_heights
		
		obstacle = ET.SubElement(self.xmlparent, 'obstacle')
		ET.SubElement(obstacle, 'xmin').text= str(x[0])
		ET.SubElement(obstacle, 'xmax').text= str(x[1])
		ET.SubElement(obstacle, 'ymin').text= "0"
		ET.SubElement(obstacle, 'ymax').text= str(y)
		ET.SubElement(obstacle, 'zmin').text= str(z[0])
		ET.SubElement(obstacle, 'zmax').text= str(z[1])


	#prettify output
	def indent(self, elem, level=0):
		i = "\n" + level*"  "
		if len(elem):
			if not elem.text or not elem.text.strip():
				elem.text = i + "  "
			if not elem.tail or not elem.tail.strip():
				elem.tail = i
			for elem in elem:
				self.indent(elem, level+1)
			if not elem.tail or not elem.tail.strip():
				elem.tail = i
		else:
			if level and (not elem.tail or not elem.tail.strip()):
				elem.tail = i

	def add_goal(self, goal_sequence_xml, goal_dict):
		"""
		Add a goal to an agent xml
		"""
		goal_type = goal_dict["goal_type"]
		
		if(goal_type == "statictarget"):
			goal_location = goal_dict["goal_location"]
			seekStaticTarget = ET.SubElement(goal_sequence_xml, 'seekStaticTarget')
			targetLocation = ET.SubElement(seekStaticTarget, 'targetLocation')
			ET.SubElement(targetLocation, 'x').text= str(goal_location[0])
			ET.SubElement(targetLocation, 'y').text= "0"
			ET.SubElement(targetLocation, 'z').text= str(goal_location[1])
			ET.SubElement(seekStaticTarget, 'timeDuration').text = "1000.0"
			ET.SubElement(seekStaticTarget, 'desiredSpeed').text = "1.3"
		
		elif(goal_type == "boxregion"):
			goal_region = goal_dict["goal_region"]
			goal_location = [0,0]
			seekAxisAlignedBoxRegion = ET.SubElement(goal_sequence_xml, 'seekAxisAlignedBoxRegion')
			targetLocation = ET.SubElement(seekAxisAlignedBoxRegion, 'targetLocation')
			ET.SubElement(targetLocation, 'x').text= str(goal_location[0])
			ET.SubElement(targetLocation, 'y').text= "0"
			ET.SubElement(targetLocation, 'z').text= str(goal_location[1])
			ET.SubElement(seekAxisAlignedBoxRegion, 'desiredSpeed').text = "1.3"
			ET.SubElement(seekAxisAlignedBoxRegion, 'timeDuration').text = "1000.0"
			goalRegionBounds = ET.SubElement(seekAxisAlignedBoxRegion, 'goalRegionBounds')
			ET.SubElement(goalRegionBounds, 'xmin').text= str(goal_region[0])
			ET.SubElement(goalRegionBounds, 'xmax').text= str(goal_region[1])
			ET.SubElement(goalRegionBounds, 'ymin').text= "0"
			ET.SubElement(goalRegionBounds, 'ymax').text= "0"
			ET.SubElement(goalRegionBounds, 'zmin').text= str(goal_region[2])
			ET.SubElement(goalRegionBounds, 'zmax').text= str(goal_region[3])
		
		elif(goal_type == "targetSet"):
			seekTargetSet = ET.SubElement(goal_sequence_xml, 'seekStaticTargetSet')
			targetLocationsSet = ET.SubElement(seekTargetSet, 'targetLocationsSet')
			for goal in goal_dict["goal_locations"]:
				targetLocation = ET.SubElement(targetLocationsSet, 'targetLocation')
				ET.SubElement(targetLocation, 'x').text = str(goal[0])		
				ET.SubElement(targetLocation, 'y').text = "0"
				ET.SubElement(targetLocation, 'z').text = str(goal[1])

			ET.SubElement(seekTargetSet, 'timeDuration').text = "1000.0"
			ET.SubElement(seekTargetSet, 'desiredSpeed').text = "1.3"
			if("parameters" in goal_dict):
				Behaviour = ET.SubElement(seekTargetSet, 'Behaviour')
				Parameters = ET.SubElement(Behaviour, 'Parameters')
				for spec in goal_dict["parameters"]:
					lp = ET.SubElement(Parameters, spec.replace(" ", ""))
					ET.SubElement(lp, 'key').text = spec
					ET.SubElement(lp, 'value').text = "1"

		else:
			print("unknown goal type attempting to be added")

	def make_agent(self, location: List[float], agent_behavior: AgentInfo):
		"""
		"""
		agent= ET.SubElement(self.xmlparent, 'agent')

		radius = agent_behavior["radius"]
		sdradius = agent_behavior["sdradius"]
		goals_arr = agent_behavior["goals"]

		#initial conditions
		initialConditions= ET.SubElement(agent, 'initialConditions')
		direction = ET.SubElement(initialConditions, 'direction')
		ET.SubElement(direction, 'x').text = "0"
		ET.SubElement(direction, 'y').text = "0"
		ET.SubElement(direction, 'z').text = "0"
		position = ET.SubElement(initialConditions, 'position')
		ET.SubElement(position, 'x').text = str(location[0])
		ET.SubElement(position, 'y').text = "0"
		ET.SubElement(position, 'z').text = str(location[1])
		ET.SubElement(initialConditions, 'radius').text= str(radius)
		ET.SubElement(initialConditions, 'sdradius').text= str(sdradius)
		ET.SubElement(initialConditions, 'speed').text= "0"

		#goal sequence
		goalSequence= ET.SubElement(agent, 'goalSequence')
		for g in goals_arr:
			self.add_goal(goalSequence, g)

		# add agent behaviour
		# behaviour = ET.SubElement(agent, 'behaviour')
		# sdradius_z = ET.SubElement(behaviour, 'sdradius_z')
		# ET.SubElement(sdradius_z, 'z0').text= "-3"
		# ET.SubElement(sdradius_z, 'z1').text= "0"
		# ET.SubElement(sdradius_z, 'sd0').text= "0.2"
		# ET.SubElement(sdradius_z, 'sd1').text= "0"

	def spawn_agents_in_square(self, square: List[float], num_agents: int, **agent_behaviour: AgentInfo) -> int:
		"""
		spawn some agents with a square region. It assumes the whole area is free from other object, and agents can spawn anywhere within the square

		square: [x1,z1,  x2, z2] 2 corners of the square (with min x,z in one and max x,z in the other)
		
		Returns: the number of agents that did not fin into the spawned region
		"""
		radius = agent_behaviour["radius"]
		sdradius = agent_behaviour["sdradius"]
		lengths = [
			square[2] - square[0],
			square[3] - square[1]
		]
		origin = [square[0], square[1]]
		goals = agent_behaviour["goals"]

		# Can we fit all the agents?
		nums_per_side = [math.floor(abs(lengths[0]/((radius+sdradius)*2))), 
						 math.floor(abs(lengths[1]/((radius+sdradius)*2)))]
		total_num = int(nums_per_side[0] * nums_per_side[1])
		leftover = num_agents - total_num
		num_agents = total_num if leftover > 0 else num_agents

		# calculate the boxes available
		box_lengths = [lengths[0] / nums_per_side[0], lengths[1] / nums_per_side[1]]
		box_occupancy = list(range(0,total_num))
		box_size = box_lengths
		box_center = [box_size[0] / 2, box_size[1] / 2]
		# extra_space = np.abs(box_size - (2*(radius+sdradius))) / 2

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
			self.make_agent([col_loc, row_loc], agent_behaviour)
			# self.make_agent(radius, sdradius, [col_loc, row_loc], goals)

			# Set id as occupied
			box_occupancy.remove(boxId)

		return leftover if leftover > 0 else 0
		
	def create_plt_4(self):
		"""
		Creates the platform layout of Peckham Rye, Platform 4
		"""
		usual_obj_h = self.obstacle_heights
		heights = self.obstacle_heights + 0.5

		# left side of plt
		self.create_thin_wall(-5, [0,5])
		self.create_thin_wall([-5,0],0)

		# back of platform
		self.create_thin_wall([-5,75], 5)
		self.create_thin_wall(75, [5,2])
		self.create_thin_wall([75,88], 2)
		self.create_thin_wall(88, [2,8])
		self.create_thin_wall([88,96], 8)
		self.create_thin_wall(96, [8,2])
		self.create_thin_wall([96,107], 2)
		self.create_thin_wall(107, [2,5])
		self.create_thin_wall([107,132], 5)

		# right side
		self.create_thin_wall(132, [0,5])
		self.create_thin_wall([128,132], 0)

		# cleanup
		self.obstacle_heights = usual_obj_h

def get_door_locations(ti: TrainInfo) -> List[List[float]]:
	doors = []
	for i in range(ti["carriages"]):
		for d in ti["doors"]:
			door_loc = [d + (i*ti["c_length"]),0]
			doors.append(door_loc)
	return doors

train: TrainInfo = {
	"carriages": 8, # how many carriages per train
	"c_length": 16, # length of the carriage
	"c_width": 3, # width of the carriage
	"doors": [5, 11], # location of door centers along cariage
	"door_size": 1
}
doors = get_door_locations(train)

gb: AgentGoalBox = {
	"goal_type": "boxregion",
	"target_location": [0,0], # positions
	"goal_region": [0,0,5,5], # (x1,y1, x2,y2) 2 opposite corners of the box goal
}
goal_doors: AgentGoalTargetSet = {
	"goal_type": "targetSet",
	"goal_locations": doors, # list of (x,y/z) goal locations
	# "parameters": Set[str] # {"low priority", "boarding"}
}

agent_spec: AgentInfo = {
	"radius": 0.3, # physical radius of person
	"sdradius": 0, # social distancing radius
	"goals": [goal_doors, gb]
}




tc = GenTestcase()
tc.gen_train(**train)
tc.create_plt_4()
tc.spawn_agents_in_square([0,0,5,5], 5, **agent_spec)
print(tc.dump())
tc.write("testcases/PR/plt-4.xml")
