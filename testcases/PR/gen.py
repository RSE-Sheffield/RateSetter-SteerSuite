from typing import List, Set, TypedDict
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
	# goal_type
	target_location: list[float] # positions
	goal_region: List[float] # (x1,y1, x2,y2) 2 opposite corners of the box goal

class AgentGoalTargetSet(AgentGoal):
	"""
	Typing hint of TargetSet agent goal
	"""
	# goal_type
	goal_location: list[list[float]] # list of (x,y/z) goal locations
	parameters: Set[str] # {"low priority", "boarding"}

class AgentInfo(TypedDict):
	"""
	Typing hint of agent info dictionary
	"""
	radius: float # physical radius of person
	sdradius: float # social distancing radius
	goals: list[AgentGoal]


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



	def spawn_agents_in_square(self, square: List[float], num_agents: int, **agent_behaviour):
		"""
		spawn some agents with a square region. It assumes the whole area is free from other object, and agents can spawn anywhere within the square
		"""


train: TrainInfo = {
	"carriages": 8, # how many carriages per train
	"c_length": 16, # length of the carriage
	"c_width": 3, # width of the carriage
	"doors": [5, 11], # location of door centers along cariage
	"door_size": 1
}

agent_spec: AgentInfo = {
	"radius": 0.3, # physical radius of person
	"sdradius": 0, # social distancing radius
	"goals": list[AgentGoal],

}

tc = GenTestcase()
tc.gen_train(**train)
tc.spawn_agents_in_square([0,0,5,5], 5, **AgentInfo)
print(tc.dump())
tc.write("testcases/PR/foo.xml")
