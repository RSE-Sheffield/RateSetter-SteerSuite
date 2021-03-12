import os
import subprocess
import time

import pandas as pd
import numpy as np

# Merseyrail xml gen file
mrxmlgen_loc = "../../testcases/Merseyrail"
mrxmlgen_name = "GenerateSetup.py"

# Steersuite
ss_wd = "./"
# ss_exe = "../bin/steersim"
ss_exe = "../../build/bin/steersim"
ss_config = "myconfig.xml"

min_radius = 0.2 # the person-environment interaction radius for a person. Must adjust and recompile steersuite: Set MIN_RADIUS in RVO2D_Parameters.h and pedestrian_radius in DrawLib.cpp

# radius_arr = [0.4, 0.9] #[0.4, 0.9]#np.arange(0.4, 1.0, 0.1)
sd_arr = [0, 1.0,  2.0]
num_arr = np.arange(10, 60, 10)
nc_arr = np.arange(10, 60, 10)
min_soc_dist = [min_radius]
region_far = [-1]
region_close = [-5]
platform_depth = [6]
iterations = 1
carriage = [0] #nb need to recompile Steersuite when changing this value 0=suburban 1=intercity

# Set this to 1/FPS where FPS is found in the config file as "FixedFPS"
# timestep = 0.016667
timestep = 0.033333
max_frames = 500000
maxtime = 60

outfile_name = "results"

# -------------------------------------
path = os.getcwd()
print (path)
num_iterations = iterations * len(min_soc_dist) * len(region_far) * len(region_close) * len(sd_arr) * len(num_arr) * len(platform_depth) * len(nc_arr)

results = []
num_skipped = 0
num_max_frames = 0
num_failed = 0
start_time = time.time()

# ---------------------------------------
# Begin many loops
print("Running " + str(num_iterations) + " iterations")
print("\n")
number_of_iterations = 0
for i in range(iterations):
	for d in platform_depth:
		for car in carriage:
			for msd in min_soc_dist:
				for rf in region_far:
					for rc in region_close:


						# generate param file for visual studio SS to read in
						with open(ss_wd+"params.txt", "w+") as f:
							f.write("min_soc_dist " + str(msd) + "\n")
							f.write("region_far " + str(rf) + "\n")
							f.write("region_close " + str(rc) + "\n")

						for sd in sd_arr:
							r = min_radius + (sd / 2)
							r_column_name = str(r)
							for n in num_arr:
								for nc in nc_arr:
									number_of_iterations += 1
									if number_of_iterations % 1 == 0:
										remaining_time = ((time.time() - start_time) / number_of_iterations) * (num_iterations - number_of_iterations) / 60
										print("\r%4.2f percent complete... %s minutes \t Remainin: %s minutes" % (number_of_iterations / num_iterations * 100, round((time.time() - start_time) / 60,2), round(remaining_time,2)),end='', flush=True) #data: %i columns \t Number skipped: %i \t Number Maxed: %i \t Number Failed: %i" % (number_of_iterations / num_iterations * 100, len(results), num_skipped, num_max_frames, num_failed), end='', flush=True)

									# Skip those that are useless combinations
									# if msd > r or (rc == 3 and msd != 0.4):
									# 	num_skipped += 1
									# 	continue

									# Gen xml file
									os.chdir(mrxmlgen_loc)

									try:
										subprocess.check_output(["python3", mrxmlgen_name, "-n", str(n), "-nc", str(nc), "-r", str(r), "-d", str(d), "-c", str(car)], stderr=subprocess.PIPE)
									except subprocess.CalledProcessError as e:
										print(e)
										os.chdir(path)
										num_failed +=1
										continue


									# run SS
									os.chdir(path)
									os.chdir(ss_wd)
									# path2 = os.getcwd()
									# print (path2)
									try:
										# print("Running sim...", end="")
										# ss_output = subprocess.check_output((ss_exe, "-config", ss_config), stderr=subprocess.STDOUT)

#ss_output = subprocess.check_output((ss_exe, "-config", ss_config, "-commandline"), stderr=subprocess.STDOUT, timeout=maxtime)
										ss_output = subprocess.check_output((ss_exe, "-config", ss_config), stderr=subprocess.STDOUT)
										# print("done")
									except subprocess.CalledProcessError as e:
										os.chdir(path)
										num_failed +=1
										continue

									except subprocess.TimeoutExpired as e:
										os.chdir(path)
										num_failed +=1
										continue                             

									# Grab number of frames
									ss_output = ss_output.decode("utf-8") 
									start_search = "Simulated "
									startloc = ss_output.find(start_search)
									endloc = ss_output.find(" frames.")
									frames = ss_output[startloc + len(start_search):endloc]
									frames = int(frames)
									actual_time = frames * timestep
									# print(frames)

									if frames == max_frames:
										os.chdir(path)
										num_max_frames += 1
										continue

									# Grab SD frames
									start_search = "less than SD: "
									startloc = ss_output.find(start_search)
									endloc = ss_output.find(" SD frames")
									aveSD = ss_output[startloc + len(start_search):endloc]
									aveSD = float(aveSD)
									aveSDtime = aveSD * timestep
									start_search = "Max individual time: "
									startloc = ss_output.find(start_search)
									endloc = ss_output.find(" max frames")
									maxSD = ss_output[startloc + len(start_search):endloc]
									maxSD = int(maxSD)
									maxSDtime = maxSD * timestep


									os.chdir(path)

									stock_type_name = "suburban" if (car==0) else "intercity"

									result = {
										"radius":r,
										"social distance": sd,
										"passengers boarding per door": n, 
										"passengers alighting per door": nc, 
										"minimum radius": msd, 
										"far distance from door": rf, 
										"close distance from door": rc, 
										"platform width": d,
										"frames": frames, 
										"time": actual_time,
										"max individual SD": maxSDtime,
										"ave SD": aveSDtime,
										"stock type": car,
									}
									results.append(result)

	# iteration complete
	outputdf = pd.DataFrame(results)
	outputdf.to_csv(outfile_name+str(i)+".csv", sep="\t", index=False )
	#outputdf.to_excel(outfile_name+str(i)+".xlsx")
#outputdf = pd.DataFrame(results)
#outputdf.to_csv(outfile_name+".csv", sep="\t", index=False )
#outputdf.to_excel(outfile_name+".xlsx")
print("saved iteration")
print("")
print("Num failed: %d" % num_failed)
print("")
print(outputdf)
