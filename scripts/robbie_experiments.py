import cPickle
import tempfile
import os
import os.path as osp
import subprocess
import argparse
import do_task
import time

SCRIPTS_DIR = "/home/robbie/ros-stuff/robbie_git/rapprentice/scripts"
DATA_DIR = "/home/henrylu/Data/overhand"
H5FILE = "master.h5"

def do_single_random_task():
	demo1 = "demo1-seg00"
	demo_name = osp.join(DATA_DIR, H5FILE)
	do_task.do_single_random_task(demofile_name=demo_name, init_rope_state_segment=demo1, 
								perturb_radius=0.1, perturb_num_points=7)

def do_many_segments(iterations):
	results = []
	for i in range(iterations):
		results.append(do_segments())
		
	print "All results =", results
	final_results = [result[2] for result in results]
	print "Final_results =", final_results
	final_result_ints = [1.0 if result else 0.0 for result in final_results]
	successes = reduce(lambda x, y: x + y, final_result_ints)
	print "num success =", successes
	print "success rate =", successes / iterations


def do_these_segments(segments, animate=False):
	start = time.time()
	demo1 = "demo1-seg00"
	demofile = osp.join(DATA_DIR, H5FILE)
	return_val = do_task.do_several_segments(demofile_name=demofile, init_rope_state_segment=demo1, 
								perturb_radius=0.15, perturb_num_points=7, segments=segments, animate=animate, filename="do_segments.pkl")
	print "do_segments return val = ", return_val
	end = time.time()
	print "seconds taken =", end-start
	return return_val

def do_segments(animate=False):
	segments = ["demo1-seg00", "demo1-seg01", "demo1-seg02"]
	#segments = ["demo1-seg00"]
	return do_these_segments(segments, animate)

def main():
	#do_single_random_task()
	#do_segments(animate=False)
	do_many_segments(1)
	
if __name__ == "__main__":
    main()