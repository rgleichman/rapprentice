import cPickle
import tempfile
import os
import os.path as osp
import subprocess
import argparse
import do_task

SCRIPTS_DIR = "/home/robbie/ros-stuff/robbie_git/rapprentice/scripts"
DATA_DIR = "/home/henrylu/Data/overhand"
H5FILE = "master.h5"

def do_single_random_task():
	demo1 = "demo1-seg00"
	demo_name = osp.join(DATA_DIR, H5FILE)
	do_task.do_single_random_task(demofile_name=demo_name, init_rope_state_segment=demo1, 
								perturb_radius=0.1, perturb_num_points=7)

def main():
	do_single_random_task()

if __name__ == "__main__":
    main()