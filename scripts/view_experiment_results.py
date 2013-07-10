import cPickle

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("input", type=argparse.FileType("r"))
parser.add_argument("--inspect_single", type=int)
args = parser.parse_args()

runs = cPickle.load(args.input)

import trajoptpy
import openravepy
env = openravepy.Environment()
viewer = trajoptpy.GetViewer(env)

if args.inspect_single is not None:
	args, log = runs[args.inspect_single]

	for rope_state in [entry.data for entry in log if entry.name == "execute_traj.sim_rope_nodes_after_full_traj"]:
		handles = []
		handles.append(env.drawlinestrip(rope_state, 5, [1, 0, 0]))
		viewer.Idle()
else:

	for i_run, run in enumerate(runs):
		args, log = run

		print "===== Run # %d =====" % i_run
		print "args:", args
		print "segs chosen:", ", ".join([entry.data for entry in log if entry.name == "find_closest_demo.seg_name"])
		last_rope_state = [entry.data for entry in log[::-1] if entry.name == "execute_traj.sim_rope_nodes_after_full_traj"][0]
		print 'result:', [(entry.data, entry.description) for entry in log[::-1] if entry.name == "result"][0]
		#last_rope_state = [entry.data for entry in log[::-1] if entry.name == "acquire_cloud.init_sim_rope_nodes"][0]
		handles = []
		handles.append(env.drawlinestrip(last_rope_state, 5, [1, 0, 0]))
		viewer.Idle()
		print
