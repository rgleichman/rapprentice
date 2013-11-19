import cPickle
import tempfile
import os
import os.path as osp
import subprocess
import argparse
import do_task
import time

SCRIPTS_DIR = "/home/robbie/ros-stuff/robbie_git/rapprentice/scripts"
DATA_DIR = "/mnt/storage/robbie/hdf5_working"
H5FILE = "all_exp"
file_path
to_add

def do_stuff():
    demo1 = "demo1-seg00"
    demo_name = file_path
    do_task.do_single_random_task(demofile_name=demo_name, init_rope_state_segment=demo1,
                                  perturb_radius=0.1, perturb_num_points=7)


def do_many_segments(iterations):
    results = []
    for i in range(iterations):
        results.append(do_segments())
        final_results = [result[-1] for result in results]
        final_result_ints = [1.0 if result else 0.0 for result in final_results]
        successes = reduce(lambda x, y: x + y, final_result_ints)
        print "num success =", successes
        print "success rate so far =", float(successes) / (i + 1.0)
    print "All results =", results
    final_results = [result[-1] for result in results]
    final_result_ints = [1.0 if result else 0.0 for result in final_results]
    successes = reduce(lambda x, y: x + y, final_result_ints)
    print "num success =", successes
    print "success rate =", successes / iterations
    print "Final_results =", final_results


def do_many(iterations, func):
    results = []
    for i in range(iterations):
        results.append(func())
        final_results = [result[-1] for result in results]
        final_result_ints = [1.0 if result else 0.0 for result in final_results]
        successes = reduce(lambda x, y: x + y, final_result_ints)
        print "num success =", successes
        print "num failures =", i+1 - successes
        print "success fraction =", successes, "/", i+1
        print "success rate so far =", float(successes) / (i + 1.0)
        print "trial length = ", iterations
    print "All results =", results
    final_results = [result[-1] for result in results]
    final_result_ints = [1.0 if result else 0.0 for result in final_results]
    successes = reduce(lambda x, y: x + y, final_result_ints)
    print "num success =", successes
    print "success rate =", successes / iterations
    print "Final_results =", final_results


def do_both(iterations, starting_seed, func1, func2):
    f1_results = []
    f2_results = []
    for i in range(iterations):
        f1_results.append(func1(starting_seed + i))
        f2_results.append(func2(starting_seed + i))
        f1_final_results = [result[-1] for result in f1_results]
        f1_passes = sum(1.0 if result else 0.0 for result in f1_final_results)
        f2_final_results = [result[-1] for result in f2_results]
        f2_passes = sum(1.0 if result else 0.0 for result in f2_final_results)
        combined_passes = sum(1 if (f1 or f2) else 0 for f1, f2 in zip(f1_final_results, f2_final_results))

        print "On iteration (i+1)=", i + 1
        print "Func1_results =", f1_results
        print "Func2_results =", f2_results
        print "f1_passes =", f1_passes
        print "f2_passes =", f2_passes
        print "f1_pass_rate =", f1_passes / float(i + 1)
        print "f2_pass_rate =", f2_passes / float(i + 1)
        print "combined_pass_rate =", combined_passes / float(i + 1)
        print "total_trials =", iterations


def do_one(iterations, starting_seed, func1):
    f1_results = []
    for i in range(iterations):
        f1_results.append(func1(starting_seed + i))
        f1_final_results = [result[-1] for result in f1_results]
        f1_passes = sum(1.0 if result else 0.0 for result in f1_final_results)

        print "On iteration (i+1)=", i + 1
        print "Func1_results =", f1_results
        print "f1_passes =", f1_passes
        print "f1_pass_rate =", f1_passes / float(i + 1)
        print "total_trials =", iterations


def do_these_segments(segments, animate=False):
    start = time.time()
    demo1 = "demo1-seg00"
    demofile = file_path
    return_val = do_task.do_single_random_task(demofile_name=demofile, init_rope_state_segment=demo1,
                                               perturb_radius=0.1, perturb_num_points=7, segments=segments,
                                               animate=animate, filename="nada.pkl")
    print "do_segments return val = ", return_val
    end = time.time()
    print "seconds taken =", end - start
    return return_val


def execute_demo1_segments(random_seed=None):
    segments = ["demo1-seg00", "demo1-seg01", "demo1-seg02"]
    #segments = ["demo1-seg00"]
    seg_iter = (x for x in segments)

    def choose_seg(_, _x):
        return seg_iter.next()

    return do_demo1(choose_seg, 3, random_seed)


def make_basic_rope_state(demo):
    return do_task.RopeState(demo, 0.1, 7)


def make_basic_task_params(demofile, choose_segment, log_name):
    return do_task.TaskParameters(demofile_name=demofile, knot="K3a1", animate=False, max_steps_before_failure=6,
                                  choose_segment=choose_segment, log_name=log_name)


def do_demo1(choose_segment, max_steps=5, random_seed=None, add_to_hdf5=False):
    start = time.time()
    demo1 = "demo1-seg00"
    demofile = file_path
    rope_state = make_basic_rope_state(demo1)
    task_params = make_basic_task_params(demofile, choose_segment,
                                         "/mnt/storage/robbie/logs/test_demo1_both_auto_and_demo1-seed841-1.pkl")
    task_params.random_seed = random_seed
    task_params.max_steps_before_failure = max_steps
    task_params.add_to_hdf5 = add_to_hdf5
    #TODO: remove
    #task_params.animate = True
    return_val = do_task.do_single_random_task(rope_state, task_params)
    print "do_segments return val = ", return_val
    end = time.time()
    print "seconds taken =", end - start
    return return_val


def do_auto(random_seed=None):
    #TODO: change max_steps to 5?
    return do_demo1(do_task.find_closest_auto, 5, random_seed, add_to_hdf5=to_add)


def main():
    #do_stuff()
    #do_segments(animate=False)
    #do_many_segments(100)
    #do_many(60, do_auto)
    do_both(100, 841, execute_demo1_segments, do_auto)
    #do_one(100, 841, do_auto)


if __name__ == "__main__":
    try:
        print 'Enter the number of the data file you wish to use.\n'
        file_number = int(raw_input('Input:'))
    except ValueError:
        print 'Not a number'
    try:
        datafile = H5FILE + file_number + ".h5"
        global to_add
        global file_path
        to_add = True
        file_path = osp.join(DATA_DIR, datafile)
        if osp.isfile(file_path):
            print 'File already exists would you like to use a copy or not add to hdf5 file?\n'
            print 'Enter 1 to make a copy, 2 to not add to hdf5 file, and -1 to exit'
            input = int(raw_input('Input:'))
            if (input == 1):
                datafile = H5FILE + file_number + '_copy.h5'
                file_path = osp.join(DATA_DIR, datafile)
            if (input == 2):
                to_add = False
            if (input == -1):
                print 'Exiting'
                sys.exit()
        main()
    except ValueError:
        print 'Not a number'
