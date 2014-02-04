import argparse, do_task
import os.path as osp
import IPython as ipy
from pdb import pm, set_trace

def run_example((taskfile, task_id, actionfile, bootstrap_file)):
    """
    runs a knot-tie attempt for task_id (taken from taskfile
    possible actions are the expert demonstrations in actionfile
        assumed to be an open h5 file in r or r+ mode
    if bootstrap_file == None, then it won't save anything to that file
    set bootstrap_file to add the results from the trial run to that file
         this is assumed to be open in r+ mode
         this will append into that file and assumes that bootstrap_file has all the actions from actionfile in it         
    returns True if this is a knot-tie else returns False
    """
    raise NotImplementedError

def setup_bootstrap_file(action_fname, bootstrap_fname):
    """
    copies over the relevant fields of action file to a bootstrap_file so that we can use the 
    resulting file for run_example
    """
    raise NotImplementedError

def check_bootstrap_file(bootstrap_file, orig_file):
    """
    checks that bootstrap file is properly formatted
    assumes bootstrap_file was generated from orig_file
    this will return False is the file is improperly formatted
        - all the actions in orig_file are in bootstrap_file as their own parent
        - all top level entries have the correct fields
        - all parent and child pointers exist and point to each other
    """
    raise NotImplementedError

DEFAULT_TREE_SIZES = [30, 60, 90, 120]

def run_bootstrap(taskfile, actionfile, bootstrap_fname, burn_in = 40, tree_sizes = None):
    """
    generates a bootstrapping tree
    taskfile has the training examples to use
    bootstrap_fname will be used as the 
    """
    if not tree_sizes:
        tree_sizes = DEFAULT_TREE_SIZES[:]
    assert len(taskfile) >= burn_in + max(tree_sizes)
    raise NotImplementedError

def gen_task_file(fname, num_examples, perturb_bounds, num_perturb_pts):
    """
    draw num_examples states from the initial state distribution defined by
    do_task.sample_rope_state

    writes results to fname
    """
    raise NotImplementedError

def check_task_file(fname):
    """
    probably unecessary, but checks that a task file is properly labelled sequentially and has pt clouds in the values
    """
    raise NotImplementedError

