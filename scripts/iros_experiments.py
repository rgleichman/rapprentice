import argparse, h5py, do_task, sys

import os.path as osp
import IPython as ipy
import numpy as np

from rapprentice import clouds
from pdb import pm, set_trace

DS_SIZE = .025
DEFAULT_TREE_SIZES = [30, 60, 90, 120]

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
    actfile = h5py.File(action_fname, 'r')
    bootfile = h5py.File(bootstrap_fname, 'w')
    for seg_name, seg_info in actfile.iteritems():
        seg_name = str(seg_name)
        cloud_xyz = clouds.downsample(seg_info['cloud_xyz'][:], DS_SIZE)
        hmats = dict((lr, seg_info['{}_gripper_tool_frame'.format(lr)]['hmat'][:]) for lr in 'lr')
        cmat = np.eye(cloud_xyz.shape[0])
        gripper_joints = dict(('{}_gripper_joint'.format(lr), seg_info['{}_gripper_joint'.format(lr)][:]) for lr in 'lr')
        create_bootstrap_item(outfile=bootfile, cloud_xyz=cloud_xyz, root_seg=seg_name, parent=seg_name,
                              children=[], hmats=hmats, cmat=cmat, other_keys=gripper_joints,
                              update_parent=False, seg_name=seg_name)
    actfile.close()
    assert check_bootstrap_file(bootstrap_fname, action_fname)
    return bootfile

def create_bootstrap_item(outfile, cloud_xyz, root_seg, parent, children, hmats, 
                          cmat, other_keys = None, update_parent=True, seg_name=None):
    if not seg_name:
        seg_name = str(len(outfile))
    assert seg_name not in outfile, 'created duplicate segment in bootstrap file'

    g = outfile.create_group(seg_name)
    g['cloud_xyz'] = cloud_xyz # pt cloud associated with this action
    g['root_seg'] = root_seg # string that points to the root segment associated with this segment
    g['parent'] = parent     # string that points to the parent that this segment was matched to
    g['children'] = children if children else 0 # will contain a list of pointers to children for this node
                                                # initialized to be 0 b/c you can't store a 0-sized list in h5
    hmat_g = g.create_group('hmats')
    for lr in 'lr':
        hmat_g[lr] = hmats[lr] # list of hmats that contains the trajectory for the lr gripper in this segment    

    root_xyz = outfile[root_seg]['cloud_xyz'][:]
    root_n = root_xyz.shape[0]
    seg_m = cloud_xyz.shape[0]
    # do this check here, b/c we don't have to explicitly check if root_seg == seg_name
    assert cmat.shape == (root_n, seg_m), 'correspondence matrix formatted wrong'
    g['cmat'] = cmat

    if update_parent:
        parent_children = outfile[parent]['children']
        del outfile[parent]['children']
        if not parent_children: parent_children = []
        parent_children.append(seg_name)
        outfile[parent]['children'] = parent_children
    if other_keys:
        for k, v in other_keys.iteritems():
            g[k] = v
    outfile.flush()
    return seg_name

def check_bootstrap_file(bootstrap_fname, orig_fname):
    """
    checks that bootstrap file is properly formatted
    assumes bootstrap_file was generated from orig_file
    this will return False is the file is improperly formatted
        - all the actions in orig_file are in bootstrap_file as their own parent
        - all top level entries have the correct fields
        - all parent and child pointers exist and point to each other
    returns True if file is formatted correctly
    """
    required_keys = ['children', 'cloud_xyz', 'cmat', 'hmats', 'parent', 'root_seg']
    bootf = h5py.File(bootstrap_fname, 'r')
    origf = h5py.File(orig_fname, 'r')
    success = True
    try:
        for seg_name in origf: # do original action checks
            seg_name = str(seg_name)
            if seg_name not in bootf:
                print 'original action {} from {} not in {}'.format(seg_name, orig_fname, bootstrap_fname)
                success = False
            for lr in 'lr':
                if '{}_gripper_joint'.format(lr) not in bootf[seg_name]:
                    print 'boostrap file {} root segment {} missing {}_gripper_joint'.format(boostrap_fname, seg_name, lr)
                    success = False
        for seg_name, seg_info in bootf.iteritems():
            seg_name = str(seg_name)
            for k in required_keys:
                if k not in seg_info:
                    print 'bootstrap file {} segment {} missing key {}'.format(bootstrap_fname, seg_name, k)
            for lr in 'lr':
                if lr not in seg_info['hmats']:
                    print 'boostrap file {} segment {} missing {} hmats'.format(boostrap_fname, seg_name, lr)
                    success = False
            parent = str(seg_info['parent'][()])
            if parent not in bootf:
                print 'boostrap file {} missing parent {} for segment {}'.format(boostrap_fname, parent, seg_name)
                success = False
            parent_children = bootf[parent]['children'][()]
            if parent != seg_name and seg_name not in parent_children:
                print 'boostrap file {} parent {} does not have pointer to child {}'.format(boostrap_fname, parent, seg_name)
                success = False
            root_seg = str(seg_info['root_seg'][()])
            if root_seg not in bootf:
                print 'boostrap file {} missing root_seg {} for segment {}'.format(boostrap_fname, root_seg, seg_name)
                success = False
            root_n = bootf[root_seg]['cloud_xyz'][:].shape[0]
            seg_m = seg_info['cloud_xyz'][:].shape[0]
            if seg_info['cmat'][:].shape != (root_n, seg_m):
                print 'boostrap file {} cmat for segment {} has wrong dimension'.format(boostrap_fname, root_seg, seg_name)
                print 'is', seg_info['cloud_xyz'][:].shape[0], 'should be', (root_n, seg_m)
                success = False
    except:
        print 'encountered exception', sys.exc_info()
        bootf.close()
        origf.close()
        success = False
        raise
    return success

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


