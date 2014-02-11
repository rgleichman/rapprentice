import argparse, h5py, do_task, sys, dhm_utils, shutil, os

import os.path as osp
import IPython as ipy
import numpy as np
import math
import cloud
import cPickle as cp

from rapprentice import clouds
from rapprentice.colorize import colorize
from pdb import pm, set_trace

try:
    import do_task_floating
    reload(do_task_floating)
    from do_task_floating import sample_rope_state, TaskParameters, do_single_task
except:
    raise
    print "do_task_floating import failed, using do_task"
    from do_task import sample_rope_state
    from do_task import TaskParameters_floating as TaskParameters
    from do_task import do_single_task_floating as do_single_task


DS_SIZE = 0.03
DEFAULT_TREE_SIZES = [0, 30, 60, 90, 120]

def run_bootstrap(task_fname, action_fname, bootstrap_fname, burn_in = 40, tree_sizes = None, animate=False, no_cmat=False, warp_root=False):
    """
    generates a bootstrapping tree
    taskfile has the training examples to use
    bootstrap_fname will be used as the file to create all of the bootstrapping trees
    tree_sizes controls the number of trees we want to build
    results for tree size i will be in bootstrap_fname_i.h5
    """
    if not tree_sizes:
        tree_sizes = DEFAULT_TREE_SIZES[:]
    taskf = h5py.File(task_fname, 'r')    
    assert len(taskf) >= burn_in + max(tree_sizes)
    taskf.close()
    task_ctr = 0
    setup_bootstrap_file(action_fname, bootstrap_fname)
    bootstrap_orig = osp.splitext(bootstrap_fname)[0] + '_orig.h5'
    shutil.copyfile(bootstrap_fname, bootstrap_orig)
    results = []
    for i in range(burn_in):
        print 'doing burn in {}/{}'.format(i, burn_in)
        res = run_example((task_fname, str(task_ctr), bootstrap_orig, bootstrap_fname, animate, no_cmat, warp_root))
        results.append(res)
        task_ctr += 1
    for i in range(max(tree_sizes)):
        print 'doing bootstrapping {}/{}'.format(i, max(tree_sizes))
        if i in tree_sizes:
            bootstrap_i_fname = osp.splitext(bootstrap_fname)[0] + '_{}.h5'.format(i)
            shutil.copyfile(bootstrap_fname, bootstrap_i_fname)
        res = run_example((task_fname, str(task_ctr), bootstrap_fname, bootstrap_fname, animate, no_cmat, warp_root))
        results.append(res)
        task_ctr += 1
    print 'success rate', sum(results)/float(len(results))
    return sum(results)/float(len(results))

def run_example((task_fname, task_id, action_fname, bootstrap_fname, animate, no_cmat, warp_root)):
    """
    runs a knot-tie attempt for task_id (taken from taskfile
    possible actions are the expert demonstrations in actionfile
         assumed to be openable in 'r' mode
    if bootstrap_fname == '', then it won't save anything to that file
    set bootstrap_fname to add the results from the trial run to that file
         this is assumed to already be initialized
         this will append into that file and assumes that bootstrap_file has all the actions from actionfile in it         
    returns True if this is a knot-tie else returns False
    """
    taskfile = h5py.File(task_fname, 'r')
    init_xyz = taskfile[str(task_id)][:]
    taskfile.close()
    # currently set to test that correspondence trick does what we want
    task_params = TaskParameters(action_fname, init_xyz, animate=animate, no_cmat=no_cmat, warp_root=warp_root)
    task_results = do_single_task(task_params)
    if task_results['success'] and bootstrap_fname:
        try:
            bootf = h5py.File(bootstrap_fname, 'r+')
            for seg_values in task_results['seg_info']:
                cloud_xyz, parent, hmats, cmat = [seg_values[k] for k in ['cloud_xyz', 'parent', 'hmats', 'cmat']]
                children = []
                root_seg = str(bootf[str(parent)]['root_seg'][()])
                create_bootstrap_item(bootf, cloud_xyz, root_seg, str(parent), children, hmats, cmat)
        except:
            print 'encountered exception', sys.exc_info()
            print 'warning, bootstrap file may be malformed'
            raise
        finally:
            bootf.close()
    return task_results['success']


class CloudParams:
    def __init__(self):
        self.cmd_params      = None
        self.num_batches     = None
        self.start_batch_num = None
        self.end_batch_num   = None
        self.results_fname   = None
        self.env             = 'RSS3'
        self.vol             = 'iros_dat'
        self.core_type       = 'f2'

def create_test_params(local_task_fname, task_fname, action_fname, no_cmat):
    """
    The list of params returned by this is to be mapped to run_example
    """
    taskfile   = h5py.File(local_task_fname, 'r')
    ntasks     = len(taskfile.keys())
    taskfile.close()
    cmd_params = [(task_fname, i, action_fname, "", False, no_cmat) for i in xrange(ntasks)]
    return cmd_params


def run_tests_on_cloud(cloud_params, do_local=False):
    """
    make sure that task_fname and action_fname are available on the volume in the cloud.
    
    do_local : if true, the 'map' is done locally.
    """
    
    ntests     = len(cloud_params.cmd_params)
    batch_size = int(math.ceil(ntests/(cloud_params.num_batches+0.0)))

    batch_edges = batch_size*np.array(xrange(cloud_params.num_batches))[cloud_params.start_batch_num : cloud_params.end_batch_num]

    all_succ = []
    for i in xrange(len(batch_edges)):
        if i==len(batch_edges)-1:
            cmds = cloud_params.cmd_params[batch_edges[i]:]
        else:
            cmds = cloud_params.cmd_params[batch_edges[i]:min(batch_edges[i+1], ntests)]
        print colorize("calling on cloud : batch [%d/%d] "%(i, len(batch_edges)), "yellow", True)
        try:
            if not do_local:
                jids = cloud.map(run_example, cmds, _vol=cloud_params.vol, _env=cloud_params.env, _type=cloud_params.core_type)
                print colorize("\t submitted %d jobs"%len(jids), "yellow", False)
                succ = cloud.result(jids)
                print colorize("\t got results for batch %d/%d "%(i, len(batch_edges)), "green", True)
            else:
                succ = map(run_example, cmds)
            all_succ += succ
        except Exception as e:
            print "Found exception %s. Not saving data for this demo."%e

    with open(cloud_params.results_fname, 'w') as f:
        print colorize("\t\t saved results in : %s"%cloud_params.results_fname, "green")
        cp.dump(all_succ, f)


def test_bootrun(bootrun_name='boot_1', do_nn=False, tree_sizes=[30,60,90,120], test_fname="final_test_set.h5", no_cmat=False):
    """
    @ res_dir       : the directory where the results from the test runs will be saved.
                      the saved results will be like: 
                         res_dir/<bootrun_name>_<tree_size>_res.cp
    @ bootrun_name  : the directory holding the actions from the bootstrap runs.
    @ do_nn         : the action file here is just the nearest neighbor file [use this for baseline]
    @ tree_sizes    : the sizes of bootstrap trees to run tests on.
    """
    cloud_bootstrapping_dir = "/home/picloud/sandbox/bootstrapping"
    data_dir           = osp.join(os.getenv("BOOTSTRAPPING_DIR"), "data")
    local_task_fname   = osp.join(data_dir, test_fname)
    task_fname         = osp.join(cloud_bootstrapping_dir, "data/%s"%test_fname) 

    if not do_nn:
        result_fnames      = [osp.join(data_dir, "test_results", "%s_%d_result.cp"%(bootrun_name, s)) for s in tree_sizes]
        test_action_fnames = [osp.join(cloud_bootstrapping_dir, "data", bootrun_name, 'test_bootstrapping_%d.h5'%s) for s in tree_sizes]
    else:
        test_basename      = osp.splitext(osp.basename(test_fname))[0]
        result_fnames      = [osp.join(data_dir, "test_results", "%s_%s_nn_result.cp"%(bootrun_name, test_basename))]
        test_action_fnames = [osp.join(cloud_bootstrapping_dir, "data", bootrun_name, 'test_bootstrapping_orig.h5')]


    for i in xrange(len(test_action_fnames)):
        cmd_params      = create_test_params(local_task_fname, task_fname, test_action_fnames[i], no_cmat)
        print colorize(" SUBMITTING %d jobs to run on the cloud"%len(cmd_params), "red", True)
        cloud_params    =  CloudParams()
        cloud_params.cmd_params      = cmd_params
        cloud_params.num_batches     = 1
        cloud_params.start_batch_num = 0
        cloud_params.end_batch_num   = 1 ## exclusive
        cloud_params.results_fname   = result_fnames[i]
        cloud_params.env             = 'RSS3'
        cloud_params.vol             = 'iros_dat'
        cloud_params.core_type       = 'f2'

        print colorize("running tests for file: %s"%test_action_fnames[i], "magenta", True)
        run_tests_on_cloud(cloud_params, False)


def setup_bootstrap_file(action_fname, bootstrap_fname):
    """
    copies over the relevant fields of action file to a bootstrap_file so that we can use the 
    resulting file for run_example
    """
    print action_fname, bootstrap_fname
    actfile = h5py.File(action_fname, 'r')
    bootfile = h5py.File(bootstrap_fname, 'w')
    for seg_name, seg_info in actfile.iteritems():
        seg_name = str(seg_name)
        cloud_xyz = clouds.downsample(seg_info['cloud_xyz'][:], DS_SIZE)
        hmats = dict((lr, seg_info['{}_gripper_tool_frame'.format(lr)]['hmat'][:]) for lr in 'lr')
        cmat = np.eye(cloud_xyz.shape[0])
        gripper_joints = dict(('{}_gripper_joint'.format(lr), seg_info['{}_gripper_joint'.format(lr)][:]) for lr in 'lr')
        create_bootstrap_item(outfile=bootfile, cloud_xyz=cloud_xyz, root_seg=seg_name, parent=seg_name,
                              children=[], hmats=hmats, cmat=cmat, other_items=gripper_joints,
                              update_parent=False, seg_name=seg_name)
    actfile.close()
    assert check_bootstrap_file(bootstrap_fname, action_fname)
    return bootfile

def create_bootstrap_item(outfile, cloud_xyz, root_seg, parent, children, hmats, 
                          cmat, other_items = None, update_parent=True, seg_name=None):
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
        parent_children = outfile[parent]['children'][()]
        del outfile[parent]['children']
        if parent_children == 0:
            parent_children = []
        parent_children = np.append(parent_children, [seg_name])
        outfile[parent]['children'] = parent_children
    if other_items:
        for k, v in other_items.iteritems():
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
                    print 'boostrap file {} root segment {} missing {}_gripper_joint'.format(bootstrap_fname, seg_name, lr)
                    success = False
        for seg_name, seg_info in bootf.iteritems():
            seg_name = str(seg_name)
            for k in required_keys:
                if k not in seg_info:
                    print 'bootstrap file {} segment {} missing key {}'.format(bootstrap_fname, seg_name, k)
            for lr in 'lr':
                if lr not in seg_info['hmats']:
                    print 'boostrap file {} segment {} missing {} hmats'.format(bootstrap_fname, seg_name, lr)
                    success = False
            parent = str(seg_info['parent'][()])
            if parent not in bootf:
                print 'boostrap file {} missing parent {} for segment {}'.format(bootstrap_fname, parent, seg_name)
                success = False
            parent_children = bootf[parent]['children'][()]
            if parent != seg_name and seg_name not in parent_children:
                print 'boostrap file {} parent {} does not have pointer to child {}'.format(bootstrap_fname, parent, seg_name)
                success = False
            root_seg = str(seg_info['root_seg'][()])
            if root_seg not in bootf:
                print 'boostrap file {} missing root_seg {} for segment {}'.format(bootstrap_fname, root_seg, seg_name)
                success = False
            root_n = bootf[root_seg]['cloud_xyz'][:].shape[0]
            seg_m = seg_info['cloud_xyz'][:].shape[0]
            if seg_info['cmat'][:].shape != (root_n, seg_m):
                print 'boostrap file {} cmat for segment {} has wrong dimension'.format(bootstrap_fname, root_seg, seg_name)
                print 'is', seg_info['cloud_xyz'][:].shape[0], 'should be', (root_n, seg_m)
                success = False
    except:
        print 'encountered exception', sys.exc_info()
        bootf.close()
        origf.close()
        success = False
        raise
    return success

def gen_rot_sequence_task_file(taskfname, actionfname):
    """
    draw a sequence of training examples that consider rotating the pt clouds
    rotation is initially and introduced so that we explore the manifold better
    generates 200 samples
    """
    min_theta = np.pi/8
    max_theta = np.pi
    burnin_length = 50
    num_samples = 200
    theta_vals = np.linspace(min_theta, max_theta, num_samples - burnin_length)
    min_rad, max_rad = 0.1, 0.1
    num_perturb_pts = 7
    taskfile = h5py.File(taskfname, 'w')
    actionfile = h5py.File(actionfname, 'r')
    try:
        for i in range(burnin_length):
            dhm_utils.one_l_print('Creating State {}/{}'.format(i, num_samples))
            with dhm_utils.suppress_stdout():
                taskfile[str(i)] = sample_rope_state(actionfile, perturb_points=num_perturb_pts,
                                                        min_rad=min_rad, max_rad=max_rad, rotation = min_theta)
        for j in range(burnin_length, num_samples):
            dhm_utils.one_l_print('Creating State {}/{}'.format(j, num_samples))
            print j, theta_vals[j-burnin_length]
            with dhm_utils.suppress_stdout():
                taskfile[str(j)] = sample_rope_state(actionfile, perturb_points=num_perturb_pts,
                                                        min_rad=min_rad, max_rad=max_rad, rotation = theta_vals[j-burnin_length])
        print ''
    except:
        print 'encountered exception', sys.exc_info()
        raise
    finally:                
        taskfile.close()
        actionfile.close()
    
    

def gen_task_file(taskfname, num_examples, actionfname, perturb_bounds=None, num_perturb_pts=7):
    """
    draw num_examples states from the initial state distribution defined by
    do_task.sample_rope_state
    using intial states from actionfile

    writes results to fname
    """
    if not perturb_bounds:
        min_rad, max_rad = 0.1, 0.1
    else:
        min_rad, max_rad = perturb_bounds
    taskfile = h5py.File(taskfname, 'w')
    actionfile = h5py.File(actionfname, 'r')
    try:
        for i in range(num_examples):
            dhm_utils.one_l_print('Creating State {}/{}'.format(i, num_examples))
            with dhm_utils.suppress_stdout():
                taskfile[str(i)] = sample_rope_state(actionfile, perturb_points=num_perturb_pts,
                                                        min_rad=min_rad, max_rad=max_rad, rotation=np.pi/4)
        print ''
    except:
        print 'encountered exception', sys.exc_info()
        raise
    finally:                
        taskfile.close()
        actionfile.close()
    assert check_task_file(taskfname)

def check_task_file(fname):
    """
    probably unecessary, but checks that a task file is properly labelled sequentially
    """
    f = h5py.File(fname, 'r')
    success = True
    for i in range(len(f)):
        if str(i) not in f:
            print 'task file {} is missing key {}'.format(fname, i)
            success = False
    f.close()
    return success


def run_example_test():
    boot_fname = 'data/test_bootstrapping.h5'
    try:
        os.remove(boot_fname)
    except:
        pass
    act_fname = 'data/actions.h5'
    task_fname = 'data/test_tasks.h5'
    return run_bootstrap(task_fname, act_fname, boot_fname, burn_in = 5, tree_sizes = [0])


def main():
    args = parse_arguments()
    boot_dir = args.bootstrapping_directory
    #TODO  Should the boot_fname be different?
    boot_fname_leaf = 'boot.h5'
    if args.tree_sizes:
        boot_fname_leaf = 'boot_{}.h5'.format(max(args.tree_sizes))
    boot_fname = osp.join(boot_dir, boot_fname_leaf)
    try:
        os.remove(boot_fname)
    except:
        pass
    act_fname = args.actions_file
    task_fname = osp.join(boot_dir, 'tasks.h5')
    try:
        good_task_file = check_task_file(task_fname)
        if not good_task_file:
            raise
    except:
        gen_task_file(task_fname, 200, act_fname)
    if args.burn_in is not None:
        burn_in = args.burn_in
    else:
        burn_in = 40
    if args.tree_sizes:
        tree_sizes = args.tree_sizes
    else:
        tree_sizes = None
    success_rate = run_bootstrap(task_fname, act_fname, boot_fname, burn_in=burn_in, tree_sizes=tree_sizes, animate=args.animate, no_cmat=args.no_cmat, warp_root=args.warp_root)
    import cPickle as cp
    res_fname = osp.join(boot_dir, 'res.cp')
    with open(res_fname, 'w') as f:
        cp.dump({'success_rate':success_rate, 'args':args}, f)
    return success_rate


def parse_arguments():
    import argparse

    usage = """
    Run {0} --help for a list of arguments
    Warning: This may modify existing hdf5 files.
    The task file should be in bootstrapping_directory/tasks.h5
    """.format(sys.argv[0])

    parser = argparse.ArgumentParser(usage=usage)
    parser.add_argument("actions_file", type=str,
                        help="The file that contains the original (probably human) demonstrations.")
    parser.add_argument("bootstrapping_directory", type=str,
                        help="The directory that contains or will contain the learned bootstrapped h5 files.")
    parser.add_argument("--animate", action='store_true', help='If included, then it will show the animation')
    parser.add_argument("--burn_in", type=int, default=None,
                        help="The number of burn-in iterations to run. The burn-in iterations only uses original segments")
    parser.add_argument("--tree_sizes", type=int, nargs="+", default=None,
                        help="A space separated list of the number of bootstrapping iterations each bootstrap file should be created from")
    parser.add_argument("--no_cmat", action='store_true', help="If included, then the cmat will not be used when warping to the root.")
    parser.add_argument("--warp_root", action='store_true', help ="If included, then segments will per warped from with the root of the closest segemnt tree.")
    args = parser.parse_args()
    print "args =", args
    return args

def testing_main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--bootstrap_name", type=str,
                        help="name of the bootstrap directory like : boot_1, boot_2, ...")
    parser.add_argument("--baseline", action='store_true', help='If included, then only the original segments will be chosen [for baseline].')
    parser.add_argument("--tree_sizes", type=int, nargs="+", default=[30,60,90,120],
                        help="A space separated list of the number of bootstrapping iterations each bootstrap file should be created from")
    
    parser.add_argument("--test_fname", type=str, default="final_test_set.h5",
                        help="name of test initial states file.")
    parser.add_argument("--no_cmat", action='store_true')

    args = parser.parse_args()
    print args.tree_sizes
    test_bootrun(args.bootstrap_name, args.baseline, args.tree_sizes, test_fname=args.test_fname, no_cmat=args.no_cmat)


if __name__ == "__main__":
    main()
    #testing_main()


