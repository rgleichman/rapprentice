#!/usr/bin/env python
###################
"""
Workflow:
1. Fake data + animation only
    --fake_data_segment=xxx --execution=0
2. Fake data + Gazebo. Set Gazebo to initial state of fake data segment so we'll execute the same thing.
    --fake_data_segment=xxx --execution=1
    This is just so we know the robot won't do something stupid that we didn't catch with openrave only mode.
3. Real data + Gazebo
    --execution=1
    The problem is that the gazebo robot is in a different state from the real robot, in particular, the head tilt angle. TODO: write a script that       sets gazebo head to real robot head
4. Real data + Real execution.
    --execution=1

The question is, do you update the robot's head transform.
If you're using fake data, don't update it.

"""
from rapprentice import registration, colorize, \
     animate_traj, ros2rave, plotting_openrave, task_execution, \
     planning, func_utils, resampling, ropesim, rope_initialization, clouds
from rapprentice import math_utils as mu

import trajoptpy
import openravepy
import numpy as np
import h5py
from numpy import asarray
import atexit
import time

#Don't use args, use globals
#args = None

class Globals:
    robot = None
    env = None
    pr2 = None
    sim = None
    exec_log = None
    viewer = None
    random_seed = None
    
class BijArgs:
    n_iter = 50
    bend_init = 0.1
    bend_final = 0.01
    rad_init = 0.5
    rad_final = 0.01
    rot_reg = 1
    #corr was 1
    corr_reg = 0.5
    #out was 2
    outliersd = 3
    reg_init = 1
    reg_final = .01
    

class RopeState:
    def __init__(self, segment, perturb_radius, perturb_num_points):
        self.segment = segment
        self.perturb_radius = perturb_radius
        self.perturb_num_points = perturb_num_points

class TaskParameters:
    def __init__(self, demofile_name,  knot, animate, max_steps_before_failure, choose_segment, log_name):
        self.demofile_name = demofile_name
        self.knot = knot
        self.animate = animate
        self.max_steps_before_failure = max_steps_before_failure
        self.choose_segment = choose_segment
        self.log_name = log_name
        self.random_seed = None

#init_rope_state_segment, perturb_radius, perturb_num_points
def redprint(msg):
    """Print the message to the console in red, bold font."""
    print colorize.colorize(msg, "red", bold=True)

def split_trajectory_by_gripper(seg_info):
    """Split up the trajectory into sections with breaks occuring when the grippers open or close.

    Return: (seg_starts, seg_ends)

    """
    rgrip = asarray(seg_info["r_gripper_joint"])
    lgrip = asarray(seg_info["l_gripper_joint"])

    thresh = .04  # open/close threshold

    n_steps = len(lgrip)

    # indices BEFORE transition occurs
    l_openings = np.flatnonzero((lgrip[1:] >= thresh) & (lgrip[:-1] < thresh))
    r_openings = np.flatnonzero((rgrip[1:] >= thresh) & (rgrip[:-1] < thresh))
    l_closings = np.flatnonzero((lgrip[1:] < thresh) & (lgrip[:-1] >= thresh))
    r_closings = np.flatnonzero((rgrip[1:] < thresh) & (rgrip[:-1] >= thresh))

    before_transitions = np.r_[l_openings, r_openings, l_closings, r_closings]
    after_transitions = before_transitions + 1
    seg_starts = np.unique(np.r_[0, after_transitions])
    seg_ends = np.unique(np.r_[before_transitions, n_steps - 1])

    return seg_starts, seg_ends


def binarize_gripper(angle):
    thresh = .04
    return angle > thresh

def set_gripper_sim(lr, is_open, prev_is_open, animate=True):
    """Opens or closes the gripper. Also steps the simulation?

    Arguments:
        is_open -- boolean that is true if the gripper should be open
        prev_is_open -- boolean that is true if the gripper was open last step
    May send an open command if is_open is true but prev_is_open is false.

    Return False if the simulated gripper failed to grab the rope, eles return True.
    """
    mult = 5
    open_angle = .08 * mult
    closed_angle = .02 * mult

    target_val = open_angle if is_open else closed_angle

    #elif args.simulation:
    # release constraints if necessary
    if is_open and not prev_is_open:
        Globals.sim.release_rope(lr)

    # execute gripper open/close trajectory
    joint_ind = Globals.robot.GetJoint("%s_gripper_l_finger_joint" % lr).GetDOFIndex()
    start_val = Globals.robot.GetDOFValues([joint_ind])[0]
    #gripper_velocity = 0.2
    #a smaller number makes the gripper move slower.
    #if the gripper moves slower then it will fling the rope less.
    gripper_velocity = 0.005
    joint_traj = np.linspace(start_val, target_val, np.ceil(abs(target_val - start_val) / gripper_velocity))
    #print "joint_traj after retime =", joint_traj
    for val in joint_traj:
        Globals.robot.SetDOFValues([val], [joint_ind])
        Globals.sim.step()
        if animate:
            Globals.viewer.Step()
    # add constraints if necessary
    #TODO find out what grab_rope does. Add the return value to doctstring
    if not is_open and prev_is_open:
        if not Globals.sim.grab_rope(lr):
            return False
    return True


def unwrap_arm_traj_in_place(traj):
    assert traj.shape[1] == 7
    for i in [2, 4, 6]:
        traj[:, i] = np.unwrap(traj[:, i])
    return traj


def unwrap_in_place(t):
    # TODO: do something smarter than just checking shape[1]
    if t.shape[1] == 7:
        unwrap_arm_traj_in_place(t)
    elif t.shape[1] == 14:
        unwrap_arm_traj_in_place(t[:, :7])
        unwrap_arm_traj_in_place(t[:, 7:])
    else:
        raise NotImplementedError

def exec_traj_sim(bodypart2traj, animate):
    def sim_callback(i):
        Globals.sim.step()
    #if args.animation or args.simulation:
    dof_inds = []
    trajs = []
    for (part_name, traj) in bodypart2traj.items():
        manip_name = {"larm": "leftarm", "rarm": "rightarm"}[part_name]
        dof_inds.extend(Globals.robot.GetManipulator(manip_name).GetArmIndices())
        trajs.append(traj)
    full_traj = np.concatenate(trajs, axis=1)
    Globals.robot.SetActiveDOFs(dof_inds)

    #if args.simulation:
    # make the trajectory slow enough for the simulation
    #orig full_traj = ropesim.retime_traj(Globals.robot, dof_inds, full_traj)
    full_traj = ropesim.retime_traj(Globals.robot, dof_inds, full_traj, max_cart_vel=.01)

    # in simulation mode, we must make sure to gradually move to the new starting position
    curr_vals = Globals.robot.GetActiveDOFValues()
    transition_traj = np.r_[[curr_vals], [full_traj[0]]]
    unwrap_in_place(transition_traj)
    transition_traj = ropesim.retime_traj(Globals.robot, dof_inds, transition_traj, max_cart_vel=.05)
    #transition_traj = ropesim.retime_traj(Globals.robot, dof_inds, transition_traj, max_cart_vel=.005)
    animate_traj.animate_traj(transition_traj, Globals.robot, restore=False, pause=False,
        callback=sim_callback, step_viewer=animate)
    full_traj[0] = transition_traj[-1]
    unwrap_in_place(full_traj)

    animate_traj.animate_traj(full_traj, Globals.robot, restore=False, pause=False,
        callback=sim_callback, step_viewer=animate)
    return True

def registration_cost(xyz0, xyz1):
    scaled_xyz0, _ = registration.unit_boxify(xyz0)
    scaled_xyz1, _ = registration.unit_boxify(xyz1)
    #TODO: fix parameters
    #f, g = registration.tps_rpm_bij(scaled_xyz0, scaled_xyz1, rot_reg=1e-3, n_iter=10)
    f, g = registration.tps_rpm_bij(scaled_xyz0, scaled_xyz1, rot_reg=1e-3, 
                                    n_iter=10, reg_init=BijArgs.reg_init, 
                                    reg_final=BijArgs.reg_final, rad_init = BijArgs.rad_init, 
                                    rad_final = BijArgs.rad_final, outliersd = BijArgs.outliersd, 
                                    corr_reg=BijArgs.corr_reg)
    cost = registration.tps_reg_cost(f) + registration.tps_reg_cost(g)
    return cost

DS_SIZE = .025

@func_utils.once
def get_downsampled_clouds(demofile):
    return [clouds.downsample(seg["cloud_xyz"], DS_SIZE) for seg in demofile.values()]


def remove_inds(a, inds):
    return [x for (i, x) in enumerate(a) if i not in inds]

def find_closest_manual(demofile, _new_xyz):
    "for now, just prompt the user"
    seg_names = demofile.keys()
    print "choose from the following options (type an integer)"
    for (i, seg_name) in enumerate(seg_names):
        print "%i: %s" % (i, seg_name)
    choice_ind = task_execution.request_int_in_range(len(seg_names))
    chosen_seg = seg_names[choice_ind]
    return chosen_seg

def find_closest_auto(demofile, new_xyz):
    """Return the segment with the lowest warping cost. Takes about 2 seconds."""
    parallel = True
    if parallel:
        from joblib import Parallel, delayed

    keys = demofile.keys()
    ds_clouds = get_downsampled_clouds(demofile)
    ds_new = clouds.downsample(new_xyz, DS_SIZE)
    if parallel:
        before = time.time()
        costs = Parallel(n_jobs=12, verbose=0)(delayed(registration_cost)(ds_cloud, ds_new) for ds_cloud in ds_clouds)
        after = time.time()
        print "Parallel registration time in seconds =", after - before
    else:
        costs = []
        for (i, ds_cloud) in enumerate(ds_clouds):
            costs.append(registration_cost(ds_cloud, ds_new))
            print(("completed %i/%i" % (i + 1, len(ds_clouds))))
    print(("costs\n", costs))
    ibest = np.argmin(costs)
    return keys[ibest]


def arm_moved(joint_traj):
    if len(joint_traj) < 2:
        return False
    return ((joint_traj[1:] - joint_traj[:-1]).ptp(axis=0) > .01).any()


def tpsrpm_plot_cb(x_nd, y_md, targ_Nd, corr_nm, wt_n, f, old_xyz, new_xyz, last_one=False):
    _, src_params = registration.unit_boxify(old_xyz)
    _, targ_params = registration.unit_boxify(new_xyz)
    f = registration.unscale_tps(f, src_params, targ_params)
    #ypred_nd = f.transform_points(x_nd)
    handles = []
    #handles.append(Globals.env.plot3(ypred_nd, 3, (0, 1, 0, 1)))
    ypred_nd = f.transform_points(old_xyz)
    handles.append(Globals.env.plot3(ypred_nd, 3, (0, 1, 0, 1)))
    handles.extend(plotting_openrave.draw_grid(Globals.env, f.transform_points, old_xyz.min(axis=0) - np.r_[0, 0, .1], old_xyz.max(axis=0)+ np.r_[0, 0, .1], xres=.1, yres=.1, zres=.04))

    if Globals.viewer:
        Globals.viewer.Step()
        time.sleep(0.1)
        #Globals.viewer.Idle()
        
def load_segment(demofile, segment, fake_data_transform=[0, 0, 0, 0, 0, 0]):
    fake_seg = demofile[segment]
    new_xyz = np.squeeze(fake_seg["cloud_xyz"])
    hmat = openravepy.matrixFromAxisAngle(fake_data_transform[3:6])
    hmat[:3, 3] = fake_data_transform[0:3]
    new_xyz = new_xyz.dot(hmat[:3, :3].T) + hmat[:3, 3][None, :]
    r2r = ros2rave.RosToRave(Globals.robot, asarray(fake_seg["joint_states"]["name"]))
    return new_xyz, r2r

def unif_resample(traj, max_diff, wt=None):
    """
    Resample a trajectory so steps have same length in joint space
    """
    import scipy.interpolate as si
    tol = .005
    if wt is not None:
        wt = np.atleast_2d(wt)
        traj = traj * wt

    dl = mu.norms(traj[1:] - traj[:-1], 1)
    l = np.cumsum(np.r_[0, dl])
    goodinds = np.r_[True, dl > 1e-8]
    deg = min(3, sum(goodinds) - 1)
    if deg < 1:
        return traj, np.arange(len(traj))

    nsteps = max(int(np.ceil(float(l[-1]) / max_diff)), 2)
    newl = np.linspace(0, l[-1], nsteps)

    ncols = traj.shape[1]
    colstep = 10
    traj_rs = np.empty((nsteps, ncols))
    for istart in xrange(0, traj.shape[1], colstep):
        (tck, _) = si.splprep(traj[goodinds, istart:istart + colstep].T, k=deg, s=tol ** 2 * len(traj), u=l[goodinds])
        traj_rs[:, istart:istart + colstep] = np.array(si.splev(newl, tck)).T
    if wt is not None:
        traj_rs = traj_rs / wt

    newt = np.interp(newl, l, np.arange(len(traj)))
    return traj_rs, newt

#<diffuseColor>.96 .87 .70</diffuseColor>
def make_table_xml(translation, extents):
    xml = """
<Environment>
  <KinBody name="table">
    <Body type="static" name="table_link">
      <Geom type="box">
        <Translation>%f %f %f</Translation>
        <extents>%f %f %f</extents>
        <diffuseColor>.96 .87 .70</diffuseColor>
      </Geom>
    </Body>
  </KinBody>
</Environment>
""" % (translation[0], translation[1], translation[2], extents[0], extents[1], extents[2])
    return xml

PR2_L_POSTURES = dict(
    untucked=[0.4, 1.0, 0.0, -2.05, 0.0, -0.1, 0.0],
    tucked=[0.06, 1.25, 1.79, -1.68, -1.73, -0.10, -0.09],
    up=[0.33, -0.35, 2.59, -0.15, 0.59, -1.41, -0.27],
    side=[1.832, -0.332, 1.011, -1.437, 1.1, -2.106, 3.074]
)


def mirror_arm_joints(x):
    "mirror image of joints (r->l or l->r)"
    return np.r_[-x[0], x[1], -x[2], x[3], -x[4], x[5], -x[6]]
###################

#TODO: Change for no body
def move_sim_arms_to_side():
    """Moves the simulated arms to the side."""
    #SetDOFValues sets the joint angles. DOF = degree of feedom
    #Move the arms back ("side" posture)
    Globals.robot.SetDOFValues(PR2_L_POSTURES["side"], Globals.robot.GetManipulator("leftarm").GetArmIndices())
    Globals.robot.SetDOFValues(mirror_arm_joints(PR2_L_POSTURES["side"]), Globals.robot.GetManipulator("rightarm").GetArmIndices())

def do_several_segments(demofile_name, init_rope_state_segment, perturb_radius, perturb_num_points, segments, knot='K3a1', animate=True, filename=None):
    """Execute on a random rope state several segments.
    Arguments:
        segments -- A list of the demonstration segments to execute.
    """
    setup_log(filename)
    demofile, new_xyz = setup_and_return_demofile(demofile_name, init_rope_state_segment, perturb_radius, perturb_num_points, animate)
    results = []
    for i, segment in enumerate(segments):
        results.append(loop_body(new_xyz, demofile, (lambda _,__: segment), knot, animate, curr_step=i))
    return results

def do_single_random_task(rope_state, task_params):
    """Manually choose the segment.
    Do one task.

    Arguments:
    rope_state -- a RopeState object
    task_params -- a task_params object

    If task_parms.max_steps_before failure is -1, then it loops until the knot is detected.
    
    """
    filename = task_params.log_name
    demofile_name = task_params.demofile_name
    init_rope_state_segment = rope_state.segment
    perturb_radius = rope_state.perturb_radius
    perturb_num_points = rope_state.perturb_num_points
    animate = task_params.animate
    max_steps_before_failure = task_params.max_steps_before_failure
    choose_segment = task_params.choose_segment
    knot = task_params.knot
    
    ### Setup ###
    setup_random(task_params)
    setup_log(filename)
    demofile, new_xyz = setup_and_return_demofile(demofile_name, init_rope_state_segment, perturb_radius, perturb_num_points, animate=animate)
    results = []
    i = 0
    while True:
        print "max_steps_before_failure =", max_steps_before_failure
        print "i =", i
        if max_steps_before_failure != -1 and i >= max_steps_before_failure:
            break
        result = loop_body(new_xyz, demofile, choose_segment, knot, animate, curr_step=i)
        results.append(result)
        if result:
            break
        i += 1
    return results
def setup_random(task_params):
    if task_params.random_seed:
        Globals.random_seed = task_params.random_seed
        print "Found a random seed"
        print "Random seed is", Globals.random_seed
        
def setup_log(filename):
    if filename:
        if Globals.exec_log is None:
            redprint("Writing log to file %s" % filename)
            Globals.exec_log = task_execution.ExecutionLog(filename)
            #This will flush to the log when the program closes.
            atexit.register(Globals.exec_log.close)
    
#TODO  Consider encapsulating these intermedite return values in a class.
def setup_and_return_demofile(demofile_name, init_rope_state_segment, perturb_radius, perturb_num_points, animate):
    """For the simulation, this code runs before the main loop. Also sets the random seed"""
    if Globals.random_seed is not None:
        np.random.seed(Globals.random_seed)
        
    demofile = h5py.File(demofile_name, 'r')
    Globals.env = openravepy.Environment()
    Globals.env.StopSimulation()
    #TODO: fix this
    try:
        Globals.env.Load("robots/pr2-beta-static.zae")
        Globals.robot = Globals.env.GetRobots()[0]
    except:
        for i in range(5):
            time.sleep(5)
            Globals.env.Load("robots/pr2-beta-static.zae")
            Globals.robot = Globals.env.GetRobots()[0]
    
    #if args.simulation:
    #Set up the simulation with the table (no rope yet)
    new_xyz, _ = load_segment(demofile, init_rope_state_segment)
    #table_height = new_xyz[:, 2].mean() - .02
    table_height = new_xyz[:, 2].mean() - 0.17
    table_xml = make_table_xml(translation=[1, 0, table_height], extents=[.85, .55, .01])
    if animate:
        Globals.viewer = trajoptpy.GetViewer(Globals.env)
    Globals.env.LoadData(table_xml)
    Globals.sim = ropesim.Simulation(Globals.env, Globals.robot)

        # create rope with optional pertubations
    #Globals.exec_log(curr_step, "acquire_cloud.orig_cloud", new_xyz)
    move_sim_arms_to_side()
    rope_nodes = rope_initialization.find_path_through_point_cloud(
        new_xyz,
        perturb_peak_dist=perturb_radius,
        num_perturb_points=perturb_num_points)
    #Globals.exec_log(curr_step, "acquire_cloud.init_sim_rope_nodes", rope_nodes)
    Globals.sim.create(rope_nodes)
    return demofile, new_xyz

def loop_body(new_xyz, demofile, choose_segment, knot, animate, curr_step=None):
    """Do the body of the main task execution loop (ie. do a segment). 
    Arguments:
        curr_step is 0 indexed
        choose_segment is a function that returns the key in the demofile to the segment
        knot is the knot the rope is checked against
        new_xyz is the new pointcloud
    """
    #TODO -- End condition
    #TODO -- max_segments logic
    redprint("Acquire point cloud")
    #if args.simulation:
    
    move_sim_arms_to_side()
    #TODO -- Possibly refactor this section to be before the loop.

    new_xyz = Globals.sim.observe_cloud()
    segment = choose_segment(demofile, new_xyz)
    seg_info = demofile[segment]
    
    handles = []
    old_xyz = np.squeeze(seg_info["cloud_xyz"])
    handles.append(Globals.env.plot3(old_xyz, 5, (1, 0, 0)))
    handles.append(Globals.env.plot3(new_xyz, 5, (0, 0, 1)))

    old_xyz = clouds.downsample(old_xyz, DS_SIZE)
    new_xyz = clouds.downsample(new_xyz, DS_SIZE)

    scaled_old_xyz, src_params = registration.unit_boxify(old_xyz)
    scaled_new_xyz, targ_params = registration.unit_boxify(new_xyz)
    #f, _ = registration.tps_rpm_(scaled_old_xyz, scaled_new_xyz, plot_cb=tpsrpm_plot_cb,
    #                              plotting=5 if animate else 0, rot_reg=np.r_[1e-4, 1e-4, 1e-1], n_iter=50, reg_init=10, reg_final=.01, old_xyz=old_xyz, new_xyz=new_xyz)
    #TODO: Fix plotting
    f, _ = registration.tps_rpm_bij(scaled_old_xyz, scaled_new_xyz, plot_cb=tpsrpm_plot_cb,
                             plotting=0 if animate else 0, rot_reg=np.r_[1e-4, 1e-4, 1e-1], n_iter=50, reg_init=BijArgs.reg_init, 
                                    reg_final=BijArgs.reg_final, rad_init=BijArgs.rad_init, 
                                    rad_final=BijArgs.rad_final, outliersd=BijArgs.outliersd, 
                                    corr_reg=BijArgs.corr_reg)
    
    #tps_rpm_bij(x_nd, y_md, n_iter = 20, reg_init = .1, reg_final = .001, rad_init = .1, rad_final = .01, rot_reg = 1e-3, 
    #        plotting = False, plot_cb = None, outliersd = 2., corr_reg=.5, update_rot_target=False):
    
    
    f = registration.unscale_tps(f, src_params, targ_params)
    #Globals.exec_log(curr_step, "gen_traj.f", f)

    handles.extend(plotting_openrave.draw_grid(Globals.env, f.transform_points, old_xyz.min(axis=0) - np.r_[0, 0, .1], old_xyz.max(axis=0) + np.r_[0, 0, .1], xres=.1, yres=.1, zres=.04))

    link2eetraj = {}
    #link2eetraj is a hash of gripper fram to new trajectory

    #Transform the gripper trajectory here
    for lr in 'lr':
        link_name = "%s_gripper_tool_frame" % lr
        #old_ee_traj is the old gripper trajectory
        old_ee_traj = asarray(seg_info[link_name]["hmat"])
        #new_ee_traj is the transformed gripper trajectory
        new_ee_traj = f.transform_hmats(old_ee_traj)
        #What is link2eetraj?
        link2eetraj[link_name] = new_ee_traj

        #Draw the old and new gripper trajectories as lines
        handles.append(Globals.env.drawlinestrip(old_ee_traj[:, :3, 3], 2, (1, 0, 0, 1)))
        handles.append(Globals.env.drawlinestrip(new_ee_traj[:, :3, 3], 2, (0, 1, 0, 1)))
    #Globals.exec_log(curr_step, "gen_traj.link2eetraj", link2eetraj)

    miniseg_starts, miniseg_ends = split_trajectory_by_gripper(seg_info)
    success = True
    #print colorize.colorize("mini segments:", "red"), miniseg_starts, miniseg_ends

    #TODO: modify for no body
    for (i_miniseg, (i_start, i_end)) in enumerate(zip(miniseg_starts, miniseg_ends)):
        ################################
        redprint("Generating joint trajectory for segment %s, part %i" % (segment, i_miniseg))

        # figure out how we're gonna resample stuff
        lr2oldtraj = {}
        for lr in 'lr':
            manip_name = {"l": "leftarm", "r": "rightarm"}[lr]
            #old_joint_traj is the old trajectory inside the minisegment
            old_joint_traj = asarray(seg_info[manip_name][i_start: i_end + 1])
            #print (old_joint_traj[1:] - old_joint_traj[:-1]).ptp(axis=0), i_start, i_end
            if arm_moved(old_joint_traj):
                lr2oldtraj[lr] = old_joint_traj
        if len(lr2oldtraj) > 0:
            old_total_traj = np.concatenate(lr2oldtraj.values(), 1)
            JOINT_LENGTH_PER_STEP = .1
            _, timesteps_rs = unif_resample(old_total_traj, JOINT_LENGTH_PER_STEP)
        ####

        ### Generate fullbody traj
        bodypart2traj = {}
        for (lr, old_joint_traj) in lr2oldtraj.items():
            manip_name = {"l": "leftarm", "r": "rightarm"}[lr]

            old_joint_traj_rs = mu.interp2d(timesteps_rs, np.arange(len(old_joint_traj)), old_joint_traj)

            ee_link_name = "%s_gripper_tool_frame" % lr
            new_ee_traj = link2eetraj[ee_link_name][i_start: i_end + 1]
            new_ee_traj_rs = resampling.interp_hmats(timesteps_rs, np.arange(len(new_ee_traj)), new_ee_traj)
            #if args.execution:
            #    Globals.pr2.update_rave()
            #Call the planner (eg. trajopt)
            new_joint_traj = planning.plan_follow_traj(Globals.robot, manip_name,
            Globals.robot.GetLink(ee_link_name), new_ee_traj_rs, old_joint_traj_rs)
            part_name = {"l": "larm", "r": "rarm"}[lr]
            bodypart2traj[part_name] = new_joint_traj

        ### Execute the gripper ###
        redprint("Executing joint trajectory for segment %s, part %i using arms '%s'" % (segment, i_miniseg, bodypart2traj.keys()))

        for lr in 'lr':
            gripper_open = binarize_gripper(seg_info["%s_gripper_joint" % lr][i_start])
            prev_gripper_open = binarize_gripper(seg_info["%s_gripper_joint" % lr][i_start - 1]) if i_start != 0 else False
            if not set_gripper_sim(lr, gripper_open, prev_gripper_open, animate):
                redprint("Grab %s failed" % lr)
                success = False
        if not success:
            break
        # Execute the robot trajectory
        if len(bodypart2traj) > 0:
            success &= exec_traj_sim(bodypart2traj, animate)
        #if args.simulation:
            #Globals.exec_log(curr_step, "execute_traj.miniseg_%d.sim_rope_nodes_after_traj" % i_miniseg, Globals.sim.rope.GetNodes())

        if not success:
            break

    #if args.simulation:
    Globals.sim.settle(animate=animate)
    if Globals.exec_log:
        Globals.exec_log(curr_step, "execute_traj.sim_rope_nodes_after_full_traj", Globals.sim.rope.GetNodes())

    #if args.sim_desired_knot_name is not None:
    from rapprentice import knot_identification
    knot_name = knot_identification.identify_knot(Globals.sim.rope.GetControlPoints())
    if knot_name is not None:
        if knot_name == knot or knot == "any":
            redprint("Identified knot: %s. Success!" % knot_name)
            #Globals.exec_log(curr_step, "result", True, description="identified knot %s" % knot_name)
            return True
        else:
            redprint("Identified knot: %s, but expected %s. Continuing." % (knot_name, knot))
    else:
        redprint("Not a knot. Continuing.")

    redprint("Segment %s result: %s" % (segment, success))
    return False


def prase_arguments():
    import argparse
    usage = """

Run in simulation with a translation and a rotation of fake data:
./do_task.py ~/Data/sampledata/overhand/overhand.h5 --fake_data_segment=overhand0_seg00 --execution=0  --animation=1 --select_manual --fake_data_transform .1 .1 .1 .1 .1 .1

Run in simulation choosing the closest demo, single threaded
./do_task.py ~/Data/all.h5 --fake_data_segment=demo1-seg00 --execution=0  --animation=1  --parallel=0

Actually run on the robot without pausing or animating
./do_task.py ~/Data/overhand2/all.h5 --execution=1 --animation=0

"""
    parser = argparse.ArgumentParser(usage=usage)
    parser.add_argument("h5file", type=str)
    parser.add_argument("--cloud_proc_func", default="extract_red")
    parser.add_argument("--cloud_proc_mod", default="rapprentice.cloud_proc_funcs")
    parser.add_argument("--execution", type=int, default=0)
    parser.add_argument("--animation", type=int, default=0)
    parser.add_argument("--simulation", type=int, default=0)
    parser.add_argument("--parallel", type=int, default=1)
    parser.add_argument("--prompt", action="store_true")
    parser.add_argument("--select_manual", action="store_true")
    parser.add_argument("--fake_data_segment", type=str)
    parser.add_argument("--fake_data_transform", type=float, nargs=6, metavar=("tx", "ty", "tz", "rx", "ry", "rz"), default=[0, 0, 0, 0, 0, 0], help="translation=(tx, ty, tz), axis-angle rotation=(rx, ry, rz)")
    parser.add_argument("--sim_init_perturb_radius", type=float, default=None)
    parser.add_argument("--sim_init_perturb_num_points", type=int, default=7)
    parser.add_argument("--sim_desired_knot_name", type=str, default=None)
    parser.add_argument("--max_steps_before_failure", type=int, default=-1)
    parser.add_argument("--random_seed", type=int, default=None)
    parser.add_argument("--no_failure_examples", type=int, default=0)
    parser.add_argument("--only_first_n_examples", type=int, default=-1)
    parser.add_argument("--only_examples_from_list", type=str)
    parser.add_argument("--interactive", action="store_true")
    parser.add_argument("--log", type=str, default="", help="")
    args = parser.parse_args()
    print "args =", args
    if args.fake_data_segment is None:
        assert args.execution == 1
    if args.simulation:
        assert args.execution == 0 and args.fake_data_segment is not None
    return args

def main():
    args = prase_arguments()
    if args.random_seed is not None:
        Globals.random_seed = args.random_seed
        
    choose_segment = find_closest_manual if args.select_manual else find_closest_auto
    rope_state = RopeState(args.fake_data_segment, args.sim_init_perturb_radius, args.sim_init_perturb_num_points)
    params = TaskParameters(args.h5file, args.sim_desired_knot_name, animate=args.animation, 
                            max_steps_before_failure=args.max_steps_before_failure, choose_segment=choose_segment, log_name=None)
    result = do_single_random_task(rope_state, params)
    print "Main results are", result

if __name__ == "__main__":
    main()
