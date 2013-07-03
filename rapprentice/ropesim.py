import bulletsimpy
import numpy as np
from rapprentice import rope_initialization, retiming
from trajoptpy import math_utils as mu

def transform(hmat, p):
    return hmat[:3,:3].dot(p) + hmat[:3,3]

def in_grasp_region(robot, lr, pt):
    tol = .00

    manip_name = {"l": "leftarm", "r": "rightarm"}[lr]
    manip = robot.GetManipulator(manip_name)
    l_finger = robot.GetLink("%s_gripper_l_finger_tip_link"%lr)
    r_finger = robot.GetLink("%s_gripper_r_finger_tip_link"%lr)

    def on_inner_side(pt, finger_lr):
        finger = l_finger
        closing_dir = np.cross(manip.GetLocalToolDirection(), [-1, 0, 0])
        local_inner_pt = np.array([0.234402, -0.299, 0])/20.
        if finger_lr == "r":
            finger = r_finger
            closing_dir *= -1
            local_inner_pt[1] *= -1
        inner_pt = transform(finger.GetTransform(), local_inner_pt)
        return manip.GetTransform()[:3,:3].dot(closing_dir).dot(pt - inner_pt) > 0

    # check that pt is behind the gripper tip
    pt_local = transform(np.linalg.inv(manip.GetTransform()), pt)
    if pt_local[2] > .02 + tol:
        return False

    # check that pt is within the finger width
    if abs(pt_local[0]) > .01 + tol:
        return False

    # check that pt is between the fingers
    if not on_inner_side(pt, "l") or not on_inner_side(pt, "r"):
        return False

    return True


# def retime_traj(robot, inds, traj):
#     vel_ratio = .02
#     vel_limits = robot.GetDOFVelocityLimits()[inds] * vel_ratio
#     times = retiming.retime_with_vel_limits(traj, vel_limits)
#     times_up = np.arange(0,times[-1],.1)
#     traj_up = mu.interp2d(times_up, times, traj)
#     return traj_up

def retime_traj(robot, inds, traj):
    """retime a trajectory so that it executes slowly enough for the simulation"""
    max_cart_vel = .02

    cart_traj = np.empty((len(traj), 6))
    leftarm, rightarm = robot.GetManipulator("leftarm"), robot.GetManipulator("rightarm")
    with robot:
        for i in range(len(traj)):
            robot.SetDOFValues(traj[i], inds)
            cart_traj[i,:3] = leftarm.GetTransform()[:3,3]
            cart_traj[i,3:] = rightarm.GetTransform()[:3,3]

    times = retiming.retime_with_vel_limits(cart_traj, np.repeat(max_cart_vel, 6))
    times_up = np.arange(0, times[-1], .1)
    traj_up = mu.interp2d(times_up, times, traj)
    return traj_up


class Simulation(object):
    def __init__(self, env, robot):
        self.env = env
        self.robot = robot
        self.rope_pts = None
        self.bt_env = None
        self.bt_robot = None
        self.rope = None
        self.constraints = {"l": [], "r": []}

        self.rope_params = bulletsimpy.CapsuleRopeParams()
        self.rope_params.radius = 0.005
        self.rope_params.angStiffness = .1
        self.rope_params.angDamping = 1
        self.rope_params.linDamping = .75
        self.rope_params.angLimit = .4
        self.rope_params.linStopErp = .2

    def set_rope_cloud(self, xyz):
        self.rope_pts = rope_initialization.find_path_through_point_cloud(xyz)

    def create(self):
        assert self.rope_pts is not None
        self.bt_env = bulletsimpy.BulletEnvironment(self.env, [])
        self.bt_env.SetGravity([0, 0, -9.8])
        self.bt_robot = self.bt_env.GetObjectByName(self.robot.GetName())
        self.rope = bulletsimpy.CapsuleRope(self.bt_env, 'rope', self.rope_pts, self.rope_params)

        for i in range(1000):
            self.bt_env.Step(.01, 200, .005)

    def step(self):
        self.bt_robot.UpdateBullet()
        self.bt_env.Step(.01, 200, .005)
        self.rope.UpdateRave()
        self.env.UpdatePublishedBodies()

    def observe_cloud(self, upsample=0):
        pts = self.rope.GetControlPoints()
        if upsample == 0:
            return pts
        lengths = np.r_[0, self.rope.GetHalfHeights() * 2]
        summed_lengths = np.cumsum(lengths)
        assert len(lengths) == len(pts)
        return mu.interp2d(np.linspace(0, summed_lengths[-1], upsample), summed_lengths, pts)

    def grab_rope(self, lr):
        nodes = self.rope.GetNodes()
        graspable_inds = [i for (i, n) in enumerate(nodes) if in_grasp_region(self.robot, lr, n)]
        print 'graspable inds for %s: %s' % (lr, str(graspable_inds))
        if len(graspable_inds) == 0:
            return False

        robot_link = self.robot.GetLink("%s_gripper_l_finger_tip_link"%lr)
        rope_links = self.rope.GetKinBody().GetLinks()
        for i in graspable_inds:
            print 'pivot_in_b', np.linalg.inv(rope_links[i].GetTransform()).dot(np.r_[robot_link.GetTransform()[:3,3], 1])[:3]
            self.constraints[lr].append(self.bt_env.AddConstraint({
                "type": "point2point",
                "params": {
                    "link_a": robot_link,
                    "link_b": rope_links[i],
                    "pivot_in_a": transform(np.linalg.inv(robot_link.GetTransform()), rope_links[i].GetTransform()[:3,3]),
                    "pivot_in_b": [0, 0, 0],
                    "disable_collision_between_linked_bodies": True,
                }
            }))

        for i in range(10):
            self.step()

        # import trajoptpy
        # for i in range(20):
        #   handles = []
        #   handles.append(self.env.plot3(nodes[graspable_inds], 10, [1,0,0]))
        #   trajoptpy.GetViewer(self.env).Idle()
        #   self.step()

        return True

    def release_rope(self, lr):
        print 'RELEASE: %s (%d constraints)' % (lr, len(self.constraints[lr]))
        for c in self.constraints[lr]:
            self.bt_env.RemoveConstraint(c)
        self.constraints[lr] = []
