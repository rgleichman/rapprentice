import h5py
import cPickle as cp
import numpy as np
import argparse
import os, os.path as osp
import matplotlib.pylab as plt
from rapprentice import registration


def find_long_path(h5py_file, node_key, path_so_far, thresh):
    new_path = path_so_far + [node_key]
    if len(new_path) >= thresh:
        return new_path

    children = h5py_file[node_key]['children'][()]
    if np.isscalar(children):
        return None
    else:
        for child_key in children:
            cpath = find_long_path(h5py_file, child_key, new_path, thresh)
            if cpath:
                return cpath
        return None


def get_root_keys(h5py_file):
    """
    returns a list of the nodes corresponding to human-demos.
    """                
    hd_keys = []
    for k in h5py_file.keys():
        try:
            _ = int(k)
        except:
            hd_keys.append(k)
    return hd_keys


def plot_path(h5py_file, path, plot=False):
    for i,node in enumerate(path):
        node_cloud_xy = h5py_file[node]['cloud_xyz'][:,:2]
        if plot:
            if i==0:
                plt.scatter(node_cloud_xy[:,0], node_cloud_xy[:,1])
            else:
                plt.plot(node_cloud_xy[:,0], node_cloud_xy[:,1])
            plt.show(block=False)


def get_long_path(h5py_fname, plen):
    h5py_file = h5py.File(h5py_fname)
    hd_keys   = get_root_keys(h5py_file)
    for root_key in hd_keys:
        path = find_long_path(h5py_file, root_key, [], plen)
        if path:
            plot_path(h5py_file, path)
            return path
    h5py_file.close()
    return None


def get_fwarp(xyz0, xyz1):
    scaled_xyz0, xyz0_params = registration.unit_boxify(xyz0)
    scaled_xyz1, xyz1_params = registration.unit_boxify(xyz1)
    fwarp, _, corr_nm  = registration.tps_rpm_bij(scaled_xyz0, scaled_xyz1, plotting=False,
                                            rot_reg=np.r_[1e-4, 1e-4, 1e-1], n_iter=50,
                                            reg_init=10, reg_final=.01, old_xyz=xyz0, new_xyz=xyz1,
                                            return_corr=True)

    return registration.unscale_tps(fwarp, xyz0_params, xyz1_params), corr_nm


def get_f_path(h5py_fname, path):
    h5py_file = h5py.File(h5py_fname)
    fs = []
    corrs = []
    for i in xrange(1, len(path)):
        src  = h5py_file[path[i-1]]['cloud_xyz'][()]
        targ = h5py_file[path[i]]['cloud_xyz'][()]
        f, corr = get_fwarp(src, targ)
        fs.append(f)
        corrs.append(corr)
    h5py_file.close()
    return fs, corrs


def draw_grid(f, mins, maxes, xres = .1, yres = .1, zres = -1):
    xmin, ymin, zmin = mins-.1
    xmax, ymax, zmax = maxes+.2

    nfine = 30
    xcoarse = np.arange(xmin, xmax, xres)
    ycoarse = np.arange(ymin, ymax, yres)
    if zres == -1: zcoarse = [(zmin+zmax)/2.]
    else: zcoarse = np.arange(zmin, zmax, zres)
    xfine = np.linspace(xmin, xmax, nfine)
    yfine = np.linspace(ymin, ymax, nfine)
    zfine = np.linspace(zmin, zmax, nfine)
    
    lines = []
    if len(zcoarse) > 1:    
        for x in xcoarse:
            for y in ycoarse:
                xyz = np.zeros((nfine, 3))
                xyz[:,0] = x
                xyz[:,1] = y
                xyz[:,2] = zfine
                lines.append(f(xyz))

    for y in ycoarse:
        for z in zcoarse:
            xyz = np.zeros((nfine, 3))
            xyz[:,0] = xfine
            xyz[:,1] = y
            xyz[:,2] = z
            lines.append(f(xyz))
        
    for z in zcoarse:
        for x in xcoarse:
            xyz = np.zeros((nfine, 3))
            xyz[:,0] = x
            xyz[:,1] = yfine
            xyz[:,2] = z
            lines.append(f(xyz))

    for i,line in enumerate(lines):
        if i==2 or i==4:
            plt.plot(line[:,0], line[:,1], '0.1', lw=2.5)
        else:
            plt.plot(line[:,0], line[:,1], '0.7')
            

def plot_correspondences(xyz, corr_xyz):
    for i in xrange(len(xyz)):
        plt.plot([xyz[i,0]+0.05, corr_xyz[i,0]], [xyz[i,1]-0.05, corr_xyz[i,1]], '0.5')


class chain_fs:
    def __init__(self, fs):
        self.fs = fs

    def transform_points(self, pts):
        wpts = pts.copy()
        for i in range(len(self.fs)):
            wpts = self.fs[i].transform_points(wpts)
        return wpts


def get_f_path_bootstrap(h5py_fname, path):
    h5py_file = h5py.File(h5py_fname)

    root_xyz                 = h5py_file[path[0]]['cloud_xyz'][()]
    scaled_root, root_params = registration.unit_boxify(root_xyz)

    corr_prev = np.eye(len(root_xyz))
    path = path

    for i in xrange(1, len(path)):
        ii   = h5py_file[path[i-1]]['cloud_xyz'][()]
        targ = h5py_file[path[i]]['cloud_xyz'][()]
        
        scaled_ii, ii_params     = registration.unit_boxify(ii)
        scaled_targ, targ_params = registration.unit_boxify(targ)

        fboot, _, corr_prev = registration.tps_rpm_bootstrap(scaled_root, scaled_ii, scaled_targ, corr_prev, n_iter=50, reg_init=10, reg_final=0.01)
        fboot =  registration.unscale_tps(fboot, root_params, targ_params)
    return fboot, None    

def plot_warp(src, targ, warp, f):
    plt.axis('off')
    plt.hold(True)
    plt.axis((0.2,1.1,-0.4,0.1))
    draw_grid(f.transform_points, np.min(src, axis=0), np.max(src, axis=0))
    plot_correspondences(src[:,:], warp[:,:])
    plt.scatter(src[:,0]+0.05, src[:,1]-0.05, c='g', lw=0, marker='d')
    plt.scatter(targ[:,0], targ[:,1], c='b', lw=0)
    plt.scatter(warp[:,0], warp[:,1], c='r', lw=0, marker='s')
    
def plot_chained_f(path, h5py_fname):
    h5py_file = h5py.File(h5py_fname)
    init_xyz  = h5py_file[path[0]]['cloud_xyz'][()]
    final_xyz = h5py_file[path[-1]]['cloud_xyz'][()]
    penul_xyz = h5py_file[path[-2]]['cloud_xyz'][()]
    h5py_file.close()

    #plt.subplot(2,2,2)
    plt.clf()
    fdirect          = get_fwarp(init_xyz, final_xyz)[0]
    direct_warp_xyz  = fdirect.transform_points(init_xyz)
    plot_warp(init_xyz, final_xyz, direct_warp_xyz, fdirect)
    plt.savefig('2.pdf')

    #plt.subplot(2,2,4)
    plt.clf()
    fs, corrs = get_f_path(h5py_fname, path)
    fchain          = chain_fs(fs)
    chain_warp_xyz  = fchain.transform_points(init_xyz)
    plot_warp(init_xyz, final_xyz, chain_warp_xyz, fchain)
    plt.savefig('4.pdf')


    """
    plt.subplot(1,3,3)    
    ftps = registration.fit_ThinPlateSpline(init_xyz, chain_warp_xyz, bend_coef = 0.1, rot_coef = (1e-4,1e-4,1e-1))
    tps_warp_xyz  = ftps.transform_points(init_xyz)
    plt.hold(True)
    plt.scatter(init_xyz[:,0], init_xyz[:,1], c='r', lw=0)
    plt.scatter(final_xyz[:,0], final_xyz[:,1], c='b', lw=0)
    plt.scatter(tps_warp_xyz[:,0], tps_warp_xyz[:,1], c='g', lw=0)
    draw_grid(ftps.transform_points, np.min(init_xyz, axis=0), np.max(init_xyz, axis=0))
    plot_correspondences(init_xyz, tps_warp_xyz)
    """
    
    """
    plt.subplot(1,3,3)
    scaled_i, xyz0_params = registration.unit_boxify(init_xyz)
    scaled_ii, p_params = registration.unit_boxify(penul_xyz)
    scaled_f, xyz1_params = registration.unit_boxify(final_xyz)
    fboot, _,_ = registration.tps_rpm_bootstrap(scaled_i, scaled_ii, scaled_f, reduce(np.dot, corrs[:-1]), n_iter=50, reg_init=10, reg_final=0.01)
    fboot =  registration.unscale_tps(fboot, xyz0_params, xyz1_params)
 
    boot_warp_xyz  = fboot.transform_points(init_xyz)
    plt.hold(True)
    plt.scatter(init_xyz[:,0], init_xyz[:,1], c='r', lw=0)
    plt.scatter(final_xyz[:,0], final_xyz[:,1], c='b', lw=0)
    plt.scatter(boot_warp_xyz[:,0], boot_warp_xyz[:,1], c='g', lw=0)
    draw_grid(fboot.transform_points, np.min(init_xyz, axis=0), np.max(init_xyz, axis=0))
    plot_correspondences(init_xyz, boot_warp_xyz)
    """
    

    #plt.subplot(2,2,3)
    plt.clf()    
    fboot = get_f_path_bootstrap(h5py_fname, path)[0]
    boot_warp_xyz  = fboot.transform_points(init_xyz)
    plot_warp(init_xyz, final_xyz, boot_warp_xyz, fboot)
    plt.savefig('3.pdf')
    
    #plt.subplot(2,2,1)
    plt.clf()
    plt.axis('off')
    plt.hold(True)
    plt.scatter(init_xyz[:,0], init_xyz[:,1], c='r', lw=0)
    draw_grid(lambda x: x, np.min(init_xyz, axis=0), np.max(init_xyz, axis=0))
    plt.savefig('1.pdf')
    
    plt.show()
    

no_root_res = {30 : np.array([0.63,0.72,0.77,0.62,0.696,0.6,0.753,0.75,0.703,0.689]),
               60 : np.array([0.64,0.74,0.74,0.63,0.75,0.636,0.77,0.76,0.753,0.683]),
               90 : np.array([0.67,0.753,0.786,0.67,0.796,0.686,0.803,0.706,0.75,0.72]),
               120: np.array([0.68,0.74,0.783,0.683,0.77,0.68,0.843,0.74,0.756,0.686])}

root_res = {30 : np.array([]),
            60 : np.array([]),
            90 : np.array([]),
            120: np.array([])}

def gen_plot(root):
    dat    = no_root_res if not root else root_res
    ndemos = np.array(np.sort(dat.keys()))
    dmat   = np.array([dat[k] for k in np.sort(dat.keys())])

    savg   = np.mean(dmat, axis=1)
    stds   = np.std(dmat, axis=1)
    plt.axis((20,130,0,1))

    ybest = dmat[:,6]
    baseline = np.array([0.6,0.6,0.6,0.6])
    plt.hold(True)
    _, caps,dd = plt.errorbar(ndemos, savg, yerr=stds, ecolor='red', lw=2, fmt=':b', label="average")
    plt.plot(ndemos, ybest, '--b', lw=2, label="best")
    plt.plot(ndemos, baseline, '-b', lw=2, label="baseline")
    plt.xlabel('number of exploration runs')
    plt.ylabel('success rate')
    plt.legend()
    plt.plot()
    
    
    plt.show()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tree_h5_fname", type=str,
                        help="name of the h5 file containing the tree [bootstrapping/data]/<anything that goes here>")
    parser.add_argument("--path_len", type=int, help="number of nodes a path should have", default=4)
    args = parser.parse_args()

    data_dir = osp.join(os.getenv("BOOTSTRAPPING_DIR"), "data")
    h5py_fname = osp.join(data_dir, args.tree_h5_fname)
    path = get_long_path(h5py_fname, args.path_len)
    print path
    plot_chained_f(path, h5py_fname)
    
    

if __name__=='__main__':
    gen_plot(False)
    #main()
