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
    fwarp, _, _  = registration.tps_rpm_bij(scaled_xyz0, scaled_xyz1, plotting=False,
                                            rot_reg=np.r_[1e-4, 1e-4, 1e-1], n_iter=50,
                                            reg_init=10, reg_final=.01, old_xyz=xyz0, new_xyz=xyz1,
                                            return_corr=True)

    return registration.unscale_tps(fwarp, xyz0_params, xyz1_params)


def get_f_path(h5py_fname, path):
    h5py_file = h5py.File(h5py_fname)
    fs = []
    for i in xrange(1, len(path)):
        src  = h5py_file[path[i-1]]['cloud_xyz'][()]
        targ = h5py_file[path[i]]['cloud_xyz'][()]
        fs.append(get_fwarp(src, targ))
    h5py_file.close()
    return fs


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

    for line in lines:
        plt.plot(line[:,0], line[:,1], '0.75')

def plot_correspondences(xyz, corr_xyz):
    for i in xrange(len(xyz)):
        plt.plot([xyz[i,0], corr_xyz[i,0]], [xyz[i,1], corr_xyz[i,1]], '0.2')


class chain_fs:
    def __init__(self, fs):
        self.fs = fs

    def transform_points(self, pts):
        wpts = pts.copy()
        for i in range(len(self.fs)):
            wpts = self.fs[i].transform_points(wpts)
        return wpts


def plot_chained_f(path, h5py_fname):
    h5py_file = h5py.File(h5py_fname)
    init_xyz  = h5py_file[path[0]]['cloud_xyz'][()]
    final_xyz = h5py_file[path[-1]]['cloud_xyz'][()]
    h5py_file.close()

    plt.subplot(1,2,1)
    fdirect          = get_fwarp(init_xyz, final_xyz)
    direct_warp_xyz  = fdirect.transform_points(init_xyz)
    plt.hold(True)
    plt.scatter(init_xyz[:,0], init_xyz[:,1], c='r', lw=0)
    plt.scatter(final_xyz[:,0], final_xyz[:,1], c='b', lw=0)
    plt.scatter(direct_warp_xyz[:,0], direct_warp_xyz[:,1], c='g', lw=0)
    draw_grid(fdirect.transform_points, np.min(final_xyz, axis=0), np.max(final_xyz, axis=0))
    plot_correspondences(init_xyz, direct_warp_xyz)
    #plt.show(block=False)
    
    plt.subplot(1,2,2)
    fchain  = chain_fs(get_f_path(h5py_fname, path))
    chain_warp_xyz  = fchain.transform_points(init_xyz)
    plt.hold(True)
    plt.scatter(init_xyz[:,0], init_xyz[:,1], c='r', lw=0)
    plt.scatter(final_xyz[:,0], final_xyz[:,1], c='b', lw=0)
    plt.scatter(chain_warp_xyz[:,0], chain_warp_xyz[:,1], c='g', lw=0)
    draw_grid(fchain.transform_points, np.min(final_xyz, axis=0), np.max(final_xyz, axis=0))
    plot_correspondences(init_xyz, chain_warp_xyz)
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
    main()
