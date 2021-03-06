#!/usr/bin/env python

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--delete", action="store_true")
parser.add_argument("sampledata_dir")
args = parser.parse_args()

import subprocess,os
import os.path as osp
import rapprentice
os.chdir(args.sampledata_dir)

print "creating zip file"
subprocess.check_call('tar cvf all.tar . --exclude all.tar',shell=True)
print "uploading"            
subprocess.check_call("rsync -azvu %s ./ pabbeel@rll.berkeley.edu:/var/www/rapprentice/sampledata"%("--delete" if args.delete else ""), shell=True)
            