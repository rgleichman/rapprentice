#! /bin/bash
## script to run several tests of bootstrapping evals of demo1.

python scripts/iros_experiments.py data/demo1_warp_parent_copy/boot_30.h5 data/demo1_test/warp_parent_30.cp --task_file data/demo1_test/tasks.h5
python scripts/iros_experiments.py data/demo1_warp_parent_copy/boot_30_20.h5 data/demo1_test/warp_parent_20.cp --task_file data/demo1_test/tasks.h5
python scripts/iros_experiments.py data/demo1_warp_parent_copy/boot_30_10.h5 data/demo1_test/warp_parent_10.cp --task_file data/demo1_test/tasks.h5
python scripts/iros_experiments.py data/demo1_warp_root_copy/boot_30.h5 data/demo1_test/warp_root_30.cp --task_file data/demo1_test/tasks.h5 --warp_root
python scripts/iros_experiments.py data/demo1_warp_root_copy/boot_30_20.h5 data/demo1_test/warp_root_20.cp --task_file data/demo1_test/tasks.h5 --warp_root
python scripts/iros_experiments.py data/demo1_warp_root_copy/boot_30_10.h5 data/demo1_test/warp_root_10.cp --task_file data/demo1_test/tasks.h5 --warp_root
python scripts/iros_experiments.py data/demo1_warp_root_copy/boot_30_orig.h5 data/demo1_test/warp_orig.cp --task_file data/demo1_test/tasks.h5
