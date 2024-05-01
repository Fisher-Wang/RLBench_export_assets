import os
from glob import glob
from os.path import join as pjoin
from os.path import exists as pexists
from utils import read_yaml
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-c', '--conf', required=True)
parser.add_argument('-f', '--base_dir_path')
args = parser.parse_args()

base_dir = args.base_dir_path

tasks = read_yaml(args.conf)['Objects']
failure_tasks = []

for t in tasks:
    urdf_dir = pjoin(base_dir, 'urdf', t)
    usd_dir = pjoin(base_dir, 'usd', t)
    urdf_filenames = [filename for filename in os.listdir(urdf_dir) if filename.endswith('.urdf') and not filename.endswith('_unique.urdf')]
    if len(urdf_filenames) == 0:
        failure_tasks.append(t)
        continue
    
    fail = False
    for urdf_filename in urdf_filenames:
        usd_filename = urdf_filename.replace('.urdf', '.usd')
        urdf_path = pjoin(urdf_dir, urdf_filename)
        usd_path = pjoin(usd_dir, usd_filename)
        if not pexists(usd_path):
            print(f'Fail to convert {t}/{urdf_filename} to USD.')
            fail = True
            continue
        else:
            print(f'Successfully convert {t}/{urdf_filename} to USD.')
    if fail:
        failure_tasks.append(t)
    

print(f'Total {len(failure_tasks)} out of {len(tasks)} tasks failed.')
print(f'Failed tasks: {failure_tasks}')