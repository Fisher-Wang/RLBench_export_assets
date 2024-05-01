import os, yaml
import xml.etree.ElementTree as ET
from collections import defaultdict

def mkdir(path):
    os.makedirs(path, exist_ok=True)
    return path

def read_yaml(path):
    with open(path) as f:
        data = yaml.load(f, Loader=yaml.Loader)
    return data

def write_yaml(path, data):
    with open(path, 'w+') as f:
        yaml.dump(data, f, default_flow_style=False)
