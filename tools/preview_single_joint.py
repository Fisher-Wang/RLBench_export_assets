import pybullet as p
import pybullet_data
import time
import math
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-f', '--file_path')
args = parser.parse_args()

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the URDF file
path = args.file_path
robot_id = p.loadURDF(path)

# Set the gravity
p.setGravity(0, 0, -9.81)

# Create a fixed constraint between the base link and the world frame
constraint_id = p.createConstraint(
    parentBodyUniqueId=robot_id,
    parentLinkIndex=-1,  # -1 refers to the base link of the loaded model
    childBodyUniqueId=-1,  # -1 refers to the world frame
    childLinkIndex=-1,
    jointType=p.JOINT_FIXED,
    jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, 0],
    childFramePosition=[0, 0, 0],
)

# Print joints info
print("There are", p.getNumJoints(robot_id), "joints in the robot.")
for i in range(p.getNumJoints(robot_id)):
    joint_info = p.getJointInfo(robot_id, i)
    print(f'Joint Index: {joint_info[0]}')
    print(f'Joint Name: {joint_info[1].decode("utf-8")}')
    print(f'Joint Type: {joint_info[2]}')
    print(f'Joint Lower Limit: {joint_info[8]}')
    print(f'Joint Upper Limit: {joint_info[9]}')

# Get joint
joint_index = 0
joint_info = p.getJointInfo(robot_id, joint_index)
joint_lower_limit = joint_info[8]
joint_upper_limit = joint_info[9]

# Set the joint parameters
frequency = 0.2  # Frequency of the sinusoidal motion

# Simulation loop
start_time = time.time()
while True:
    current_time = time.time() - start_time
    target_position = math.sin(2 * math.pi * frequency * current_time)  # range: [-1, 1]
    target_position = (target_position + 1) / 2                         # range: [0, 1]
    target_position = target_position * (joint_upper_limit - joint_lower_limit) + joint_lower_limit  # range: [joint_lower_limit, joint_upper_limit]
    
    p.setJointMotorControl2(
        bodyUniqueId=robot_id,
        jointIndex=0,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target_position,
        force=10  # Adjust the force as needed
    )
    
    p.stepSimulation()
    time.sleep(1./20)  # Simulate at 240 Hz

# Disconnect from PyBullet
p.disconnect()