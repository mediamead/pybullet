#!/usr/bin/env python3

import pybullet as p
import time
import pybullet_data

# Start pybullet simulation
p.connect(p.GUI)    
# p.connect(p.DIRECT) # don't render

#p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, True)

# load urdf file path
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 

# load urdf and set gravity
#p.setGravity(0,0,-10)
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
bodyId = p.loadURDF("./manipulator.urdf", cubeStartPos, cubeStartOrientation, useFixedBase=1)

for i in range(p.getNumJoints(bodyId)):
    p.setJointMotorControl2(bodyId, i, p.POSITION_CONTROL, (i+1)%2/10)

#p.setJointMotorControl2(bodyId, 0, p.POSITION_CONTROL, 1)
#p.setJointMotorControl2(bodyId, 2, p.POSITION_CONTROL, 1)

def camera():
    p.getCameraImage(320, 200)

# step through the simluation
for i in range (10000):
    p.stepSimulation()
    camera()
    time.sleep(1./240.)

p.disconnect()
