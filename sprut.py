#!/usr/bin/env python3

import pybullet as p
import time
import pybullet_data
import numpy as np

# Start pybullet simulation
p.connect(p.GUI)    
# p.connect(p.DIRECT) # don't render

p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)

# load urdf file path
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 

# load urdf and set gravity
#p.setGravity(0,0,-10)

def loadBody(f, startPos=[0, 0, 0]):
    startOrientation = p.getQuaternionFromEuler([0,0,0])
    bodyId = p.loadURDF("./" + f, startPos, startOrientation, useFixedBase=1)
    return bodyId

loadBody("target.urdf", [0, 0, 3])

bodyId = loadBody("manipulator.urdf")

for i in range(p.getNumJoints(bodyId)):
    if (i+1) % 2 == 1:
        pos = 0.1
    else:
        pos = 0
    p.setJointMotorControl2(bodyId, i, p.POSITION_CONTROL, pos, force=5000)
    #link_name = p.getJointInfo(bodyId, i)[12].decode('UTF-8')
    #print("joint %d, link %s" % (i, link_name))

#p.setJointMotorControl2(bodyId, 0, p.POSITION_CONTROL, 1)
#p.setJointMotorControl2(bodyId, 2, p.POSITION_CONTROL, 1)

# attach camera to the last joint of the body
cameraLinkId = p.getNumJoints(bodyId) - 1

projection_matrix = p.computeProjectionMatrixFOV(60, 1.0, 0.01, 100)
def camera():
    # Center of mass position and orientation (of link-7)
    com_p, com_o, _, _, _, _ = p.getLinkState(bodyId, cameraLinkId)
    rot_matrix = p.getMatrixFromQuaternion(com_o)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)
    # Initial vectors
    init_camera_vector = (0, 0, 1) # z-axis
    init_up_vector = (0, 1, 0) # y-axis
    # Rotated vectors
    camera_vector = rot_matrix.dot(init_camera_vector)
    up_vector = rot_matrix.dot(init_up_vector)
    view_matrix = p.computeViewMatrix(com_p, com_p + 0.1 * camera_vector, up_vector)
    img = p.getCameraImage(320, 200, view_matrix, projection_matrix)
    return img

# step through the simluation
for i in range (10000):
    p.stepSimulation()
    camera()
    #time.sleep(1./240.)

p.disconnect()
