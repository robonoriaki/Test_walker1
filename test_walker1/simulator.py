import pybullet as p
import time
import pybullet_data
import numpy


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally 
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
robotId = p.loadURDF("urdf/test_walker1.urdf",cubeStartPos, cubeStartOrientation)

mode = p.POSITION_CONTROL
angle = 0
add_angle = 3.14 / 50
jindex = 0
count = -30

for i in range (10000):
    count = count + 1;
    if count < 0:
        angle = angle + add_angle
    elif count >= 0 and count < 30:
        angle = angle - add_angle
    else:
        count = -30

    #p.setJointMotorControl2(robotId, jindex, controlMode = mode, targetPosition = angle)
    
    p.setJointMotorControl2(robotId, jindex, controlMode = mode, targetPosition = 3.14 / 7)
    p.setJointMotorControl2(robotId, 2, controlMode = mode, targetPosition = -3.14 / 6)
    p.setJointMotorControl2(robotId, 4, controlMode = mode, targetPosition = -3.14 / 6)
    p.setJointMotorControl2(robotId, 6, controlMode = mode, targetPosition = 3.14 / 7)

    #Get joints num
    #jnum = p.getNumJoints(robotId)
    #print('joints num = {}'.format(jnum))

    #Get joint information
    jointinfo = p.getJointInfo(robotId, jindex)
    #print(jointinfo[1])

    #display
    p.stepSimulation()
    time.sleep(1./2400.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
print(cubePos,cubeOrn)
p.disconnect()

