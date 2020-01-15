import pybullet as p
import time
import pybullet_data
import math

PI = math.pi


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally 
#set Gravity
p.setGravity(0,0,-9.8)
#load plane URDF
planeId = p.loadURDF("plane.urdf")
#initial position of a robot
cubeStartPos = [0,0,1]
#initial orientation of a robot
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
#load a robot
robotId = p.loadURDF("urdf/test_walker1.urdf", cubeStartPos, cubeStartOrientation)

mode = p.POSITION_CONTROL
angle = 0
add_angle = 3.14 / 50
jindex = 0
count = -30

simpara = p.getPhysicsEngineParameters(physicsClient)
print('simulation parameters')
print(simpara)

for i in range (10000):
    #get Base position
    basePos, baseQua = p.getBasePositionAndOrientation(robotId)
    
    #VRPos = (basePos[0] + 0.2, basePos[1] - 0.5, basePos[0] + 0.2)
    #VRQua = (baseQua[0], baseQua[1], baseQua[2])
    #VRtarget = (basePos[0], basePos[1], basePos[2])
    #print('VRPos type')
    #print(type(VRPos))
    #set Camera state
    #p.setVRCameraState(VRPos, VRQua, VRtarget)
    p.resetDebugVisualizerCamera(2.0, 10, -30, basePos)

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
    
    #get contact point
    conflag = p.getContactPoints(robotId, planeId, 1)
    #print('contact flag')
    #print(conflag)

   
    print('conflag')
    conflen = len(conflag)
    if conflen != 0:
        if conflag[0][0] == 0:
            print('contact!')
    else:
        print('no contct..')

    #display
    p.stepSimulation()
    time.sleep(1./240.)

    #basePos, baseQua = p.getBasePositionAndOrientation(robotId)
    #print(basePos,baseQua)
p.disconnect()

