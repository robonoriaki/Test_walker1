import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally 
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,3]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("urdf/test_walker1.urdf",cubeStartPos, cubeStartOrientation)

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

    p.setJointMotorControl2(boxId, jindex, controlMode = mode, targetPosition = angle)
    
    #Get joints num
    #jnum = p.getNumJoints(boxId)
    #print('joints num = {}'.format(jnum))

    #Get joint information
    jointinfo = p.getJointInfo(boxId, jindex)
    print(jointinfo[1])

    #display
    p.stepSimulation()
    time.sleep(1./2400.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()

