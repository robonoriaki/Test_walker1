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
cubeStartPos = [0,0,1.0]
#initial orientation of a robot
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
#load a robot
robotId = p.loadURDF("urdf/test_walker1.urdf", cubeStartPos, cubeStartOrientation)

mode = p.POSITION_CONTROL
angle = 0
add_angle = 3.14 / 50
jindex = 0
count = -30

#simpara = p.getPhysicsEngineParameters(physicsClient)
#print('simulation parameters')
#print(simpara)

#joint indices
jointIndex = {"outer_hip_joint1":0, "outer_knee_joint1":1, "inner_hip_joint1":2, "inner_knee_joint1":3, "inner_hip_joint2":4, "inner_knee_joint2":5, "outer_hip_joint2":6, "outer_knee_joint2":7}

#joint indices list
outer_hip_joints = [jointIndex["outer_hip_joint1"], jointIndex["outer_hip_joint2"]]
outer_knee_joints = [jointIndex["outer_knee_joint1"], jointIndex["outer_knee_joint2"]]
inner_hip_joints = [jointIndex["inner_hip_joint1"], jointIndex["inner_hip_joint2"]]
inner_knee_joints = [jointIndex["inner_knee_joint1"], jointIndex["inner_knee_joint2"]]

while True:
    #get Base position
    basePos, baseQua = p.getBasePositionAndOrientation(robotId)
    #set camera position 
    p.resetDebugVisualizerCamera(2.0, 20, -30, basePos)
    
    #hip joints move using setJointMotorControl2
    p.setJointMotorControl2(robotId, jointIndex["outer_hip_joint1"], controlMode=mode, targetPosition=3.14 / 7)
    p.setJointMotorControl2(robotId, jointIndex["inner_hip_joint1"], controlMode=mode, targetPosition=-3.14 / 6)
    p.setJointMotorControl2(robotId, jointIndex["inner_hip_joint2"], controlMode=mode, targetPosition=-3.14 / 6)
    p.setJointMotorControl2(robotId, jointIndex["outer_hip_joint2"], controlMode=mode, targetPosition=3.14 / 7)
    
    #knee joints move using setJointMotorControlArray
    p.setJointMotorControlArray(robotId, outer_knee_joints, controlMode=mode, targetPositions=[PI / 7, PI / 7]) 
    p.setJointMotorControlArray(robotId, inner_knee_joints, controlMode=mode, targetPositions=[PI / 3, PI / 3])

    #Get joints num
    #jnum = p.getNumJoints(robotId)
    #print('joints num = {}'.format(jnum))

    #Get joint information
    #jointinfo = p.getJointInfo(robotId, jindex)
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

    #step forward in the simlation
    p.stepSimulation()
    #wait a bit  to smooth the simulation
    time.sleep(1./240.)

p.disconnect()

