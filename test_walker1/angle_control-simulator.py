#walking 7 or 8 steps

import pybullet as p
import time
import pybullet_data
import math

PI = math.pi

def getcontact(robotid, planeid, linkindex):
    #get contact infomation
    coninfo = p.getContactPoints(robotid, planeid, linkindex)
    #print('contact infomation')
    #print(coninfo)

    coninfolen = len(coninfo)
    #contact -> 1, no contact -> 0
    conflag = 0
    if coninfolen != 0:
        if coninfo[0][0] == 0:
            #print('contact!')
            conflag = 1
    #else:
        #print('no contct..')

    return conflag

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

#CONTROL MODE
pmode = p.POSITION_CONTROL
vmode = p.VELOCITY_CONTROL
tmode = p.TORQUE_CONTROL


simpara = p.getPhysicsEngineParameters(physicsClient)
print('simulation parameters')
print(simpara)

#joint indices
jointIndex = {"outer_hip_joint1":0, "outer_knee_joint1":1, "inner_hip_joint1":2, "inner_knee_joint1":3, "inner_hip_joint2":4, "inner_knee_joint2":5, "outer_hip_joint2":6, "outer_knee_joint2":7}

#link indices
linkIndex = {"outer_thigh1":0, "outer_shin1":1, "inner_thigh1":2, "inner_shin1":3, "inner_thigh2":4, "inner_shin2":5, "outer_thigh2":6, "outer_shin2":7}

#joint indices list
outer_hip_joints = [jointIndex["outer_hip_joint1"], jointIndex["outer_hip_joint2"]]
outer_knee_joints = [jointIndex["outer_knee_joint1"], jointIndex["outer_knee_joint2"]]
inner_hip_joints = [jointIndex["inner_hip_joint1"], jointIndex["inner_hip_joint2"]]
inner_knee_joints = [jointIndex["inner_knee_joint1"], jointIndex["inner_knee_joint2"]]

#Initial Position of hips
#hip joints move using setJointMotorControl2
p.setJointMotorControl2(robotId, jointIndex["outer_hip_joint1"], controlMode=pmode, targetPosition=PI * 5 / 36)
p.setJointMotorControl2(robotId, jointIndex["inner_hip_joint1"], controlMode=pmode, targetPosition=-1 * PI / 12)
p.setJointMotorControl2(robotId, jointIndex["inner_hip_joint2"], controlMode=pmode, targetPosition=-1 * PI / 12)
p.setJointMotorControl2(robotId, jointIndex["outer_hip_joint2"], controlMode=pmode, targetPosition=PI * 5 / 36)
    
#Initial Position of knees
#knee joints move using setJointMotorControlArray
#p.setJointMotorControlArray(robotId, outer_knee_joints, controlMode=pmode, targetPositions=[PI / 7, PI / 7]) 
#p.setJointMotorControlArray(robotId, inner_knee_joints, controlMode=pmode, targetPositions=[PI / 3, PI / 3])
p.setJointMotorControlArray(robotId, outer_knee_joints, controlMode=pmode, targetPositions=[0,0]) 
p.setJointMotorControlArray(robotId, inner_knee_joints, controlMode=pmode, targetPositions=[0,0])

#phase1 -> outer legs is swinging
#phase2 -> inner legs is swinging
phase = 1

count = 0
outer_kneeangle = 0
inner_hipangle = -1 * PI / 12
outer_hipangle = PI * 5 / 36

while True:
    count = count + 1

    #move angle / max count -> changing by average speed
    if count <= 500:
        inner_kneeangle = 0
        if count <= 260:
            outer_kneeangle = outer_kneeangle + PI / 4 / 260
        else:
            outer_kneeangle = outer_kneeangle - PI / 4 / 240

        inner_hipangle = inner_hipangle + PI * 8 / 36 / 500 
        
        if count <= 310:
            outer_hipangle = outer_hipangle - PI * 8 / 36 / 310
    ###############################################################
    elif count >= 700 and count < 1200:
        outer_kneeangle = 0
        if count <= 960:
            inner_kneeangle = inner_kneeangle + PI / 4 / 260
        else:
            inner_kneeangle = inner_kneeangle - PI / 4 / 240

        outer_hipangle = outer_hipangle + PI * 8 / 36 / 500

        if count <= 1010:
            inner_hipangle = inner_hipangle - PI * 8 / 36 / 310
    elif count == 1400:
        count = 0

    #get Base position
    basePos, baseQua = p.getBasePositionAndOrientation(robotId)
    #set camera position 
    p.resetDebugVisualizerCamera(2.0, 20, -30, basePos)
     
    #Get joints num
    #jnum = p.getNumJoints(robotId)
    #print('joints num = {}'.format(jnum))

    #Get joint information
    #jointinfo = p.getJointInfo(robotId, jindex)
    #print(jointinfo[1])
    
    #Swing Legs
    #inner legs
    p.setJointMotorControlArray(robotId, inner_hip_joints, controlMode=pmode, targetPositions=[inner_hipangle, inner_hipangle]) 
    #p.setJointMotorControlArray(robotId, inner_hip_joints, controlMode=vmode, targetVelocities=[PI/6, PI/6]) 
    p.setJointMotorControlArray(robotId, inner_knee_joints, controlMode=pmode, targetPositions=[inner_kneeangle, inner_kneeangle]) 
    
    #outer legs
    #p.setJointMotorControlArray(robotId, outer_hip_joints, controlMode=vmode, targetVelocities=[-1 * PI/6, -1 * PI/6]) 
    p.setJointMotorControlArray(robotId, outer_hip_joints, controlMode=pmode, targetPositions=[outer_hipangle, outer_hipangle]) 
    #torque 0 knee
    #p.setJointMotorControlArray(robotId, outer_knee_joints, controlMode=vmode, forces=[0, 0]) 
    p.setJointMotorControlArray(robotId, outer_knee_joints, controlMode=pmode, targetPositions=[outer_kneeangle, outer_kneeangle]) 


    #get contact infomation
    conflag_outer_shin1 = getcontact(robotId, planeId, linkIndex["outer_shin1"])
    conflag_outer_shin2 = getcontact(robotId, planeId, linkIndex["outer_shin2"])
    conflag_inner_shin1 = getcontact(robotId, planeId, linkIndex["inner_shin1"])
    conflag_inner_shin2 = getcontact(robotId, planeId, linkIndex["inner_shin2"])
    
    #attach the bottun to feet
    #if conflag_outer_shin1 == 1:
        #print("outer shin1 contact!")
    #if conflag_outer_shin2 == 1:
        #print("outer shin2 contact!")
    #else:
        #print("outer shin2 no contact")
    #if conflag_inner_shin1 == 1:
        #print("inner shin1 contact!")
    #else:
        #print("inner shin1 no contact")
    #if conflag_inner_shin2 == 1:
        #print("inner shin2 contact!")
    #else:
        #print("inner shin2 no contact")


    #step forward in the simlation
    p.stepSimulation()
    #wait a bit  to smooth the simulation
    time.sleep(1./240.)

p.disconnect()

