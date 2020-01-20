#passive dynamic walking
#KEEP WALKING!!

import pybullet as p
import time
import pybullet_data
import math
import csv

dirpath = ''

PI = math.pi

base_xyz = []
base_rpy = []
phase_changing = []
inner_hip_angle_log = []
inner_knee_angle_log = []
outer_hip_angle_log = []
outer_knee_angle_log = []
knee_passive_counter = []
swing_or_support_counter = []

#Contact Checker
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

#Write data in text file
def write_textfile(filename, data):
    #file path
    path = dirpath + filename
    
    #open write mode
    with open(path, mode='w', newline='') as f:
        for d in data:
            f.write(str(d)+'\n')

    print('data -> {}'.format(filename))

#Write data in csv file
def write_csvfile(filename, data):
    #file path
    path = dirpath + filename

    #open write mode
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        for d in data:
            writer.writerow(d)

    print('data -> {}'.format(filename))

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally 
#set Gravity
p.setGravity(0,0,-9.8)
#load plane URDF
planeId = p.loadURDF("plane.urdf")
#initial position of a robot
cubeStartPos = [0,0,0.9]
#initial orientation of a robot
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
#load a robot
robotId = p.loadURDF("urdf/test_walker1-2.urdf", cubeStartPos, cubeStartOrientation)

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
p.setJointMotorControl2(robotId, jointIndex["outer_hip_joint1"], controlMode=pmode, targetPosition=PI / 9)
p.setJointMotorControl2(robotId, jointIndex["outer_hip_joint2"], controlMode=pmode, targetPosition=PI / 9)
p.setJointMotorControl2(robotId, jointIndex["inner_hip_joint1"], controlMode=pmode, targetPosition=-1 * PI / 9)
p.setJointMotorControl2(robotId, jointIndex["inner_hip_joint2"], controlMode=pmode, targetPosition=-1 * PI / 9)
    
#Initial Position of knees
#knee joints move using setJointMotorControlArray
p.setJointMotorControlArray(robotId, outer_knee_joints, controlMode=pmode, targetPositions=[0, 0]) 
p.setJointMotorControlArray(robotId, inner_knee_joints, controlMode=pmode, targetPositions=[PI / 36, PI / 36])

#phase1 -> outer legs are swinging
#phase2 -> inner legs are swinging
phase = 1

count = 0

ic_hip_angle = PI / 9
ic_knee_angle = PI / 36
lr_knee_angle = PI / 12
tst_hip_angle = -1 * PI / 9
tst_knee_angle = PI / 36
knee_angle_zero = 0 

swing_hipangle = ic_hip_angle
swing_kneeangle = ic_knee_angle
support_hipangle = tst_hip_angle
support_kneeangle = tst_knee_angle

sleeptime = 500
swhiptime = 560
swkneetime = 590
suhiptime = 515
sukneetime = 540

swkneemovetime = (swkneetime - sleeptime) * 0.8
sukneemovetime = ((sukneetime - sleeptime) * 0.8) / 2   #this time is CW time and CCW time -> /2.0

sw_zero_to_five_time = (swkneetime - (sleeptime + swkneemovetime)) * 0.5
su_zero_to_five_time = (sukneetime - (sleeptime + sukneemovetime + sukneemovetime)) * 0.2
su_mid_time = (sukneetime - su_zero_to_five_time) - (sleeptime + sukneemovetime * 2)

#knee is passive or control flag
pa_or_con_flag = 1

#outer legs is swing or support
outer_sw_flag = 0

#simulation start -> simflag = 1
simflag = 0

#while True:
for i in range(5000):
    count = count + 1
   
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
    
    #check! if swing leg's feet contact ground, it would say OUT!
    #if (simflag == 1) and ((phase == 1 and conflag_outer_shin1 == 1) or (phase == 1 and conflag_outer_shin2 == 1)):
        #print("OUT!!")

    #swing or support reporter
    if (simflag == 1) and ((conflag_outer_shin1 == 1) or (conflag_outer_shin2 == 1)):
        #print(count)
        #outer legs is support
        outer_sw_flag = 0
    else:
        #outer legs is swing
        outer_sw_flag = 1

    if conflag_outer_shin1 == 1 and conflag_outer_shin2 == 1:
        outer_conflag = 1
    else:
        outer_conflag = 0

    if conflag_inner_shin1 == 1 and conflag_inner_shin2 == 1:
        inner_conflag = 1
    else:
        inner_conflag = 0

    #move angle / max count -> changing by average speed
    #swing legs
    #hip tst_hip_angle -> ic_hip_angle
    if count >= sleeptime and count <= swhiptime:
        swing_hipangle = swing_hipangle - PI * 2 / 9 / (swhiptime - sleeptime)
    #knee
    if count >= sleeptime and count <= swkneetime:
        if count <= sleeptime + 2:
            pa_or_con_flag = 1
            swing_kneeangle = ic_knee_angle
        #free joint
        elif count > sleeptime + 2 and count <= sleeptime + swkneemovetime:
            #swing_kneeangle = swing_kneeangle + PI / 3 / swkneemovetime
            pa_or_con_flag = 0
            #print(count)
        # -> 0
        elif count > sleeptime + swkneemovetime and count <= swkneetime - sw_zero_to_five_time:
            pa_or_con_flag = 1
            swing_kneeangle = knee_angle_zero
        #0 -> ic_knee_angle
        else:
            pa_or_con_flag = 1
            swing_kneeangle = swing_kneeangle + ic_knee_angle / sw_zero_to_five_time
   
    #support legs
    #hip ic_hip_angle -> tst_hip_angle
    if count >= sleeptime and count <= suhiptime:
        support_hipangle = support_hipangle + PI * 2 / 9 / (suhiptime - sleeptime)
    #knee
    if count >= sleeptime and count <= sukneetime:
        #ic_knee_angle -> lr_knee_angle
        if count <= sleeptime + sukneemovetime:
            support_kneeangle = support_kneeangle + (lr_knee_angle - ic_knee_angle) / sukneemovetime
        #reverse
        elif count > sleeptime + sukneemovetime and count <= sleeptime + sukneemovetime * 2:
            support_kneeangle = support_kneeangle - (lr_knee_angle - ic_knee_angle) / sukneemovetime
        #ic_knee_angle -> 0
        elif count > sleeptime + sukneemovetime * 2 and count <= sukneetime - su_zero_to_five_time:
            support_kneeangle = support_kneeangle - (ic_knee_angle / su_mid_time)
        #reverse
        else:
            support_kneeangle = support_kneeangle + (ic_knee_angle / su_zero_to_five_time)

       
    if (count >= sleeptime + 100) and ((phase == 1 and outer_conflag == 1) or (phase == 2 and inner_conflag == 1)): 
        simflag = 1
        #print("pahse change!!")
        
        count = sleeptime
        #reset angle 
         
        swing_hipangle = ic_hip_angle
        swing_kneeangle = ic_knee_angle
        support_hipangle = tst_hip_angle
        support_kneeangle = tst_knee_angle


        #change support legs
        if phase == 1:
            phase = 2
        elif phase == 2:
            phase = 1
    
    #outer legs: swing leg
    #inner legs: support leg
    if phase == 1:
        outer_hipangle = swing_hipangle
        outer_kneeangle = swing_kneeangle
        inner_hipangle = support_hipangle
        inner_kneeangle = support_kneeangle
        #Inner Legs
        p.setJointMotorControlArray(robotId, inner_hip_joints, controlMode=pmode, targetPositions=[inner_hipangle, inner_hipangle])  
        p.setJointMotorControlArray(robotId, inner_knee_joints, controlMode=pmode, targetPositions=[inner_kneeangle, inner_kneeangle]) 
    
        #Outer Legs
        p.setJointMotorControlArray(robotId, outer_hip_joints, controlMode=pmode, targetPositions=[outer_hipangle, outer_hipangle]) 
        
        #passive mode
        if pa_or_con_flag == 0:
            #torque 0 knee
            p.setJointMotorControlArray(robotId, outer_knee_joints, controlMode=vmode, forces=[0, 0]) 
        #position contorol mode
        else:
            p.setJointMotorControlArray(robotId, outer_knee_joints, controlMode=pmode, targetPositions=[outer_kneeangle, outer_kneeangle]) 

    #outer legs: supprt leg
    #inner legs: swing leg
    elif phase == 2: 
        outer_hipangle = support_hipangle
        outer_kneeangle = support_kneeangle
        inner_hipangle = swing_hipangle
        inner_kneeangle = swing_kneeangle
        #Outer Legs
        p.setJointMotorControlArray(robotId, outer_hip_joints, controlMode=pmode, targetPositions=[outer_hipangle, outer_hipangle])  
        p.setJointMotorControlArray(robotId, outer_knee_joints, controlMode=pmode, targetPositions=[outer_kneeangle, outer_kneeangle]) 
    
        #Inner Legs
        p.setJointMotorControlArray(robotId, inner_hip_joints, controlMode=pmode, targetPositions=[inner_hipangle, inner_hipangle]) 
        
        #passive mode
        if pa_or_con_flag == 0:
            #torque 0 knee
            p.setJointMotorControlArray(robotId, inner_knee_joints, controlMode=vmode, forces=[0, 0]) 
        #position contorol mode
        else:
            p.setJointMotorControlArray(robotId, inner_knee_joints, controlMode=pmode, targetPositions=[inner_kneeangle, inner_kneeangle]) 


    #get Base position
    basePos, baseQua = p.getBasePositionAndOrientation(robotId)
    baseEuler = p.getEulerFromQuaternion(baseQua)
    
    #set camera position 
    p.resetDebugVisualizerCamera(2.0, 20, -30, basePos)
     
    #Get joints num
    #jnum = p.getNumJoints(robotId)
    #print('joints num = {}'.format(jnum))

    #Get joint information
    #jointinfo = p.getJointInfo(robotId, jindex)
    #print(jointinfo[1]) 
    
    #RECORD DATA
    #############################################
    #basePos
    base_xyz.append(basePos)
    base_rpy.append(baseEuler)
    #angle
    inner_hip_angle_log.append(inner_hipangle)
    inner_knee_angle_log.append(inner_kneeangle)
    outer_hip_angle_log.append(outer_hipangle)
    outer_knee_angle_log.append(outer_kneeangle)
    #phase
    phase_changing.append(phase)
    #knee_passive_counter
    #flag -> 0 -> passive
    if phase == 1 and pa_or_con_flag == 0:
        knee_passive_counter.append('1')
    else:
        knee_passive_counter.append('0')
    #swing or support
    swing_or_support_counter.append(outer_sw_flag)

    ############################################
    #step forward in the simlation
    p.stepSimulation()
    #wait a bit  to smooth the simulation
    time.sleep(1./1000.)

#WRITE DATA
print('########################################')
#phase
fname = 'phase_changing.txt'
write_textfile(fname, phase_changing)
#baseXYZ
fname = 'base_xyz.csv'
write_csvfile(fname, base_xyz)
#baseRPY
fname = 'base_rpy.csv'
write_csvfile(fname, base_rpy)
#inner Hip angle
fname = 'inner_hipangle.txt'
write_textfile(fname, inner_hip_angle_log)
#inner Knee angle
fname = 'inner_kneeangle.txt'
write_textfile(fname, inner_knee_angle_log)
#outer Hip angle
fname = 'outer_hipangle.txt'
write_textfile(fname, outer_hip_angle_log)
#outer Knee angle
fname = 'outer_kneeangle.txt'
write_textfile(fname, outer_knee_angle_log)
#outer knee passive counter
fname = 'outer_knee_passive_counter.txt'
write_textfile(fname, knee_passive_counter)
#outer leg swing flag
fname = 'outer_legs_swing_flag.txt'
write_textfile(fname, swing_or_support_counter)

print('########################################')

p.disconnect()

