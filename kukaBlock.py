import pybullet as p
import pybullet_data
import time
import math
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
import pdb


def main():
  p.connect(p.GUI)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  p.loadURDF("plane.urdf", [0, 0, -0.05], useFixedBase=True)
  kukaId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
  p.resetBasePositionAndOrientation(kukaId, [0, 0, 0], [0, 0, 0, 1])
  kukaEndEffectorIndex = 6
  numJoints = p.getNumJoints(kukaId)
  if (numJoints != 7):
    exit()

  blockId = p.loadURDF("block_big.urdf", [-0.4, 0, .1])

  #lower limits for null space
  ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
  #upper limits for null space
  ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
  #joint ranges for null space
  jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
  #restposes for null space
  rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
  #joint damping coefficents
  jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

  for i in range(numJoints):
    p.resetJointState(kukaId, i, rp[i])

  p.setGravity(0, 0, -10)
  t = 0.
  prevPose = [0, 0, 0]
  prevPose1 = [0, 0, 0]
  hasPrevPose = 0
  useNullSpace = 0

  count = 0
  useOrientation = 1
  useSimulation = 1
  useRealTimeSimulation = 1
  useLogger = False
  p.setRealTimeSimulation(useRealTimeSimulation)

  #trailDuration is duration (in seconds) after debug lines will be removed automatically
  #use 0 for no-removal
  trailDuration = 15

  if useLogger:
    logId1 = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT, "LOG0001.txt", [0, 1, 2])
    logId2 = p.startStateLogging(p.STATE_LOGGING_CONTACT_POINTS, "LOG0002.txt", bodyUniqueIdA=2)

  simLength = 6000
  x = -0.4
  y = -0.2
  z = 0.01
  t = 0
  orn = p.getQuaternionFromEuler([0, -math.pi, 0])

  traj = np.zeros((simLength, 3))
  contactForceRobotBlock = np.zeros((simLength, ))
  contactCountRobotBlock = np.empty_like(contactForceRobotBlock)

  jointInds = [i for i in range(numJoints)]
  posGains = [0.3] * numJoints
  velGains = [1.0] * numJoints

  for simTime in range(simLength):
    p.stepSimulation()

    # set end effector pose
    dx = 0.00
    dy = 0.00005
    dz = 0.00
    x += dx
    y += dy
    z += dz
    eePos = [x, y, z]

    # compute the inverse kinematics
    jointPoses = p.calculateInverseKinematics(kukaId,
                                              kukaEndEffectorIndex,
                                              eePos,
                                              orn,
                                              jointDamping=jd)

    for i in range(numJoints):
      p.setJointMotorControl2(bodyIndex=kukaId,
                              jointIndex=i,
                              controlMode=p.POSITION_CONTROL,
                              targetPosition=jointPoses[i],
                              targetVelocity=0,
                              force=500,
                              positionGain=0.3,
                              velocityGain=1)

    # get joint states
    ls = p.getLinkState(kukaId, kukaEndEffectorIndex)

    # get contact information
    contactInfo = p.getContactPoints(kukaId, blockId)
    # pdb.set_trace()
    if len(contactInfo)>0:
      # print('Num of Contact points: {}'.format(len(contactInfo)))
      # print('Contact point: {}'.format(contactInfo[:][5]))
      # print('Contact Normal: {}'.format(contactInfo[:][7]))
      # print('Contact Force: {}'.format(contactInfo[:][9]))

      f_c_temp = 0
      for i in range(len(contactInfo)):
        f_c_temp += contactInfo[i][9]
      contactForceRobotBlock[simTime] = f_c_temp
      contactCountRobotBlock[simTime] = len(contactInfo)

  contactForceRobotBlock[:1000] = 0

  fig_force = plt.figure()
  ax_force = fig_force.add_subplot()
  ax_force.plot(contactForceRobotBlock)
  ax_force.set_xlabel('Time-step')
  ax_force.set_ylabel('Net normal force')

  fig_cont = plt.figure()
  ax_cont = fig_cont.add_subplot()
  ax_cont.plot(contactCountRobotBlock)
  ax_cont.set_xlabel('Time-step')
  ax_cont.set_ylabel('Number of registered contacts')


  plt.show()

if __name__=='__main__':
  main()
