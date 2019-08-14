import pybullet as p
import pybullet_data
import time
import math
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
import pdb


class KukaBlock():
    def __init__(self):
      # connect to pybullet server
      p.connect(p.GUI)

      # set additional path to find kuka model
      p.setAdditionalSearchPath(pybullet_data.getDataPath())

      # add plane to push on (slightly below the base of the robot)
      self.planeId = p.loadURDF("plane.urdf", [0, 0, -0.05], useFixedBase=True)

      # add the robot at the origin with fixed base
      self.kukaId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)

      # reset the base
      p.resetBasePositionAndOrientation(self.kukaId, [0, 0, 0], [0, 0, 0, 1])

      # get useful robot information
      self.kukaEndEffectorIndex = 6
      self.numJoints = p.getNumJoints(self.kukaId)

      # add the block
      self.blockId = p.loadURDF("block_big.urdf", [-0.4, 0, .1])
      # p.resetBasePositionAndOrientation(self.blockId, [-0.4, 0, 0.1], [0, 0, 0, 1])
      # print(p.getNumJoints(self.blockId))

      # reset joint states to nominal pose
      self.rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
      for i in range(self.numJoints):
        p.resetJointState(self.kukaId, i, self.rp[i])

      # get the joint ids
      self.jointInds = [i for i in range(self.numJoints)]

      # set gravity
      p.setGravity(0, 0, -10)

      # set simulation length
      self.simLength = 6000

      # set joint damping
      #joint damping coefficents
      self.jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

      # pre-define the trajectory/force vectors
      self.traj = np.zeros((self.simLength, 5))
      self.contactForce = np.zeros((self.simLength, ))
      self.contactCount = np.empty_like(self.contactForce)

    def simulate(self, theta=0.1):

      self.getInitConds()
      pushStep = self.getPushStep()
      pushDir = self.getPushDir(theta)

      x = self.robotInitPoseCart[0]
      y = self.robotInitPoseCart[1]
      z = self.robotInitPoseCart[2]

      for simTime in range(self.simLength):
        p.stepSimulation()

        # set end effector pose
        d = pushDir * pushStep
        x += d[0]
        y += d[1]
        z += d[2]
        eePos = [x, y, z]

        # compute the inverse kinematics
        jointPoses = p.calculateInverseKinematics(self.kukaId,
                                                  self.kukaEndEffectorIndex,
                                                  eePos,
                                                  self.orn,
                                                  jointDamping=self.jd)

        for i in range(self.numJoints):
          p.setJointMotorControl2(bodyIndex=self.kukaId,
                                  jointIndex=i,
                                  controlMode=p.POSITION_CONTROL,
                                  targetPosition=jointPoses[i],
                                  targetVelocity=0,
                                  force=500,
                                  positionGain=0.3,
                                  velocityGain=1)

        # get joint states
        ls = p.getLinkState(self.kukaId, self.kukaEndEffectorIndex)
        lb = p.getLinkState(self.blockId, 0)
        lp = p.getLinkState(self.planeId, 0)
        print(lb)
        print(lp)
        pdb.set_trace()

        # get contact information
        contactInfo = p.getContactPoints(self.kukaId, self.blockId)

        # get the net contact force between robot and block
        if len(contactInfo)>0:
          f_c_temp = 0
          for i in range(len(contactInfo)):
            f_c_temp += contactInfo[i][9]
          self.contactForce[simTime] = f_c_temp
          self.contactCount[simTime] = len(contactInfo)

        self.traj[simTime, :] = np.array([x, y, z, x, y])
        self.contactForce[:1000] = 0
        return self.traj

    def getInitConds(self):
      self.robotInitPoseCart = [-0.4, -0.2, 0.01] # (x,y,z)
      self.t = 0
      self.orn = p.getQuaternionFromEuler([0, -math.pi, 0])

    def getPushDir(self, theta):
      # get unit direction of the push
      return np.array([math.sin(theta), math.cos(theta), 0.])

    def getPushStep(self):
      return 0.00005


if __name__ == "__main__":
    kb = KukaBlock()

    kb.simulate()
