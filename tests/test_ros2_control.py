#!/usr/bin/env python

#####################
#  LOADING MODULES  #
#####################

import pinocchio as pin
import yaml


from bullet_robot import PyBulletSimulatorAndRos2Control


################
#  PARAMETERS  #
################

enableGUI = True
T_total = 3000

modelPath = "/opt/openrobots/share/example-robot-data/robots/talos_data/"
URDF = modelPath + "robots/talos_reduced.urdf"
SRDF = modelPath + "srdf/talos.srdf"

# Parameters filename
filename = "../config/settings_sobec.yaml"

####################
#  INITIALIZATION  #
####################

# Loading extra parameters from file
with open(filename, "r") as paramFile:
    params = yaml.safe_load(paramFile)

controlledJoints = params["robot"]["controlledJoints"]
toolFramePos = params["robot"]["toolFramePos"]


# Robot model
design_conf = dict(
    urdfPath=URDF,
    srdfPath=SRDF,
    leftFootName="right_sole_link",
    rightFootName="left_sole_link",
    robotDescription="",
    controlledJointsNames=controlledJoints,
)
pinWrapper = RobotDesigner()
pinWrapper.initialize(design_conf)

gripper_SE3_tool = pin.SE3.Identity()
gripper_SE3_tool.translation[0] = toolFramePos[0]
gripper_SE3_tool.translation[1] = toolFramePos[1]
gripper_SE3_tool.translation[2] = toolFramePos[2]
pinWrapper.addEndEffectorFrame(
    "deburring_tool", "gripper_left_fingertip_3_link", gripper_SE3_tool
)


print("Robot successfully loaded")

# Simulator
simulator = PyBulletSimulatorAndRos2Control(
    URDF=URDF,
    initialConfiguration=pinWrapper.get_q0Complete(),
    robotJointNames=pinWrapper.get_rModelComplete().names,
    controlledJointsIDs=pinWrapper.get_controlledJointsIDs(),
    enableGUI=enableGUI,
)


###############
#  MAIN LOOP  #
###############

NcontrolKnots = 10
T = 0

while T < T_total:
    # Controller works faster than trajectory generation
    for i_control in range(NcontrolKnots):
        x_measured = simulator.getRobotState()

        # Compute torque to be applied by adding Riccati term
        torques = # Will come from ros2_control

        # Apply torque on complete model
        simulator.step(torques)


    T += 1

simulator.end()
print("Simulation ended")

plotter.plotResults()
