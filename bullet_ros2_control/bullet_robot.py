import numpy as np
import pinocchio as pin
import pybullet as p  # PyBullet simulator
import pybullet_data


class PyBulletSimulatorAndRos2Control:
    """Pybullet based simulator integrating ros2_control"""

    def __init__(
        self,
        URDF,
        initialConfiguration,
        robotJointNames,
        controlledJointsIDs,
        enableGUI=False,
        enableGravity=True,
        dt=1e-3,
    ):
        """Constructor of the simulator

        Arguments:
            URDF -- Path to the URDF of the robot
            initialConfiguration -- Initial configuration of the robot
            robotJointNames -- Full list of the joint names of the robot
            controlledJointsIDs -- ID list of the torque controlled joints
            toolPlacement -- Position of the tool

        Keyword Arguments:
            enableGUI -- _description_ (default: {False})
            enableGravity -- Parameter to enable Gravity (default: {True})
            dt -- Integration time step for the simulator (default: {1e-3})
        """

        self._setupBullet(enableGUI, enableGravity, dt)
        self._setupRobot(
            URDF, initialConfiguration, robotJointNames, controlledJointsIDs
        )


    def _setupBullet(self, enableGUI, enableGravity, dt):
        """Setup the bullet environment

        Arguments:
            enableGUI -- Parameter to enable Graphical User Interface
            enableGravity -- Parameter to enable Gravity
            dt -- Integration time step for the simulator
        """
        # Start the PyBullet client
        if enableGUI:
            self.physicsClient = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        else:
            self.physicsClient = p.connect(p.DIRECT)

        if enableGravity:
            p.setGravity(0, 0, -9.81)
        else:
            p.setGravity(0, 0, 0)

        p.setTimeStep(dt)

        # Load horizontal plane for PyBullet
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")

    def _setupRobot(
        self, URDF, initialConfiguration, robotJointNames, controlledJointsIDs
    ):
        """Setup the robot inside the simulator

        Arguments:
            URDF -- Complete path to the URDF of the robot
            initialConfiguration -- Pinocchio model of the robot containing all the joints
            robotJointNames -- Names of all the joint of the robot
            controlledJointsIDs --  ID of the torque controlled joints (the other joints will stay fixed)
        """
        # Loading initial configuration from pinocchio
        self.q0 = initialConfiguration
        self.initial_base_position = list(self.q0[:3])
        self.initial_base_orientation = list(self.q0[3:7])
        self.initial_joint_positions = list(self.q0[7:])

        # Loading robot
        self.robotId = p.loadURDF(
            URDF,
            self.initial_base_position,
            self.initial_base_orientation,
            useFixedBase=False,
        )

        # Fetching the position of the center of mass
        # (which is different from the origin of the root link)
        self.localInertiaPos = p.getDynamicsInfo(self.robotId, -1)[3]

        # Expressing initial position wrt the CoM
        for i in range(3):
            self.initial_base_position[i] += self.localInertiaPos[i]

        # Helpers to switch from pinocchio to bullet joint indexes
        self.names2bulletIndices = {
            p.getJointInfo(1, i)[1].decode(): i for i in range(p.getNumJoints(1))
        }
        self.bulletJointsIdInPinOrder = [
            self.names2bulletIndices[n] for n in robotJointNames[2:]
        ]

        # Indexes of the torque-controlled joints in bullet
        self.bullet_controlledJoints = [
            self.names2bulletIndices[robotJointNames[i]]
            for i in controlledJointsIDs[1:]
        ]

        self._setInitialConfig()
        self._changeFriction(["leg_left_6_joint", "leg_right_6_joint"], 100, 30)
        self._setControlledJoints()

    def _setInitialConfig(self):
        """Set initial robot configuration in pyBullet"""
        for i in range(len(self.initial_joint_positions)):
            p.enableJointForceTorqueSensor(self.robotId, i, True)
            p.resetJointState(
                self.robotId,
                self.bulletJointsIdInPinOrder[i],
                self.initial_joint_positions[i],
            )

    def _changeFriction(self, names, lateralFriction=100, spinningFriction=30):
        for n in names:
            idx = self.names2bulletIndices[n]
            p.changeDynamics(
                self.robotId,
                idx,
                lateralFriction=lateralFriction,
                spinningFriction=spinningFriction,
            )

    def _setControlledJoints(self):
        """Set robot joints controlled by pyBullet"""
        # Disable default position controler in torque controlled joints
        # Default controller will take care of other joints
        p.setJointMotorControlArray(
            self.robotId,
            jointIndices=self.bullet_controlledJoints,
            controlMode=p.VELOCITY_CONTROL,
            forces=[0.0 for m in self.bullet_controlledJoints],
        )

    def _setObjectPosition(self, objectName, oMobject):
        """Move an object to the given position

        Arguments:
            objectName -- Name of the object to move
            oMobject -- Position of the object in the world
        """

        p.resetBasePositionAndOrientation(
            objectName,
            oMobject.translation,
            pin.Quaternion(oMobject.rotation).coeffs(),
        )

    def getRobotState(self):
        """Get current state of the robot

        Takes the measurments of pyBullet and rearrange them in the order of pinocchio"""
        # Get articulated joint pos and vel
        xbullet = p.getJointStates(self.robotId, self.bullet_controlledJoints)
        q = [x[0] for x in xbullet]
        vq = [x[1] for x in xbullet]

        # Get basis pose
        pos, quat = p.getBasePositionAndOrientation(self.robotId)
        # Get basis vel
        v, w = p.getBaseVelocity(self.robotId)

        # Concatenate into a single x vector
        x = np.concatenate([pos, quat, q, v, w, vq])

        # Transformation of the basis position because the reference is not the same in bullet and in pinocchio
        x[:3] -= self.localInertiaPos

        return x

    def step(self, torques):
        """One step of the simulator

        Arguments:
            torques -- Torques to apply to the robot
            oMtool -- Position of the tool (only used to update GUI)
            oMtool -- Position of the target (only used to update GUI)
        """
        self._applyTorques(torques)
        p.stepSimulation()

    def _applyTorques(self, torques):
        """Apply computed torques to the robot

        Arguments:
            torques -- Torques to apply to the robot"""
        p.setJointMotorControlArray(
            self.robotId,
            self.bullet_controlledJoints,
            controlMode=p.TORQUE_CONTROL,
            forces=torques,
        )

    def reset(self):
        """Reset simulation to initial state"""

        # Reset base
        p.resetBasePositionAndOrientation(
            self.robotId,
            self.initial_base_position,
            self.initial_base_orientation,
            self.physicsClient,
        )
        p.resetBaseVelocity(
            self.robotId, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], self.physicsClient
        )

        # Reset joints
        for i in range(len(self.initial_joint_positions)):
            p.resetJointState(
                self.robotId,
                self.bulletJointsIdInPinOrder[i],
                self.initial_joint_positions[i],
            )

    def end(self):
        """Ends connection with pybullet."""
        p.disconnect()
