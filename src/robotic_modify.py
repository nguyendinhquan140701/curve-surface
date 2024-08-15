import sys

sys.path.append("E:\Quan\AutoRoboticInspection-V1\curve-surface")

from robodk.robolink import *
from robodk.robomath import *
import numpy as np
import time
from library import robot_lib_modify as rob
from datetime import datetime
from config import config as CFG


def movHome():
    VisRob = VisionRobot()
    VisRob.homePos(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[1])

def stop():
    VisRob = VisionRobot()
    VisRob.disConnectRobot()

def target_curve():
    realTargetStart = [26, 102, 450]
    realTargetEnd = [26, -107, 450]

    yStep = abs(realTargetStart[1] - realTargetEnd[1]) / 100

    init_Z = realTargetStart[2]
    init_alpha = 0
    alpha = []
    yNewTarget = []
    zNewTarget = []
    newTarget = []


    for i in range(100):
        if i + 1 <= 30:
            alpha.append(0)
        elif i + 1 >= 31 and i + 1 <= 38:
            init_alpha += 0.5
            alpha.append(init_alpha)
            init_Z += 0.1
        else:  # i >= 17
            init_alpha += 0.5
            alpha.append(init_alpha)
            init_Z += 0.1

        # # Calculate the new Z position
        # if i % 5 == 0 and i != 0 and i >= 15:
        #     init_Z += 2

        zNewTarget.append(init_Z)

        # Calculate the new Y position
        yNewTarget.append(realTargetStart[1] - i * yStep)


        # Create the new target position and append to the newTarget list
        currentTarget = [realTargetStart[0], yNewTarget[-1], zNewTarget[-1]]
        newTarget.append(currentTarget)

    # print("(newTarget):", (newTarget))

    return realTargetStart, realTargetEnd, alpha, newTarget
           

def run(realTargetStart, realTargetEnd, alpha, newTarget, pos_status="home"):

    VisRob = VisionRobot()
    VisRob.homePos(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[1])
    if pos_status == "home":
        VisRob.fixedRef(0)

    else:
        stop()
    # print("alpha", alpha)
    target01, target02, _, _, target_to_send = VisRob.getTarget(realTargetStart, realTargetEnd, alpha[0], alpha[0], "0_degree")
    # VisRob.runMoveJ(target01, CFG.JOINT_SPEEDS[1])

    # joints, _ = VisRob.getParam()
    # print("joints:", joints)
    # robot_position = VisRob.robot.SolveFK(joints)
    # print("robot_position:", robot_position)

    # print("H_flangeToBase01:", H_flangeToBase01)
    # new_joints = VisRob.robot.SolveIK(H_flangeToBase01)
    # print("new_joints:", new_joints)
     
    _target = []
    for i in range(1, len(newTarget) - 1):
        pointA = newTarget[i]
        pointB = newTarget[i + 1]
        alpha_i = alpha[i]
        target_i0, target_i1, _, _, target_to_send = VisRob.getTarget(pointA, pointB, alpha_i, 0, "0_degree")
        # VisRob.runMoveJ(target_i0, CFG.LINEAR_SPEEDS[0])
        _target.append(target_to_send)

    print("_target:", _target[:3])

    
    for i in range(len(newTarget)-1, 1, -1):
        pointA = newTarget[i]
        pointB = newTarget[i -1]
        alpha_i = alpha[i]
        target_i0, target_i1, _, _, _ = VisRob.getTarget(pointA, pointB, alpha_i, 0, "0_degree")
        # VisRob.runMoveJ(target_i0, CFG.LINEAR_SPEEDS[0])

    # VisRob.homePos(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[1])

    data_export = {
        "id": str(datetime.now()),
    }
    print(f"data_export: {data_export}")
    # rob.RobotModule.export_csv(data_export)   


## class for vision robot
class VisionRobot:
    def __init__(self, RDK=None, robot=None):
        self.RDK = Robolink()
        self.robot = self.RDK.ItemUserPick("Motoman GP8 Base", ITEM_TYPE_ROBOT)
        self.robot_module = rob.RobotModule()
        self.connectRobot()

    def connectRobot(self) -> None:
        """
        Connect Robot and PC
        """
        if not self.robot.Valid():
            raise Exception("Invalid robot selected")

        RUN_ON_ROBOT = True
        if self.RDK.RunMode() != RUNMODE_SIMULATE:
            RUN_ON_ROBOT = False

        self.RDK.setRunMode(RUNMODE_RUN_ROBOT)
        if RUN_ON_ROBOT:
            # Connect to the robot using default IP
            self.robot.Connect("192.168.10.102")  # Try to connect once (Or 192.168.10.111)
            self.robot.ConnectSafe("192.168.10.102")  # Try to connect multiple times (Or 192.168.10.111)
            self.status, status_msg = self.robot.ConnectedState()

            if self.status != ROBOTCOM_READY:
                # Stop if the connection did not succeed
                raise Exception("Failed to connect: " + status_msg)

            # This will set to run the API programs on the robot and the simulator (online programming)
            self.RDK.setRunMode(RUNMODE_RUN_ROBOT)

    def disConnectRobot(self) -> None:
        """
        Disconnect PC RObot
        """
        if self.status == ROBOTCOM_READY:
            self.robot.Disconnect()
            # print(f"Stop:{self.robot.ConnectedState()}")

    def fixedRef(self, y_flange2rf):
        """
        Fixed refer
        """

        # reference frame flange to base
        pos_flange2rf, rot_flange2rf = self.robot_module.rotPosRef(380, y_flange2rf, -405, 0, 0, 0)
        self.rf_flange2rf = self.robot_module.createRef(pos_flange2rf, rot_flange2rf)

        # reference frame camera to flange
        pos_camera2flange, rot_camera2flange = self.robot_module.rotPosRef(0, -70.5, 88, 0, 0, 0)
        rf_camera2flange = self.robot_module.createRef(pos_camera2flange, rot_camera2flange)

        rf_camera2rf = np.dot(self.rf_flange2rf, rf_camera2flange)

        # reference laser to camera
        pos_laser2camera, rot_laser2camera = self.robot_module.rotPosRef(0, 151, 12, 0, 0, 0)
        self.rf_laser2camera = self.robot_module.createRef(pos_laser2camera, rot_laser2camera)

        self.rf_laser2flange = np.dot(rf_camera2flange, self.rf_laser2camera)
        pos_laser2flange, rot_laser2flange = self.robot_module.rotPos(self.rf_laser2flange)

        rf_laser2rf = np.dot(rf_camera2rf, self.rf_laser2camera)
        pos_laser2rf, rot_laser2rf = self.robot_module.rotPos(rf_laser2rf)

        rf_laser2rf_non_matrix = np.concatenate((pos_laser2rf, rot_laser2rf))
        self.rf_laser2rf_matrix = TxyzRxyz_2_Pose(rf_laser2rf_non_matrix)

        rf_laser2flange_non_matrix = np.concatenate((pos_laser2flange, rot_laser2flange))

        rf_laser2flange_matrix = TxyzRxyz_2_Pose(rf_laser2flange_non_matrix)

        pos_setFrame = [0, 0, 0]  # Translation vector [Tx, Ty, Tz] ## sai so
        rot_setFramee = [np.radians(180), np.radians(0), np.radians(0)]  # Rotation angles [Rx, Ry, Rz] in radians
        rf_setFrame = np.concatenate((pos_setFrame, rot_setFramee))
        setFrame = TxyzRxyz_2_Pose(rf_setFrame)

        self.robot.setPoseFrame(setFrame)
        self.robot.setPoseTool(rf_laser2flange_matrix)
        self.rf_laser2rf = rf_laser2rf
        return rf_laser2rf
    
    def test_fixedRef(self):
        """
        Fixed refer
        """

        # reference frame flange to base
        test_pos_flange2rf, test_rot_flange2rf = self.robot_module.rotPosRef(380, 0, 405, 180, 0, 0)
        self.test_rf_flange2rf = self.robot_module.createRef(test_pos_flange2rf, test_rot_flange2rf)

        # reference frame camera to flange
        test_pos_camera2flange, test_rot_camera2flange = self.robot_module.rotPosRef(0, -70.5, 88, 0, 0, 0)
        test_rf_camera2flange = self.robot_module.createRef(test_pos_camera2flange, test_rot_camera2flange)

        # reference frame camera to base
        test_rf_camera2rf = np.dot(self.test_rf_flange2rf, test_rf_camera2flange)

        # reference frame laser to camera
        test_pos_laser2camera, test_rot_laser2camera = self.robot_module.rotPosRef(0, 151, 12, 0, 0, 0)
        self.test_rf_laser2camera = self.robot_module.createRef(test_pos_laser2camera, test_rot_laser2camera)

        # reference frame laser to flange
        self.test_rf_laser2flange = np.dot(test_rf_camera2flange, self.test_rf_laser2camera)
        test_rf_laser2rf = np.dot(test_rf_camera2rf, self.test_rf_laser2camera)

        self.test_rf_laser2rf = test_rf_laser2rf


    def test_target(self, target_01, target_02, theta_laser, alphaA, alphaB):

        H_target01ToCamera = self.robot_module.createRef([target_01[0], target_01[1], target_01[2]],[0, 0, np.radians(theta_laser)])
        H_target02ToCamera = self.robot_module.createRef([target_02[0], target_02[1], target_02[2]],[0, 0, np.radians(theta_laser)])

        H_target01ToLaser = np.dot(np.linalg.inv(self.test_rf_laser2camera), H_target01ToCamera)
        H_target02ToLaser = np.dot(np.linalg.inv(self.test_rf_laser2camera), H_target02ToCamera)

        """ 
        def config_target(X, Y, theta_laser, backOx, alpha_rot_Oy):    
            L1 = np.abs(backOx)
            L2 = np.abs((np.tan(np.radians(CFG.ROTATE_OX_LASER)) * CFG.DISTANCE_LASER2OBJECT))
            delta_x = delta_y = 0
            theta_laser = abs(theta_laser)          
            if theta_laser > 0:
                if alpha_rot_Oy < 0:
                    delta_x = L1 *np.cos(np.radians(theta_laser)) + L2 * np.sin(np.radians(theta_laser))
                    delta_y = L1 *np.sin(np.radians(theta_laser)) - L2 * np.cos(np.radians(theta_laser))
                    new_X = X - delta_x
                    new_Y = Y + delta_y

                elif alpha_rot_Oy > 0:
                    delta_x = L1 *np.cos(np.radians(theta_laser)) - L2 * np.sin(np.radians(theta_laser))
                    delta_y = L1 *np.sin(np.radians(theta_laser)) + L2 * np.cos(np.radians(theta_laser))
                    new_X = X + delta_x
                    new_Y = Y + delta_y

                else:
                    delta_x = L2 * np.cos(np.radians(theta_laser))
                    delta_y = L2 * np.sin(np.radians(theta_laser))
                    new_X = X - delta_x
                    new_Y = Y + delta_y
                
            elif theta_laser < 0:
                if alpha_rot_Oy < 0:
                    delta_x = L1 *np.cos(np.radians(theta_laser)) - L2 * np.sin(np.radians(theta_laser))
                    delta_y = L1 *np.sin(np.radians(theta_laser)) + L2 * np.cos(np.radians(theta_laser))
                    new_X = X - delta_x
                    new_Y = Y + delta_y

                elif alpha_rot_Oy > 0:
                    delta_x = L1 *np.cos(np.radians(theta_laser)) + L2 * np.sin(np.radians(theta_laser))
                    delta_y = L1 *np.sin(np.radians(theta_laser)) - L2 * np.cos(np.radians(theta_laser))
                    new_X = X + delta_x
                    new_Y = Y - delta_y

                else:
                    delta_x = L2 * np.cos(np.radians(theta_laser))
                    delta_y = L2 * np.sin(np.radians(theta_laser))
                    new_X = X + delta_x
                    new_Y = Y + delta_y
            else:
                delta_y = L2 * np.sin(np.radians(theta_laser))
                new_X = X
                new_Y = Y + delta_y
            
            return [new_X, new_Y]
            """
        
        pos_targetToLaser01, rot_targetToLaser01 = self.robot_module.rotPos(H_target01ToLaser)
        pos_targetToLaser02, rot_targetToLaser02 = self.robot_module.rotPos(H_target02ToLaser)

        # calculate the new XY position of laser
        # newXY_01 = config_target(pos_targetToLaser01[0], pos_targetToLaser01[1], theta_laser, backOx, alpha_rot_Oy)
        # newXY_02 = config_target(pos_targetToLaser02[0], pos_targetToLaser02[1], theta_laser, backOx, alpha_rot_Oy)

        # pos_targetToLaser01[0] = newXY_01[0]
        # pos_targetToLaser01[1] = newXY_01[1]
        # pos_targetToLaser02[0] = newXY_02[0]
        # pos_targetToLaser02[1] = newXY_02[1]

        # create the H matrix about new laser position and new rotation
        H_newTargetToLaser01 = self.robot_module.createRef(pos_targetToLaser01, rot_targetToLaser01)
        H_newTargetToLaser02 = self.robot_module.createRef(pos_targetToLaser02, rot_targetToLaser02)

        R_targetToLaser01 = np.dot(H_newTargetToLaser01,  rotx(np.radians(alphaA)))
        R_targetToLaser02 = np.dot(H_newTargetToLaser02,  rotx(np.radians(alphaB)))

        # create the H matrix new target to basec
        H_newTarget01tobase = np.dot(self.test_rf_laser2rf, R_targetToLaser01)
        H_newTarget02tobase = np.dot(self.test_rf_laser2rf, R_targetToLaser02)

        # new flange position
        H_flangeToBase01 = np.dot(H_newTarget01tobase, np.linalg.inv(self.test_rf_laser2flange))
        H_flangeToBase02 = np.dot(H_newTarget02tobase, np.linalg.inv(self.test_rf_laser2flange))
        # print(f'rf_laser2flange:{self.rf_laser2flange}')
        newFlangePos01, newFlangeRot01 = self.robot_module.rotPos(H_flangeToBase01)
        newFlangePos02, newFlangeRot02 = self.robot_module.rotPos(H_flangeToBase02)
        # print(f'newFlangePos01:{newFlangePos01}, {newFlangeRot01},\n newFlangePos02:{newFlangePos02}, {newFlangeRot02}\n')
        return newFlangePos01, newFlangeRot01

   #####################################################
   #####################################################
    def newcreatePoint(self, target_01, target_02, theta_laser, backOx, alphaA, alphaB):
        """
        (new method) Create point with respect to reference frame
        """
        orgTarget01ToCamera = self.robot_module.intialTarget(target_01[0], target_01[1], target_01[2])
        orgTarget02ToCamera = self.robot_module.intialTarget(target_02[0], target_02[1], target_02[2])

        orgTarget01ToLaser = np.dot(np.linalg.inv(self.rf_laser2camera), orgTarget01ToCamera)
        orgTarget02ToLaser = np.dot(np.linalg.inv(self.rf_laser2camera), orgTarget02ToCamera)

        targetToOrgTarget = rotz(np.radians(theta_laser))

        target01toLaser = np.dot(orgTarget01ToLaser, targetToOrgTarget)
        target02toLaser = np.dot(orgTarget02ToLaser, targetToOrgTarget)
        # print(f'posTarget01toLaser, posTarget02toLaser:{posTarget01toLaser}, {posTarget02toLaser}')

        target01ToRf = np.dot(self.rf_laser2rf, target01toLaser)
        target02ToRf = np.dot(self.rf_laser2rf, target02toLaser)

        pos1toRf, _ = self.robot_module.rotPos(target01ToRf)
        pos2toRf, _ = self.robot_module.rotPos(target02ToRf)
        # print(f'pos1toRf, pos2toRf:{pos1toRf}, {pos2toRf}')

        pos2ToPos1_rf = pos2toRf - pos1toRf
        pos_target02_oy_rf = sqrt(pos2ToPos1_rf[0] ** 2 + pos2ToPos1_rf[1] ** 2)

        rfToBase = self.robot_module.createRef([0, 0, 0], [np.radians(180), 0, 0])
        target01ToBase = np.dot(rfToBase, target01ToRf)

        change_OX = CFG.DISTANCE_LASER2OBJECT * tan(np.radians(CFG.ROTATE_OX_LASER))
        # print("changeOx:", change_OX)
  
        newPos2 = np.array([0, -pos_target02_oy_rf + change_OX, 0])
        newRef = target01ToBase
        posRef, rotRef = self.robot_module.rotPos(newRef)
        newRef_nonMat = np.concatenate((posRef, rotRef), axis=0)
        setRef = TxyzRxyz_2_Pose(newRef_nonMat)
        self.robot.setPoseFrame(setRef)


        target01toNewRf = np.dot(target01ToBase, np.linalg.inv(target01ToBase))
        pos01toNewRf, rot01ToNewRf = self.robot_module.rotPos(target01toNewRf)
        target02toNewRf = self.robot_module.createRef(newPos2, rot01ToNewRf)


        newT1toNewRf = np.dot(target01toNewRf, rotx(np.radians(alphaA)))
        newT2toNewRf = np.dot(target02toNewRf, rotx(np.radians(alphaB)))

        newPos1ToNewRf, newRot1ToNewRf = self.robot_module.rotPos(newT1toNewRf)
        newPos2ToNewRf, newRot2ToNewRf = self.robot_module.rotPos(newT2toNewRf)
        # print(f'backOx:{backOx}')

        target01_none_mat = np.concatenate((newPos1ToNewRf, newRot1ToNewRf), axis=0)
        target02_none_mat = np.concatenate((newPos2ToNewRf, newRot2ToNewRf), axis=0)
        target01 = TxyzRxyz_2_Pose(target01_none_mat)
        target02 = TxyzRxyz_2_Pose(target02_none_mat)

        return target01, target02, pos_target02_oy_rf

    def homePos(self, linearSpeed, joinSpeed):
        self.setRobot(linearSpeed, joinSpeed)
        self.fixedRef(0)
        self.robot.MoveJ(self.rf_laser2rf_matrix, CFG.JOINT_SPEEDS[1])

    def getTarget(self, realTarget01, realTarget02, alphaA, alphaB, shape):
        # target01, target02, theta_laser = None, None, None
        obj = VisionRobot()
        obj.test_fixedRef()
        testTarget01, testTarget02, angleLaserToObject = None, None, None
        angleLaserToObject = 0    #### not input angleLaserToObject
        
        zLaserToObject = realTarget01[2] - CFG.DISTANCE_LASER2OBJECT
        # print(f"zLaserToObject:{zLaserToObject}")
        if zLaserToObject > CFG.SAFE_DISTANCE:
            realTarget01[2], realTarget02[2] = 330, 330
            print("Warning the collision. Check the distance")
        # print("xToCamera01, yToCamera01:", realTarget01)
        # print("xToCamera02, yToCamera02:", realTarget02)

        if shape == "0_degree":
            backOx = 0
            testTarget01, testTarget02, test_length_weld = self.newcreatePoint(realTarget01, realTarget02, angleLaserToObject, backOx, alphaA, alphaB)
            newFlangePos01, newFlangeRot01 = obj.test_target(realTarget01, realTarget02, angleLaserToObject, alphaA, alphaB)

            # print("newFlangePos01, newFlangeRot01:", newFlangePos01, newFlangeRot01)
            degree_rot = np.degrees(newFlangeRot01)
            target01_none_mat = np.concatenate((newFlangePos01, newFlangeRot01), axis=0)
            target_to_send = np.concatenate((newFlangePos01, degree_rot), axis=0)
            # print("target_to_send:", target_to_send)
            # target01 = TxyzRxyz_2_Pose(target01_none_mat)

        else:
            stop()
    
        return testTarget01, testTarget02, angleLaserToObject , test_length_weld, target_to_send

    def runMoveL(self, target, speedScan):
        self.setRobot(speedScan, CFG.JOINT_SPEEDS[1])
        self.robot.MoveL(target)
        return True

    def runMoveJ(self, target, speed_moveJ):
        self.setRobot(CFG.LINEAR_SPEEDS[0], speed_moveJ)
        self.robot.MoveJ(target)
        return True
    
    def runMoveC(self, target1, target2, speed_moveC):
        self.setRobot(CFG.LINEAR_SPEEDS[0], speed_moveC)
        self.robot.MoveC(target1, target2)
        return True

    def setRobot(self, linearSpeed, jointSpeed):
        self.robot.setRounding(40)  # Set the rounding parameter
        self.robot.setSpeed(linearSpeed)  # Set linear speed in mm/s
        self.robot.setSpeedJoints(jointSpeed)
        self.robot.setAccelerationJoints(1)

    def getParam(self):
        """Get custom binary data from this item. Use setParam to set the data"""
        current_joint_values = self.robot.Joints()
        limit = self.robot.JointLimits()

        return current_joint_values, limit

#############################################

if __name__ == "__main__":
    print("Testing for curve-surface")
    start, end, alpha, newTarget = target_curve()
    run(start, end, alpha, newTarget, pos_status="home")

