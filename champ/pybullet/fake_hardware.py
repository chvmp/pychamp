import pybullet as p
import numpy as np
from champ.pybullet.motor import MotorModel


def parse_info(base_id, joint_names):
    num_urdf_joints = p.getNumJoints(base_id)
    foot_idx = [3, 7, 11, 15]

    actuator_ids = [0]  * 12  
    foot_ids = [0] * 4 
    
    actuator_names = []
    foot_names = []

    for i, joint_name in enumerate(joint_names):
        if i in foot_idx:
            foot_names.append(joint_name)
        else:
            actuator_names.append(joint_name)

    for i in range(num_urdf_joints):
        joint_info = p.getJointInfo(base_id, i)
        joint_name = joint_info[1]
        joint_name = joint_name.decode("utf-8")
        
        if joint_name in actuator_names:
            actuator_ids[actuator_names.index(joint_name)] = i

        elif joint_name in foot_names:
            foot_ids[foot_names.index(joint_name)] = i

    return actuator_ids, foot_ids


class Sensors(object):
    def __init__(self, plane_id, base_id, joint_names, torque_control=False):
        self._base_id = base_id
        self._plane_id = plane_id
        self._joint_ids = [0] * 16

        self._actuator_ids, self._foot_ids = parse_info(base_id, joint_names)

    @property
    def actuator_ids(self):
        return self._actuator_ids         
        
    @property
    def foot_ids(self):
       return self._foot_ids

    def get_joint_states(self):
        joint_states = p.getJointStates(self._base_id, self._actuator_ids)
        return [joint_info[0] for joint_info in joint_states],\
               [joint_info[1] for joint_info in joint_states]

    def get_contact_states(self):
        contact_info = p.getContactPoints(self._base_id, self._plane_id)
        foot_contacts = [0] * 4

        for single_contact in contact_info:
            link_id = single_contact[3]
            if link_id in self._foot_ids:
                foot_contacts[self._foot_ids.index(link_id)] = 1
        
        return tuple(foot_contacts)

    def get_base_velocity(self):
        return p.getBaseVelocity(self._base_id) #tuple(tuple(linear), tuple(angular))

    def get_base_position(self):
        return p.getBasePositionAndOrientation(self._base_id)[0] #tuple(x,y,z)

    def get_base_rpy(self):
        quat = p.getBasePositionAndOrientation(self._base_id)[1]
        return p.getEulerFromQuaternion(quat) #tuple(roll, pitch, yaw)

    def get_base_quat(self):
        return p.getBasePositionAndOrientation(self._base_id)[1] #tuple(x, y, z, w)


class Actuators:
    def __init__(self, base_id, joint_names, torque_control_enabled=False, kp=1.2, kd=0):
        self._base_id = base_id
        self._actuator_ids, _ = parse_info(base_id, joint_names)
        self._actuator_model = MotorModel(torque_control_enabled, kp, kd)

    def position_control(self, joint_positions):
        p.setJointMotorControlArray(
            self._base_id, 
            self._actuator_ids, 
            p.POSITION_CONTROL, 
            list(joint_positions)
        )



    