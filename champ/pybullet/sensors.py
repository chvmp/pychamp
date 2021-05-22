import pybullet as p
import numpy as np

class PyBulletSensors:
    def __init__(self, plane_id, base_id, joint_names, link_names):
        self._base_id = base_id
        self._plane_id = plane_id
        self._joint_ids = [0] * 16
        self._actuator_ids = [0]  * 12  
        self._foot_ids = [0] * 4 

        num_urdf_joints = p.getNumJoints(base_id)
        foot_idx = [3, 7, 11, 15]

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
                self._actuator_ids[actuator_names.index(joint_name)] = i

            elif joint_name in foot_names:
                self._foot_ids[foot_names.index(joint_name)] = i

    @property
    def actuator_ids(self):
        return self._actuator_ids         
        
    @property
    def foot_ids(self):
       return self._foot_ids

    def joint_positions(self):
        joint_states = p.getJointStates(self._base_id, self._actuator_ids)
        return [joint_info[0] for joint_info in joint_states]

    def contact_states(self):
        contact_info = p.getContactPoints(self._base_id, self._plane_id)
        foot_contacts = np.zeros(([4,1]))

        for single_contact in contact_info:
            link_id = single_contact[3]
            if link_id in self._foot_ids:
                foot_contacts[self._foot_ids.index(link_id), :] = 1
        
        return foot_contacts

    def base_velocity(self):
        return p.getBaseVelocity(self._base_id) #tuple(linear, angular)

    def base_position(self):
        return p.getBasePositionAndOrientation(self._base_id)[0] #tuple(x,y,z)

    def base_orientation_rpy(self):
        quat = p.getBasePositionAndOrientation(self._base_id)[1]
        return p.getEulerFromQuaternion(quat) #roll, pitch, yaw

    def base_orientation_quat(self):
        return p.getBasePositionAndOrientation(self._base_id)[1] #x, y, z, w
