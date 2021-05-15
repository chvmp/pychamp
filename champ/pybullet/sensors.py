import pybullet as p

class PyBulletSensors:
    def __init__(self, bullet_id, joint_names, link_names):
        self._bullet_id = bullet_id
        self._joint_ids = [0] * 16
        self._actuator_ids = [0]  * 12  
        self._foot_ids = [0] * 4 

        num_urdf_joints = p.getNumJoints(bullet_id)
        foot_idx = [3, 7, 11, 15]

        actuator_names = []
        foot_names = []

        for i, joint_name in enumerate(joint_names):
            if i in foot_idx:
                foot_names.append(joint_name)
            else:
                actuator_names.append(joint_name)

        for i in range(num_urdf_joints):
            joint_info = p.getJointInfo(bullet_id, i)
            joint_name = joint_info[1]
            joint_name = joint_name.decode("utf-8")
            
            if joint_name in actuator_names:
                self._actuator_ids[actuator_names.index(joint_name)] = i

            elif joint_name in foot_names:
                self._foot_ids[foot_names.index(joint_name)] = i

    @property
    def actuator_ids(self):
        return self._actuator_ids         
        
    def get_joint_positions(self):
        joint_states = p.getJointStates(self._bullet_id, self._actuator_ids)
        return [joint_info[0] for joint_info in joint_states]

   