import pybullet as p
from champ.utils import clip
from champ.types import Pose, Velocities

def follow_robot(id):
    p.resetDebugVisualizerCamera(
        cameraDistance=1.51, 
        cameraYaw=33.14, 
        cameraPitch=-15.51, 
        cameraTargetPosition= p.getBasePositionAndOrientation(id)[0]
    )

def get_req_vel(id, req_vel, max_linear_x=0.5, max_linear_y=0.5, max_angular_z=1.0):
    keys = p.getKeyboardEvents()

    for k,v in keys.items():
        if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_IS_DOWN)):
            req_vel.angular.z -= 0.05
        if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_RELEASED)):
            req_vel.angular.z = 0.0

        if (k == p.B3G_LEFT_ARROW and (v&p.KEY_IS_DOWN)):
            req_vel.angular.z += 0.05
        if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_RELEASED)):
            req_vel.angular.z = 0.0

        if (k == p.B3G_UP_ARROW and (v&p.KEY_IS_DOWN)):
            req_vel.linear.x += 0.5
        if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_RELEASED)):
            req_vel.linear.x = 0.0

        if (k == p.B3G_DOWN_ARROW and (v&p.KEY_IS_DOWN)):
            req_vel.linear.x -= 0.5
        if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_RELEASED)):
            req_vel.linear.x = 0.0
        
    req_vel.angular.z = clip(req_vel.angular.z, -max_angular_z, max_angular_z)
    req_vel.linear.x = clip(req_vel.linear.x, -max_linear_x, max_linear_x)
    
    return req_vel
