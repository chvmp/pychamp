import time
import sympy as sp
import numpy as np
from champ.base import Base
from champ.robots.profile import AnymalC as anymal_c

def gen_jacobian_function():
    q = [sp.Symbol('h_j'), sp.Symbol('ul_j'), sp.Symbol('ll_j')]

    htx, hty, htz = sp.symbols('htx hty htz')
    ultx, ulty, ultz = sp.symbols('ultx ulty ultz')
    lltx, llty, lltz= sp.symbols('lltx llty lltz')
    ftx, fty, ftz = sp.symbols('ftx fty ftz')
 
    hip_t = sp.Matrix(
        [[1,            0,             0, htx],
         [0, sp.cos(q[0]), -sp.sin(q[0]), hty],
         [0, sp.sin(q[0]),  sp.cos(q[0]), htz],
         [0,            0,             0, 1]]
    )
 
    upper_leg_t = sp.Matrix(
        [[ sp.cos(q[1]), 0, sp.sin(q[1]), ultx],
         [            0, 1, 0           , ulty],
         [-sp.sin(q[1]), 0, sp.cos(q[1]), ultz],
         [            0, 0, 0           , 1]]
    )
 
    lower_leg_t = sp.Matrix(
        [[ sp.cos(q[2]), 0, sp.sin(q[2]), lltx],
         [            0, 1, 0           , llty],
         [-sp.sin(q[2]), 0, sp.cos(q[2]), lltz],
         [            0, 0, 0           , 1]]
    )
 
    foot_t = sp.eye(4)
    foot_t[0, 3] = ftx
    foot_t[1, 3] = fty
    foot_t[2, 3] = ftz
    
    leg_t = hip_t  *  upper_leg_t  *  lower_leg_t  *  foot_t

    foot_trans = leg_t[0:3, 3]
    leg_jacobian = sp.simplify(foot_trans.jacobian(q))

    for i in range(3):
        for ii in range(3):
            print(f'j[{i}, {ii}] = {leg_jacobian[i,ii]}')

def get_jacobian(q, hip_trans, upper_leg_trans, lower_leg_trans, foot_trans):
    j = np.zeros((4,3,3))
    sin_lu = np.sin(q[:, 2] + q[:, 1])
    cos_lu = np.cos(q[:, 2] + q[:, 1])

    sin_h = np.sin(q[:, 0])
    cos_h = np.cos(q[:, 0])
    sin_u = np.sin(q[:, 1])
    cos_u = np.cos(q[:, 1])
   
    foot_x_sin_lu = foot_trans[:, 0]  *  sin_lu
    foot_x_cos_lu = foot_trans[:, 0]  *  cos_lu 
    foot_z_sin_lu = foot_trans[:, 2]  *  sin_lu

    lower_leg_x_cos_u = lower_leg_trans[:, 0]  *  cos_u
    lower_leg_z_sin_u = lower_leg_trans[:, 2]  *  sin_u

    j[:,0, 0] = 0.0
    j[:,0, 1] = -foot_x_sin_lu + foot_trans[:, 2] * cos_lu - lower_leg_trans[:, 0] * sin_u + lower_leg_trans[:, 2] * cos_u
    j[:,0, 2] = -foot_x_sin_lu + foot_trans[:, 2] * cos_lu
    j[:,1, 0] = foot_x_sin_lu * cos_h - foot_trans[:, 1] * sin_h - foot_trans[:, 2] * cos_h * cos_lu + lower_leg_trans[:, 0] * sin_u * cos_h - lower_leg_trans[:, 1] * sin_h - lower_leg_trans[:, 2] * cos_h * cos_u - upper_leg_trans[:, 1] * sin_h - upper_leg_trans[:, 2] * cos_h
    j[:,1, 1] = (foot_x_cos_lu + foot_z_sin_lu + lower_leg_x_cos_u + lower_leg_z_sin_u) * sin_h
    j[:,1, 2] = (foot_x_cos_lu + foot_z_sin_lu) * sin_h
    j[:,2, 0] = foot_trans[:, 0] * sin_h * sin_lu + foot_trans[:, 1] * cos_h - foot_trans[:, 2] * sin_h * cos_lu + lower_leg_trans[:, 0] * sin_h * sin_u + lower_leg_trans[:, 1] * cos_h - lower_leg_trans[:, 2] * sin_h * cos_u + upper_leg_trans[:, 1] * cos_h - upper_leg_trans[:, 2] * sin_h
    j[:,2, 1] = -(foot_x_cos_lu + foot_z_sin_lu + lower_leg_x_cos_u + lower_leg_z_sin_u) * cos_h
    j[:,2, 2] = -(foot_x_cos_lu + foot_z_sin_lu) * cos_h

    return j    

print(gen_jacobian_function())

quadruped = Base(anymal_c)

joint_states = np.zeros((12,1))
hip_trans = quadruped.hips.translation
upper_leg_trans =quadruped.upper_legs.translation.xyz
lower_leg_trans = quadruped.lower_legs.translation.xyz
feet_trans = quadruped.feet.translation.xyz

start = time.time()
calc_jacobian = get_jacobian(
    joint_states.reshape(4,3),
    hip_trans,
    upper_leg_trans,
    lower_leg_trans,
    feet_trans
)

end = time.time()-start
print(end  *  1000)
print(calc_jacobian)