import os
from dataclasses import dataclass
from champ.types import GaitConfig

@dataclass
class RobotProfile:
    urdf = ''
    link_names = []


class DKitty:
    urdf = os.path.dirname(os.path.abspath(__file__)) + '/dkitty/dkitty.urdf'
    link_names = ['lf_hip_adapter_link', 'lf_upper_leg_adapter_link', 'lf_lower_leg_adapter_link', 'lf_foot_link', 'rf_hip_adapter_link', 'rf_upper_leg_adapter_link', 'rf_lower_leg_adapter_link', 'rf_foot_link', 'lh_hip_adapter_link', 'lh_upper_leg_adapter_link', 'lh_lower_leg_adapter_link', 'lh_foot_link', 'rh_hip_adapter_link', 'rh_upper_leg_adapter_link', 'rh_lower_leg_adapter_link', 'rh_foot_link']
    joint_names = ['lf_hip_adapter_joint', 'lf_upper_leg_adapter_joint', 'lf_lower_leg_adapter_joint', 'lf_foot_joint', 'rf_hip_adapter_joint', 'rf_upper_leg_adapter_joint', 'rf_lower_leg_adapter_joint', 'rf_foot_joint', 'lh_hip_adapter_joint', 'lh_upper_leg_adapter_joint', 'lh_lower_leg_adapter_joint', 'lh_foot_joint', 'rh_hip_adapter_joint', 'rh_upper_leg_adapter_joint', 'rh_lower_leg_adapter_joint', 'rh_foot_joint']
    gait_config = GaitConfig()
    gait_config.knee_orientation = ">>"
    gait_config.pantograph_leg = False
    gait_config.odom_scaler = 1.0
    gait_config.max_linear_velocity_x = 0.5
    gait_config.max_linear_velocity_y = 0.25
    gait_config.max_angular_velocity_z = 1.0
    gait_config.com_x_translation = 0.0
    gait_config.swing_height = 0.04
    gait_config.stance_depth = 0.0
    gait_config.stance_duration = 0.2
    gait_config.nominal_height = 0.23


class SpotMicro:
    urdf = os.path.dirname(os.path.abspath(__file__)) + '/spotmicro/spotmicro.urdf'
    link_names = ['front_left_shoulder_link', 'front_left_leg_link', 'front_left_foot_link', 'front_left_toe_link', 'front_right_shoulder_link', 'front_right_leg_link', 'front_right_foot_link', 'front_right_toe_link', 'rear_left_shoulder_link', 'rear_left_leg_link', 'rear_left_foot_link', 'rear_left_toe_link', 'rear_right_shoulder_link', 'rear_right_leg_link', 'rear_right_foot_link', 'rear_right_toe_link']
    joint_names = ['front_left_shoulder', 'front_left_leg', 'front_left_foot', 'front_left_toe', 'front_right_shoulder', 'front_right_leg', 'front_right_foot', 'front_right_toe', 'rear_left_shoulder', 'rear_left_leg', 'rear_left_foot', 'rear_left_toe', 'rear_right_shoulder', 'rear_right_leg', 'rear_right_foot', 'rear_right_toe']
    gait_config = GaitConfig()
    gait_config.knee_orientation = ">>"
    gait_config.pantograph_leg = False
    gait_config.odom_scaler = 1.0
    gait_config.max_linear_velocity_x = 0.5
    gait_config.max_linear_velocity_y = 0.25
    gait_config.max_angular_velocity_z = 1.0
    gait_config.com_x_translation = 0.0
    gait_config.swing_height = 0.02
    gait_config.stance_depth = 0.0
    gait_config.stance_duration = 0.25
    gait_config.nominal_height = 0.15


class OpenDog:
    urdf = os.path.dirname(os.path.abspath(__file__)) + '/opendog/opendog.urdf'
    link_names = ['lf_hip_link', 'lf_upper_leg_link', 'lf_lower_leg_link', 'lf_foot_link', 'rf_hip_link', 'rf_upper_leg_link', 'rf_lower_leg_link', 'rf_foot_link', 'lh_hip_link', 'lh_upper_leg_link', 'lh_lower_leg_link', 'lh_foot_link', 'rh_hip_link', 'rh_upper_leg_link', 'rh_lower_leg_link', 'rh_foot_link']
    joint_names = ['lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint', 'lf_foot_joint', 'rf_hip_joint', 'rf_upper_leg_joint', 'rf_lower_leg_joint', 'rf_foot_joint', 'lh_hip_joint', 'lh_upper_leg_joint', 'lh_lower_leg_joint', 'lh_foot_joint', 'rh_hip_joint', 'rh_upper_leg_joint', 'rh_lower_leg_joint', 'rh_foot_joint']
    gait_config = GaitConfig()
    gait_config.knee_orientation = ">>"
    gait_config.pantograph_leg = False
    gait_config.odom_scaler = 2
    gait_config.max_linear_velocity_x = 0.3
    gait_config.max_linear_velocity_y = 0.2
    gait_config.max_angular_velocity_z = 1.0
    gait_config.com_x_translation = -0.01
    gait_config.swing_height = 0.04
    gait_config.stance_depth = 0.0
    gait_config.stance_duration = 0.25
    gait_config.nominal_height = 0.3


class AnymalB:
    urdf = os.path.dirname(os.path.abspath(__file__)) + '/anymal_b/anymal_b.urdf'
    link_names = ['LF_HIP', 'LF_THIGH', 'LF_SHANK', 'LF_FOOT', 'RF_HIP', 'RF_THIGH', 'RF_SHANK', 'RF_FOOT', 'LH_HIP', 'LH_THIGH', 'LH_SHANK', 'LH_FOOT', 'RH_HIP', 'RH_THIGH', 'RH_SHANK', 'RH_FOOT']
    joint_names = ['LF_HAA', 'LF_HFE', 'LF_KFE', 'LF_ADAPTER_TO_FOOT', 'RF_HAA', 'RF_HFE', 'RF_KFE', 'RF_ADAPTER_TO_FOOT', 'LH_HAA', 'LH_HFE', 'LH_KFE', 'LH_ADAPTER_TO_FOOT', 'RH_HAA', 'RH_HFE', 'RH_KFE', 'RH_ADAPTER_TO_FOOT']
    gait_config = GaitConfig()
    gait_config.knee_orientation = "><"
    gait_config.pantograph_leg = False
    gait_config.odom_scaler = 1.0
    gait_config.max_linear_velocity_x = 0.5
    gait_config.max_linear_velocity_y = 0.25
    gait_config.max_angular_velocity_z = 1.0
    gait_config.com_x_translation = 0.0
    gait_config.swing_height = 0.05
    gait_config.stance_depth = 0.0
    gait_config.stance_duration = 0.3
    gait_config.nominal_height = 0.43


class OpenQuadruped:
    urdf = os.path.dirname(os.path.abspath(__file__)) + '/open_quadruped/open_quadruped.urdf'
    link_names = ['front_left_hip', 'front_left_upper_leg', 'front_left_lower_leg', 'front_left_foot', 'front_right_hip', 'front_right_upper_leg', 'front_right_lower_leg', 'front_right_foot', 'back_left_hip', 'back_left_upper_leg', 'back_left_lower_leg', 'back_left_foot', 'back_right_hip', 'back_right_upper_leg', 'back_right_lower_leg', 'back_right_foot']
    joint_names = ['motor_front_left_hip', 'motor_front_left_upper_leg', 'motor_front_left_lower_leg', 'front_left_leg_foot', 'motor_front_right_hip', 'motor_front_right_upper_leg', 'motor_front_right_lower_leg', 'front_right_leg_foot', 'motor_back_left_hip', 'motor_back_left_upper_leg', 'motor_back_left_lower_leg', 'back_left_leg_foot', 'motor_back_right_hip', 'motor_back_right_upper_leg', 'motor_back_right_lower_leg', 'back_right_leg_foot']
    gait_config = GaitConfig()
    gait_config.knee_orientation = ">>"
    gait_config.pantograph_leg = False
    gait_config.odom_scaler = 1.15
    gait_config.max_linear_velocity_x = 0.3
    gait_config.max_linear_velocity_y = 0.15
    gait_config.max_angular_velocity_z = 1.0
    gait_config.com_x_translation = -0.03
    gait_config.swing_height = 0.02
    gait_config.stance_depth = 0.0
    gait_config.stance_duration = 0.25
    gait_config.nominal_height = 0.15


class MiniCheetah:
    urdf = os.path.dirname(os.path.abspath(__file__)) + '/mini_cheetah/mini_cheetah.urdf'
    link_names = ['FL_hip', 'FL_thigh', 'FL_calf', 'FL_foot', 'FR_hip', 'FR_thigh', 'FR_calf', 'FR_foot', 'RL_hip', 'RL_thigh', 'RL_calf', 'RL_foot', 'RR_hip', 'RR_thigh', 'RR_calf', 'RR_foot']
    joint_names = ['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint', 'FL_foot_fixed', 'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint', 'FR_foot_fixed', 'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint', 'RL_foot_fixed', 'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint', 'RR_foot_fixed']
    gait_config = GaitConfig()
    gait_config.knee_orientation = ">>"
    gait_config.pantograph_leg = False
    gait_config.odom_scaler = 0.9
    gait_config.max_linear_velocity_x = 0.5
    gait_config.max_linear_velocity_y = 0.25
    gait_config.max_angular_velocity_z = 1.0
    gait_config.com_x_translation = 0.0
    gait_config.swing_height = 0.05
    gait_config.stance_depth = 0.0
    gait_config.stance_duration = 0.3
    gait_config.nominal_height = 0.3


class AlienGo:
    urdf = os.path.dirname(os.path.abspath(__file__)) + '/aliengo/aliengo.urdf'
    link_names = ['FL_hip', 'FL_thigh', 'FL_calf', 'FL_foot', 'FR_hip', 'FR_thigh', 'FR_calf', 'FR_foot', 'RL_hip', 'RL_thigh', 'RL_calf', 'RL_foot', 'RR_hip', 'RR_thigh', 'RR_calf', 'RR_foot']
    joint_names = ['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint', 'FL_foot_fixed', 'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint', 'FR_foot_fixed', 'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint', 'RL_foot_fixed', 'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint', 'RR_foot_fixed']
    gait_config = GaitConfig()
    gait_config.knee_orientation = ">>"
    gait_config.pantograph_leg = False
    gait_config.odom_scaler = 0.9
    gait_config.max_linear_velocity_x = 1.5
    gait_config.max_linear_velocity_y = 0.25
    gait_config.max_angular_velocity_z = 1.0
    gait_config.com_x_translation = -0.05
    gait_config.swing_height = 0.06
    gait_config.stance_depth = 0.0
    gait_config.stance_duration = 0.4
    gait_config.nominal_height = 0.35


class AnymalC:
    urdf = os.path.dirname(os.path.abspath(__file__)) + '/anymal_c/anymal_c.urdf'
    link_names = ['LF_HIP', 'LF_THIGH', 'LF_SHANK', 'LF_FOOT', 'RF_HIP', 'RF_THIGH', 'RF_SHANK', 'RF_FOOT', 'LH_HIP', 'LH_THIGH', 'LH_SHANK', 'LH_FOOT', 'RH_HIP', 'RH_THIGH', 'RH_SHANK', 'RH_FOOT']
    joint_names = ['LF_HAA', 'LF_HFE', 'LF_KFE', 'LF_shank_fixed_LF_FOOT', 'RF_HAA', 'RF_HFE', 'RF_KFE', 'RF_shank_fixed_RF_FOOT', 'LH_HAA', 'LH_HFE', 'LH_KFE', 'LH_shank_fixed_LH_FOOT', 'RH_HAA', 'RH_HFE', 'RH_KFE', 'RH_shank_fixed_RH_FOOT']
    gait_config = GaitConfig()
    gait_config.knee_orientation = "><"
    gait_config.pantograph_leg = False
    gait_config.odom_scaler = 1.0
    gait_config.max_linear_velocity_x = 0.5
    gait_config.max_linear_velocity_y = 0.25
    gait_config.max_angular_velocity_z = 1.0
    gait_config.com_x_translation = 0.0
    gait_config.swing_height = 0.06
    gait_config.stance_depth = 0.0
    gait_config.stance_duration = 0.3
    gait_config.nominal_height = 0.43


class Spot:
    urdf = os.path.dirname(os.path.abspath(__file__)) + '/spot/spot.urdf'
    link_names = ['front_left_hip', 'front_left_upper_leg', 'front_left_lower_leg', 'front_left_ee', 'front_right_hip', 'front_right_upper_leg', 'front_right_lower_leg', 'front_right_ee', 'rear_left_hip', 'rear_left_upper_leg', 'rear_left_lower_leg', 'rear_left_ee', 'rear_right_hip', 'rear_right_upper_leg', 'rear_right_lower_leg', 'rear_right_ee']
    joint_names = ['front_left_hip_x', 'front_left_hip_y', 'front_left_knee', 'front_left_foot', 'front_right_hip_x', 'front_right_hip_y', 'front_right_knee', 'front_right_foot', 'rear_left_hip_x', 'rear_left_hip_y', 'rear_left_knee', 'rear_left_foot', 'rear_right_hip_x', 'rear_right_hip_y', 'rear_right_knee', 'rear_right_foot']
    gait_config = GaitConfig()
    gait_config.knee_orientation = ">>"
    gait_config.pantograph_leg = False
    gait_config.odom_scaler = 1.2
    gait_config.max_linear_velocity_x = 1.5
    gait_config.max_linear_velocity_y = 0.25
    gait_config.max_angular_velocity_z = 1.0
    gait_config.com_x_translation = -0.05
    gait_config.swing_height = 0.06
    gait_config.stance_depth = -0.01
    gait_config.stance_duration = 0.4
    gait_config.nominal_height = 0.48


class LittleDog:
    urdf = os.path.dirname(os.path.abspath(__file__)) + '/littledog/littledog.urdf'
    link_names = ['front_left_hip', 'front_left_upper_leg', 'front_left_lower_leg', 'front_left_foot', 'front_right_hip', 'front_right_upper_leg', 'front_right_lower_leg', 'front_right_foot', 'back_left_hip', 'back_left_upper_leg', 'back_left_lower_leg', 'back_left_foot', 'back_right_hip', 'back_right_upper_leg', 'back_right_lower_leg', 'back_right_foot']
    joint_names = ['front_left_hip_roll', 'front_left_hip_pitch', 'front_left_knee', 'front_left_ankle', 'front_right_hip_roll', 'front_right_hip_pitch', 'front_right_knee', 'front_right_ankle', 'back_left_hip_roll', 'back_left_hip_pitch', 'back_left_knee', 'back_left_ankle', 'back_right_hip_roll', 'back_right_hip_pitch', 'back_right_knee', 'back_right_ankle']
    gait_config = GaitConfig()
    gait_config.knee_orientation = "><"
    gait_config.pantograph_leg = False
    gait_config.odom_scaler = 1.0
    gait_config.max_linear_velocity_x = 0.5
    gait_config.max_linear_velocity_y = 0.25
    gait_config.max_angular_velocity_z = 1.0
    gait_config.com_x_translation = 0.0
    gait_config.swing_height = 0.02
    gait_config.stance_depth = 0.0
    gait_config.stance_duration = 0.2
    gait_config.nominal_height = 0.14


class StochLite:
    urdf = os.path.dirname(os.path.abspath(__file__)) + '/stochlite/stochlite.urdf'
    link_names = ['front_left_hip', 'front_left_upper_leg', 'front_left_lower_leg', 'front_left_foot', 'front_right_hip', 'front_right_upper_leg', 'front_right_lower_leg', 'front_right_foot', 'back_left_hip', 'back_left_upper_leg', 'back_left_lower_leg', 'back_left_foot', 'back_right_hip', 'back_right_upper_leg', 'back_right_lower_leg', 'back_right_foot']
    joint_names = ['front_left_hip_roll', 'front_left_hip_pitch', 'front_left_knee', 'front_left_ankle', 'front_right_hip_roll', 'front_right_hip_pitch', 'front_right_knee', 'front_right_ankle', 'back_left_hip_roll', 'back_left_hip_pitch', 'back_left_knee', 'back_left_ankle', 'back_right_hip_roll', 'back_right_hip_pitch', 'back_right_knee', 'back_right_ankle']
    gait_config = GaitConfig()
    gait_config.knee_orientation = "><"
    gait_config.pantograph_leg = False
    gait_config.odom_scaler = 1.0
    gait_config.max_linear_velocity_x = 0.5
    gait_config.max_linear_velocity_y = 0.25
    gait_config.max_angular_velocity_z = 1.0
    gait_config.com_x_translation = 0.0
    gait_config.swing_height = 0.02
    gait_config.stance_depth = 0.0
    gait_config.stance_duration = 0.2
    gait_config.nominal_height = 0.14


class StochLite:
    urdf = os.path.dirname(os.path.abspath(__file__)) + '/stochlite/stochlite.urdf'
    link_names = ['fl_abd_link', 'fl_thigh_link', 'fl_shank_link', 'fl_toe_link', 'fr_abd_link', 'fr_thigh_link', 'fr_shank_link', 'fr_toe_link', 'bl_abd_link', 'bl_thigh_link', 'bl_shank_link', 'bl_toe_link', 'br_abd_link', 'br_thigh_link', 'br_shank_link', 'br_toe_link']
    joint_names = ['fl_abd_joint', 'fl_hip_joint', 'fl_knee_joint', 'fl_toe_joint', 'fr_abd_joint', 'fr_hip_joint', 'fr_knee_joint', 'fr_toe_joint', 'bl_abd_joint', 'bl_hip_joint', 'bl_knee_joint', 'bl_toe_joint', 'br_abd_joint', 'br_hip_joint', 'br_knee_joint', 'br_toe_joint']
    gait_config = GaitConfig()
    gait_config.knee_orientation = "><"
    gait_config.pantograph_leg = False
    gait_config.odom_scaler = 1.0
    gait_config.max_linear_velocity_x = 0.4
    gait_config.max_linear_velocity_y = 0.25
    gait_config.max_angular_velocity_z = 1.0
    gait_config.com_x_translation = 0.0
    gait_config.swing_height = 0.04
    gait_config.stance_depth = 0.0
    gait_config.stance_duration = 0.25
    gait_config.nominal_height = 0.23
