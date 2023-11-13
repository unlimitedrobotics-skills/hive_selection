GRIPPER_COMMANDS = {

    'close': {
        'arm': 'both',
        'desired_position': 1.0,
        'desired_pressure': 0.8,
    },

    'open': {
        'arm': 'both',
        'desired_position': 0.0,
        'desired_pressure': 0.8,
    }
}

JOINT_NAMES = [
                "arm_right_shoulder_rail_joint",
                "arm_right_shoulder_FR_joint",
                "arm_right_shoulder_RL_joint",
                "arm_right_bicep_twist_joint",
                "arm_right_bicep_FR_joint",
                "arm_right_elbow_twist_joint",
                "arm_right_elbow_FR_joint",
                "arm_right_wrist_joint"
            ]

TREX_POSITION_ANGLES =  [0.0, 0.0, 0.0, 0.0, 1.8, 0.0, -0.25, -1.57]

# Gary 13 right arm
RIGHT_ARM_OFFSET = {'x' : -0.04,
                    'y' : 0.0,
                    'z' : 0.0065}

ARM_ERROR_THRESHOLD = [0.03, 0.03, 0.03]


