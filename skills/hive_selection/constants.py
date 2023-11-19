# Errors
ERROR_COULDNT_REACH_DESTINATION = (1, "Couldn't reach navigation destination")
ERROR_COULDNT_APPROACH_CART = (2, "Couldn't approach the cart")
ERROR_TAG_NOT_FOUND = (3, "Couldn't find the tag")
ERROR_COULDNT_POSITION_ARM = (4, "Couldn't position the arm")
ERROR_ARM_POSITION_NOT_ACCURATE = (5, 'Arm position not accurate')
ERROR_COULDNT_PICKUP_ITEM = (6, "Couldn't pick up the item")

# Max attempts
MAX_NAVIGATION_ATTEMPTS = 3
MAX_APPROACH_ATTEMPTS = 2
MAX_POSITION_ATTEMPTS = 3
MAX_PICKUP_ATTEMPTS = 2

# Timeouts
NO_TARGET_TIMEOUT = 10.0

# Other constants
HIVE_NUM_ROWS = 2
HIVE_NUM_COLS = 2
MAX_CAMERA_PIXELS_X = 850

CELL_SIZE_X = 0.08
CELL_SIZE_Z = 0.065
PICKUP_HEIGHT = 0.275

NORMAL_FOOTPRINT = [
        [-0.25,  0.32],
        [ 0.25,  0.32],
        [ 0.25, -0.32],
        [-0.25, -0.32]
    ]

#TODO: CHANGE TO ACTUAL PICKUP FOOTPRINT
PICKUP_FOOTPRINT = [
    [-0.35,  0.70],
    [ 0.35,  0.70],
    [ 0.25, -0.32],
    [-0.25, -0.32]
]