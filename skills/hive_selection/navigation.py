from raya.controllers.navigation_controller import POSITION_UNIT, ANGLE_UNIT

NAV_POINT_HIVE = {
        'x' :       -2.000,
        'y' :       -3.247,
        'angle' :    40.0,
        'pos_unit': POSITION_UNIT.METERS, 
        'ang_unit': ANGLE_UNIT.DEGREES}

NAV_POINT_HOME = {
        'x':        -2.201,
        'y':        -3.973,
        'angle':     130.0, 
        'pos_unit': POSITION_UNIT.METERS, 
        'ang_unit': ANGLE_UNIT.DEGREES}

# Threshold away from the point of navigation to be considered okay (meters)
NAVIGATION_THRESHOLD = 1.0