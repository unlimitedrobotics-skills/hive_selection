from raya.controllers.navigation_controller import POSITION_UNIT, ANGLE_UNIT

NAV_POINT_CART = {
        'x':        -2.30,
        'y':        -3.70,
        'pos_unit': POSITION_UNIT.METERS, 
        'ang_unit': ANGLE_UNIT.DEGREES}

# Threshold away from the point of navigation to be considered okay (meters)
NAVIGATION_THRESHOLD = 1.0