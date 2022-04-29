from math import radians

from motor import MotorConfig
from transmission import TransmissionConfig

M_BASE = MotorConfig(
    kv = 200.0,
    max_current = 5.0,
    position_gain = 200,
    velocity_gain = 0.00023,
    can_id = 1,
    direction = 1.0,
    name = "base"
)

M_EPAULE = MotorConfig(
    kv = 200.0,
    max_current = 5.0,
    position_gain = 200,
    velocity_gain = 0.00023,
    can_id = 2,
    direction = 1.0,
    name = "epaule"
)

M_COUDE = MotorConfig(
    kv = 200.0,
    max_current = 5.0,
    position_gain = 200,
    velocity_gain = 0.00023,
    can_id = 3,
    direction = 1.0,
    name = "coude"
)

M_POIGNET_1 = MotorConfig(
    kv = 380.0,
    max_current = 5.0,
    position_gain = 200,
    velocity_gain = 0.00023,
    can_id = 4,
    direction = 1.0,
    name = "poignet 1"
)

M_POIGNET_2 = MotorConfig(
    kv = 380.0,
    max_current = 5.0,
    position_gain = 200,
    velocity_gain = 0.00023,
    can_id = 1,
    direction = 1.0,
    name = "poignet 2"
)

M_POIGNET_3 = MotorConfig(
    kv = 380.0,
    max_current = 5.0,
    position_gain = 200,
    velocity_gain = 0.00023,
    can_id = 1,
    direction = 1.0,
    name = "poignet 3"
)

T_BASE = TransmissionConfig(
    ratio = 40.0,
    zero = 0.0, # read it init for now.
    low = radians(-170.0),
    high = radians(170.0)
)

T_EPAULE = TransmissionConfig(
    ratio = 40.0,
    zero = 0.0, # read it init for now.
    low = radians(-90.0),
    high = radians(90.0)
)

T_COUDE = TransmissionConfig(
    ratio = 40.0,
    zero = 0.0, # read it init for now.
    low = radians(-70.0),
    high = radians(200.0)
)

T_POIGNET_1 = TransmissionConfig(
    ratio = 40.0,
    zero = 0.0, # read it init for now.
    low = radians(-120.0),
    high = radians(120.0)
)

T_POIGNET_2 = TransmissionConfig(
    ratio = 40.0,
    zero = 0.0, # read it init for now.
    low = radians(-120.0),
    high = radians(120.0)
)

T_POIGNET_3 = TransmissionConfig(
    ratio = 40.0,
    zero = 0.0, # read it init for now.
    low = radians(-1800.0),
    high = radians(1800.0)
)