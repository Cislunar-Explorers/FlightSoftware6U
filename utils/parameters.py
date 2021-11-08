# Define all our parameters
DOWNLINK_BUFFER_TIME = 3
TELEM_INTERVAL = 60  # minutes
BOOTUP_SEPARATION_DELAY = 30.0  # seconds
ENTER_LOW_BATTERY_MODE_THRESHOLD = 7200  # mV
EXIT_LOW_BATTERY_MODE_THRESHOLD = 7500  # mV
# TODO Change to +50
ENTER_ECLIPSE_MODE_CURRENT = -50  # mA
ENTER_ECLIPSE_MODE_THRESHOLD = 7500  # mV
LOW_BATT_MODE_SLEEP = 10  # seconds
IGNORE_LOW_BATTERY = False

LOW_CRACKING_PRESSURE = 10.0  # psi? TODO(tmf97) figure out units
HIGH_CRACKING_PRESSURE = 20.0
IDEAL_CRACKING_PRESSURE = 15.0

GOM_VOLTAGE_MAX = 8400  # millivolts
GOM_VOLTAGE_MIN = 6000  # millivolts
OPNAV_INTERVAL = 60  # minutes

LB_OPNAV_INTERVAL = 120  # minutes
LB_TLM_INTERVAL = 100  # minutes

LAST_OPNAV_RUN = 0.0
LAST_TELEM_DOWNLINK = 0.0

SPLIT_BURNWIRE_DURATION = 1  # seconds
ANTENNAE_BURNWIRE_DURATION = 1  # seconds
GLOWPLUG_DURATION = 1  # seconds

SCHEDULED_BURN_TIME: float = -1.0

ACS_SPIKE_DURATION = 15  # milliseconds
WANT_TO_ELECTROLYZE = False
WANT_TO_OPNAV = False

DEFAULT_ELECTROLYSIS_DELAY = 1  # seconds

FILE_UPDATE_PATH = ''
GYRO_BIAS_X = 0.0  # rad/s
GYRO_BIAS_Y = 0.0  # rad/s
GYRO_BIAS_Z = 0.0  # rad/s

# TODO determine how gyro biases change at different temperatures
GYRO_BIAS_TEMPERATURE = 0  # deg C
GYRO_BIAS_DXDT = 0.0  # rad/s per deg C
GYRO_BIAS_DYDT = 0.0  # rad/s per deg C
GYRO_BIAS_DZDT = 0.0  # rad/s per deg C

# Counter storage for uplink and downlink
UPLINK_COUNTER: int = 0
DOWNLINK_COUNTER: int = 0
