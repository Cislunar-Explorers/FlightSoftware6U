import threading

global_lock = threading.Lock()
"""Lock for op-nav camera"""

want_to_run_camera = False
"""Whether op-nav wants to run the camera"""

can_run_camera = False
"""Whether the camera can be run exclusively"""

run_camera_cond = threading.Condition()
"""Condition to run op-nav camera"""

# can_run_adjust = False
# """Whether att-adjust can be run exclusively"""

# can_run_adjust_cond = threading.Condition()
# """Condition to run main-sat att-adjust mode"""
