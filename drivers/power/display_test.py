from power_structs import *

dtest = eps_hk_t()

# All of these values are arbitrary
dtest.vboost = (3700, 3700, 3700)
dtest.vbatt = 3000
dtest.curin = (100, 300, 431)
dtest.cursun = 3000
dtest.cursys = 123
dtest.curout = (1, 2, 3, 4, 5, 6)
dtest.output = (1, 1, 0, 1, 1, 0, 0, 0)
dtest.output_on_delta = (0, 0, 0, 0, 0, 0, 0, 0)
dtest.output_off_delta = (0, 0, 0, 0, 0, 0, 0, 0)
dtest.latchup = (13, 0, 0, 0, 0, 0)
dtest.wdt_i2c_time_left = 12345
dtest.wdt_gnd_time_left = 23456
dtest.wdt_csp_pings_left = (98172, 100)
dtest.counter_wdt_gnd = 0
dtest.counter_boot = 12
dtest.temp = (50, 13, 16, 108, -12, 23)
dtest.bootcause = 1
dtest.battmode = 1
dtest.pptmode = 2
dtest.reserved1 = 2
dtest.reserved2 = 2

displayHk2(dtest)
displayStruct(dtest)
