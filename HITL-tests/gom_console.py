from drivers.gom import Gomspace
import drivers.power.power_structs as ps
from time import sleep
from utils.log import get_log

logger = get_log()

gom = Gomspace()

while True:
    choice = int(input("Choose an option:\n"
                       "1: Receiving Amplifier (LNA)\n"
                       "2: Glowplug 1\n"
                       "3: Glowplug 2\n"
                       "4: Burnwire\n"
                       "5: Solenoid\n"
                       "6: Electrolyzer\n"
                       "7: Transmitting Amplifier (PA)\n"
                       "999: All Off\n"))
    if choice == 999:
        gom.all_off()
        ps.displayHk2(gom.get_health_data(level="eps"))

    if choice in [1, 5, 6, 7]:
        duration = int(input("Duration (seconds):\n"))

    if choice in [2, 3, 4]:
        duration = int(input("Duration (milliseconds):\n"))

    if choice == 1:
        gom.lna(True)
        sleep(duration / 2)
        ps.displayHk2(gom.get_health_data(level='eps'))
        sleep(duration / 2)
        gom.lna(False)

    if choice == 2:
        gom.pc.set_single_output("glowplug_2", 1, 0)
        ps.displayHk2(gom.get_health_data(level="eps"))
        sleep(1e-3 * duration)
        gom.pc.set_single_output("glowplug_2", 0, 0)

    if choice == 3:
        gom.pc.set_single_output("glowplug", 1, 0)
        ps.displayHk2(gom.get_health_data(level="eps"))
        sleep(1e-3 * duration)
        gom.pc.set_single_output("glowplug", 0, 0)

    if choice == 4:
        gom.burnwire1(duration)

    if choice == 5:
        assert 0 < duration < 300
        logger.info("Pulsing Solenoid")
        gom.solenoid(1e-3 * duration)

    if choice == 6:
        gom.set_electrolysis(True)
        sleep(duration / 2)
        ps.displayHk2(gom.get_health_data(level="eps"))
        sleep(duration / 2)
        gom.set_electrolysis(False)

    if choice == 7:
        gom.set_PA(True)
        sleep(duration / 2)
        ps.displayHk2(gom.get_health_data(level="eps"))
        sleep(duration / 2)
        gom.set_PA(False)
