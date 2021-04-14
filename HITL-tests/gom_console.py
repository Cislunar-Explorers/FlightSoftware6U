from drivers.gom import Gomspace
import drivers.power.power_structs as ps
from time import sleep
from utils.log import get_log

logger = get_log()

gom = Gomspace()

while True:
    choice = int(input("Choose an option:\n"
                       "0: Display Housekeeping Data"
                       "1: Receiving Amplifier (LNA)\n"
                       "2: Glowplug 1\n"
                       "3: Glowplug 2\n"
                       "4: Burnwire\n"
                       "5: Solenoid\n"
                       "6: Electrolyzer\n"
                       "7: Transmitting Amplifier (PA)\n"
                       "999: Turn OFF all controllable outputs\n"))
    if choice == 999:
        gom.all_off()
        ps.displayHk2(gom.get_health_data(level="eps"))

    if choice == 0:
        ps.displayHk2(gom.get_health_data(level='eps'))

    if choice in [1, 4, 6, 7]:
        duration = int(input("Duration (integer seconds):\n"))
        assert 0 < duration < 15
        if choice == 1:
            gom.lna(True)
            sleep(duration / 2)
            ps.displayHk2(gom.get_health_data(level='eps'))
            sleep(duration / 2)
            gom.lna(False)

        if choice == 4:
            gom.burnwire1(duration)

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

    if choice in [2, 3, 5]:
        duration = int(input("Duration (integer milliseconds):\n"))
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

        if choice == 5:
            assert 0 < duration < 300
            logger.info("Pulsing Solenoid")
            gom.solenoid(1e-3 * duration)
