from drivers.gom import Gomspace
import drivers.power.power_structs as ps
from time import sleep
import logging

from utils.constants import GomOutputs


gom = Gomspace()

try:
    while True:
        choice = int(
            input(
                "Choose an option:\n"
                "0: Display Housekeeping Data\n"
                "1: OUT-1 Receiving Amplifier (LNA) - max 30s \n"
                "2: OUT-3 Glowplug 1 - max 15000ms\n"
                "3: OUT-4 Glowplug 2 - max 15000ms\n"
                "4: OUT-2 Burnwire - max 30s\n"
                "5: OUT-5 Solenoid - max 400ms\n"
                "6: OUT-6 Electrolyzer - max 30s\n"
                "7: RELAY Transmitting Amplifier (PA) - max 30s\n"
                "8: OUT-5 with solenoid disconnected - max 30s\n"
                "999: Turn OFF all controllable outputs\n"
            )
        )
        if choice == 999:
            gom.all_off()
            ps.displayHk2(gom.get_health_data(level="eps"))

        if choice == 0:
            ps.displayHk2(gom.get_health_data(level="eps"))

        if choice in [1, 4, 6, 7, 8]:
            duration = int(input("Duration (integer seconds):\n"))
            assert 0 < duration <= 30
            if choice == 1:
                gom.lna(True)
                sleep(duration / 2)
                ps.displayHk2(gom.get_health_data(level="eps"))
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
                gom.set_pa(True)
                sleep(duration / 2)
                ps.displayHk2(gom.get_health_data(level="eps"))
                sleep(duration / 2)
                gom.set_pa(False)

            if choice == 8:
                gom.pc.set_single_output(GomOutputs.solenoid, 1, 0)
                sleep(duration / 2)
                ps.displayHk2(gom.get_health_data(level="eps"))
                sleep(duration / 2)
                gom.pc.set_single_output(GomOutputs.solenoid, 0, 0)

        if choice in [2, 3, 5]:
            duration = int(input("Duration (integer milliseconds):\n"))
            if choice == 2:
                gom.pc.set_single_output(GomOutputs.glowplug_2, 1, 0)
                ps.displayHk2(gom.get_health_data(level="eps"))
                sleep(1e-3 * duration)
                gom.pc.set_single_output(GomOutputs.glowplug_2, 0, 0)

            if choice == 3:
                gom.pc.set_single_output(GomOutputs.glowplug, 1, 0)
                ps.displayHk2(gom.get_health_data(level="eps"))
                sleep(1e-3 * duration)
                gom.pc.set_single_output(GomOutputs.glowplug, 0, 0)

            if choice == 5:
                assert 0 < duration < 400
                logging.info("Pulsing Solenoid")
                gom.solenoid(1e-3 * duration)

finally:
    gom.all_off()
