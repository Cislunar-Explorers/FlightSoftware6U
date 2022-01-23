# If you are comfortable with python and the Gomspace P31u,
# I would recommend using an ipython session to control the gom.
# Be careful though when hardware is connected - the ipython notebook
# does NOT have the same precautionary measures in place that are defined in this file.

from drivers.gom import Gomspace
import drivers.power.power_structs as ps
from time import sleep
import logging


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
            ps.displayHk2(gom.collect_telem())

        if choice == 0:
            ps.displayHk2(gom.collect_telem())

        if choice in [1, 4, 6, 7, 8]:
            duration = int(input("Duration (integer seconds):\n"))
            assert 0 < duration <= 30
            if choice == 1:
                gom.lna.set(True)
                sleep(duration / 2)
                ps.displayHk2(gom.collect_telem())
                sleep(duration / 2)
                gom.lna.set(False)

            if choice == 4:
                gom.burnwire.pulse(duration)

            if choice == 6:
                gom.electrolyzers.set(True)
                sleep(duration / 2)
                ps.displayHk2(gom.collect_telem())
                sleep(duration / 2)
                gom.electrolyzers.set(False)

            if choice == 7:
                gom.pa.set(True)
                sleep(duration / 2)
                ps.displayHk2(gom.collect_telem())
                sleep(duration / 2)
                gom.pa.set(False)

            if choice == 8:
                gom.solenoid.set(True)
                sleep(duration / 2)
                ps.displayHk2(gom.collect_telem())
                sleep(duration / 2)
                gom.solenoid.set(False)

        if choice in [2, 3, 5]:
            duration = int(input("Duration (integer milliseconds):\n"))
            if choice == 2:
                gom.glowplug_2.set(True)
                ps.displayHk2(gom.collect_telem())
                sleep(1e-3 * duration)
                gom.glowplug_2.set(False)

            if choice == 3:
                gom.glowplug_1.set(True)
                ps.displayHk2(gom.collect_telem())
                sleep(1e-3 * duration)
                gom.glowplug_1.set(False)

            if choice == 5:
                logging.info("Pulsing Solenoid")
                gom.solenoid.pulse()

finally:
    gom.all_off()
