from power_controller import *

HITL_test = Power()

print("Printing Current Config:\n")
HITL_test.displayAll()

confirm = input("Do you wish to proceed? (y/n)")

if confirm == "y":
    new_config = eps_config_t()
    new_config.ppt_mode = 2
    new_config.battheater_mode = 0
    new_config.battheater_low = 0
    new_config.battheater_high = 5
    new_config.output_normal_value = (0, 0, 0, 0, 0, 0, 0, 0)
    new_config.output_safe_value = (0, 0, 0, 0, 0, 0, 0, 0)
    new_config.output_initial_on_delay = (0, 0, 0, 0, 0, 0, 0, 0)
    new_config.output_initial_off_delay = (0, 0, 0, 0, 0, 0, 0, 0)
    new_config.vboost = (3700, 3700, 3700)

    displayConfig(new_config)
    set_confirm = input("Do you want to set the above config to the P31u? (y/n)")

    if set_confirm == "y":
        HITL_test.config_set(new_config)

        current_config = HITL_test.config_get()
        displayConfig(current_config)

        reboot_confirm = input("Ready to Reboot? (y/n)")

        if reboot_confirm == "y":
            print("You will lose SSH connection momentarily...")
            HITL_test.reboot()
