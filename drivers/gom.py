class Gomspace:
    def __init__(self):
        self.electrolysis = False

    def tick_watchdog(self):
        pass

    def get_health_data(self):
        return {}

    def is_electrolyzing(self):
        return self.electrolysis

    def set_electrolysis(self, status: bool):
        self.electrolysis = status

    def read_battery_percentage(self):
        return 0.7
