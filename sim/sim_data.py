class SimData:
    """Container for all data sent back to the sim after one time step."""

    def __init__(self):
        self.data = {}

    def write_single_entry(self, label, value):
        """Write one (label, value) entry to the sim output."""
        self.data[label] = value

    def write_multi_entries(self, new_entries):
        """
    Write some number of entries to the sim output.

    Args:
        new_entries (Dict): Entries to be written to the sim output.
    """
        self.data.update(new_entries)

    def to_dict(self):
        """Output to the sim as a dictionary."""
        return self.data
