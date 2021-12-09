from constants import NUM_BLOCKS, team_identifier, a, b, M
import time
from typing import cast


def verification(**kwargs):
    """CQC Comms Verification
    For more info see https://cornell.app.box.com/file/766365097328
    Assuming a data rate of 31 bits/second, 30 minutes of data transmission gives 48 data blocks"""
    num_blocks: int = cast('int', kwargs.get(NUM_BLOCKS))

    data_block_sequence_num = 0
    team_bytes = team_identifier.to_bytes(4, 'big')
    data_transmission_sequence = bytes()

    for _ in range(num_blocks):
        # header calculation:
        sequence_bytes = data_block_sequence_num.to_bytes(4, 'big')
        # get current time
        timestamp = time.time()  # each block has its own timestamp
        # extract seconds and milliseconds from timestamp:
        seconds_int = int(timestamp)
        seconds_bytes = seconds_int.to_bytes(4, 'big')
        ms_bytes = int((timestamp - seconds_int) *
                       (10 ** 6)).to_bytes(4, 'big')

        # concatenate header
        header = team_bytes + sequence_bytes + seconds_bytes + ms_bytes

        # team identifier xor with timestamp seconds
        operating_period_base_seed = team_identifier ^ seconds_int
        # xor previous with data block sequence num
        block_seed = operating_period_base_seed ^ data_block_sequence_num

        prn_length = 128 // 4  # integer division
        # preallocate memory for storing prn data
        prn = [int()] * (prn_length + 1)
        prn[0] = block_seed  # x0 is the block seed

        for i in range(1, prn_length + 1):
            # algorithm defined in sec 4.4.2 of CommsProc rev 4
            xn = (a * prn[i - 1] + b) % M
            # if the mod operator above causes issues, anding with 32-bit 2**32 should do the trick
            prn[i] = xn

        # get rid of the first value in the PRN, x0 is not included in PRN
        prn.pop(0)

        data_field = bytes()
        for j in prn:
            # concatenate prn data into bytes
            data_field += j.to_bytes(4, 'big')

        data_block = header + data_field

        # concatenate data block into transmission sequence
        data_transmission_sequence += data_block
        data_block_sequence_num += 1

    # TODO instead of returning, add to comms queue
    return data_transmission_sequence.hex()
