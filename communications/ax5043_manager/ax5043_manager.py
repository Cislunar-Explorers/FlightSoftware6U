import logging
import queue
import time
from fsw.communications.ax5043_manager.ax5043_driver import (
    Reg,
    Pwrmode,
    Bits,
    Fifocmd,
    Chunk,
    DataChunk,
)


class Manager:
    def __init__(self, driver):
        self.driver = driver
        self.tx_enabled = False
        self.rx_enabled = False
        self.reset_requested = False
        self.inbox = queue.Queue()  # TODO: consider SimpleQueue (requires Python 3.7)
        self.outbox = queue.Queue()
        # TODO: metrics
        self.state = Manager.Initializing(self)

        # TODO: try-except?
        self.state.enter()

    def transition(self, new_state):
        logging.debug(
            "Transitioning from %s to %s",
            self.state.__class__.__name__,
            new_state.__class__.__name__,
        )
        try:
            self.state.exit()
            self.state = new_state
        except Exception as e:
            logging.error(
                "Exception exiting state %s: %s", self.state.__class__.__name__, e
            )
            self.state = Manager.Error(self, e)
        try:
            self.state.enter()
        except Exception as e:
            logging.error(
                "Exception entering state %s: %s", self.state.__class__.__name__, e
            )
            self.state = Manager.Error(self, e)
            # Any exceptions raised by Error.enter() will not be caught
            self.state.enter()
            return

    def dispatch(self):
        if self.reset_requested:
            logging.info("Resetting")
            self.transition(Manager.Initializing(self))
        else:
            try:
                self.state.dispatch()
            except Exception as e:
                logging.error(
                    "Exception dispatching state %s: %s",
                    self.state.__class__.__name__,
                    e,
                )

    def enable_pa(self, enabled):
        # TODO: enable power amp
        if enabled:
            logging.info("Enabling PA")
        else:
            logging.info("Disabling PA")

    def should_transmit(self):
        return self.tx_enabled and not self.inbox.empty()

    def is_faulted(self):
        return isinstance(self.state, Manager.Error)

    def poll(self, reg, mask, target=None, timeout=0.1):
        if target is None:
            target = mask
        start_time = time.monotonic()
        val = self.driver.read(reg)
        while (val & mask) != target:
            val = self.driver.read(reg)
            dt = time.monotonic() - start_time
            if dt > timeout:
                raise RuntimeError(
                    "Timeout (%s > %s) polling for register %02X, "
                    "mask %02X, to reach %02X (last value was %02X)"
                    % (dt, timeout, reg, mask, target, val)
                )
        return val

    def post(self):
        rev = self.driver.read(Reg.SILICONREVISION)
        if rev != 0x51:
            logging.error("Expected rev 0x51, but got %02X" % rev)
            return False
        s = self.driver.read(Reg.SCRATCH)
        if s != 0xC5:
            logging.error("Expected initial scratch to be 0xC5, but got %02X" % s)
            return False
        self.driver.execute({Reg.SCRATCH: 0x3A})
        s = self.driver.read(Reg.SCRATCH)
        if s != 0x3A:
            logging.error("Expected scratch to be set to 0x3A, but got %02X" % s)
            return False
        return True

    class State:
        def __init__(self, mgr):
            assert mgr is not None
            self.mgr = mgr

        def enter(self):
            pass

        def exit(self):
            pass

        def dispatch(self):
            pass

    class Initializing(State):
        def __init__(self, mgr):
            super().__init__(mgr)

        def enter(self):
            logging.info("Initializing")
            self.mgr.driver.reset()
            if not self.mgr.post():
                raise RuntimeError("POST failed")
            logging.info("POST passed")

        def dispatch(self):
            self.mgr.driver.execute(setup_cmds)
            self.mgr.driver.execute(datarate_cmds)
            self.mgr.transition(Manager.Autoranging(self.mgr))

    class Autoranging(State):
        MAX_POLL_COUNT = 10

        def __init__(self, mgr):
            super().__init__(mgr)
            self.started = False
            self.poll_count = 0

        def enter(self):
            logging.info("Autoranging")
            self.mgr.driver.set_pwrmode(Pwrmode.STANDBY)

            # Wait until crystal oscillator is ready
            # Note: This pattern is only appropriate if we expect that the XTAL
            # settling time could be either shorter or longer than a control
            # cycle and we don't want to wait to start the autoranging process
            # in the former case.
            try:
                self.mgr.poll(Reg.XTALSTATUS, Bits.XTAL_RUN)
            except Exception as e:
                logging.warning(e)
                return

            # Set RNG_START
            self.mgr.driver.execute({Reg.PLLRANGINGA: Bits.RNG_START | 0x08})
            self.started = True

        def dispatch(self):
            if not self.started:
                # Wait until crystal oscillator is ready
                try:
                    self.mgr.poll(Reg.XTALSTATUS, Bits.XTAL_RUN)
                except Exception as e:
                    logging.error(e)
                    self.mgr.transition(
                        Manager.Error(self.mgr, "Timeout waiting for XTAL_RUN")
                    )
                    return
                # Set RNG_START
                self.mgr.driver.execute({Reg.PLLRANGINGA: Bits.RNG_START | 0x08})
                self.started = True

            # Wait for auto-ranging to terminate
            pllranging = self.mgr.driver.read(Reg.PLLRANGINGA)
            if pllranging & Bits.RNGERR:
                self.mgr.transition(Manager.Error(self.mgr, "RNGERR"))
            elif not (pllranging & Bits.RNG_START):
                self.mgr.transition(Manager.Idle(self.mgr))
            else:
                self.poll_count += 1
                if self.poll_count > self.MAX_POLL_COUNT:
                    self.mgr.transition(Manager.Error(self.mgr, "RNG_START"))

    class Idle(State):
        def __init__(self, mgr):
            super().__init__(mgr)

        def enter(self):
            self.mgr.driver.set_pwrmode(Pwrmode.POWERDOWN)

        def dispatch(self):
            if self.mgr.should_transmit():
                msg = self.mgr.inbox.get()
                self.mgr.transition(Manager.Transmitting(self.mgr, msg))
            elif self.mgr.rx_enabled:
                self.mgr.transition(Manager.Receiving(self.mgr))

    class Transmitting(State):
        def __init__(self, mgr, msg):
            super().__init__(mgr)
            self.msg = msg

        def enter(self):
            assert self.mgr.tx_enabled
            logging.info("Transmitting %d bytes", len(self.msg))
            drv = self.mgr.driver
            # Write to PWRMODE to avoid errata
            drv.set_pwrmode(Pwrmode.FIFOON)
            # Set Tx-specific configs
            drv.execute({Reg.PLLVCODIV: 0x24, Reg.TUNE_F18: 0x06})
            drv.set_pwrmode(Pwrmode.FULLTX)
            self.mgr.enable_pa(True)
            # Before writing to the FIFO, wait for voltage regulator to finish starting up
            self.mgr.poll(Reg.POWSTAT, Bits.SVMODEM)
            # Write to FIFO
            # Preamble (repeat raw 0xAA to generate 272 alternating bits)
            drv.write_fifo(bytearray([0x62, 0x38, 0x21, 0xAA]))
            # Sync word (undocumented command to write 0xCCAACCAA)
            drv.write_fifo(bytearray([0xA1, 0x18, 0xCC, 0xAA, 0xCC, 0xAA]))
            # Data (no framing, full packet)
            # First byte must be length of message (including itself)
            drv.write_fifo_data(bytearray([len(self.msg) + 1]) + self.msg)
            # Wait until crystal oscillator is running
            self.mgr.poll(Reg.XTALSTATUS, Bits.XTAL_RUN)
            # Commit FIFO
            drv.execute({Reg.FIFOSTAT: Fifocmd.COMMIT})
            # TODO: msg larger than fifo

        def dispatch(self):
            if not self.mgr.tx_enabled:
                # TODO: abort notice?
                logging.warn("Transmission aborted")
                self.mgr.transition(Manager.Idle(self.mgr))
            # Wait until transmission is done (TODO: 0-timeout)
            # radiostate = self.mgr.driver.poll(Reg.RADIOSTATE, 0xFF, 0)
            radiostate = self.mgr.driver.read(Reg.RADIOSTATE)
            if radiostate == 0:
                if self.mgr.should_transmit():
                    # No transmission to avoid powering down PA on exit
                    self.msg = self.mgr.inbox.get()
                    self.enter()
                elif self.mgr.rx_enabled:
                    self.mgr.transition(Manager.Receiving(self.mgr))
                else:
                    self.mgr.transition(Manager.Idle(self.mgr))

        def exit(self):
            self.mgr.enable_pa(False)
            self.mgr.driver.set_pwrmode(Pwrmode.STANDBY)

    class Receiving(State):
        def __init__(self, mgr):
            super().__init__(mgr)
            self.data = bytearray()
            # If true, FIFO should not be drained prior to being cleared when
            # exiting this state.
            self.bad_fifo = False

        def enter(self):
            assert self.mgr.rx_enabled
            # Write to PWRMODE to avoid errata
            self.mgr.driver.set_pwrmode(Pwrmode.FIFOON)
            # Set Rx-specific configs (TODO: why are these different from Tx?)
            self.mgr.driver.execute({Reg.PLLVCODIV: 0x25, Reg.TUNE_F18: 0x02})
            self.mgr.driver.set_pwrmode(Pwrmode.FULLRX)

        def dispatch(self):
            # TODO: poll radiostate for whether currently receiving?
            if self.mgr.should_transmit():
                msg = self.mgr.inbox.get()
                self.mgr.transition(Manager.Transmitting(self.mgr, msg))
            elif not self.mgr.rx_enabled:
                self.mgr.transition(Manager.Idle(self.mgr))
            else:
                self.drain_fifo()

        def exit(self):
            self.mgr.driver.set_pwrmode(Pwrmode.STANDBY)
            if not self.bad_fifo:
                self.drain_fifo()
            self.mgr.driver.execute({Reg.FIFOSTAT: Fifocmd.CLEAR_DATA_FLAGS})

        def drain_fifo(self):
            fifocount = self.mgr.driver.read_16(Reg.FIFOCOUNT1)
            if fifocount > 0:
                self.data += self.mgr.driver.read_fifo(fifocount)
                while len(self.data) > 0:
                    try:
                        (chunk, self.data) = Chunk.from_bytes(self.data)
                    except Exception as e:
                        logging.error(e)
                        self.bad_fifo = True
                        self.mgr.transition(Manager.Receiving(self.mgr))
                        return
                    logging.debug("Parsed chunk %s", chunk)
                    # TODO: metadata
                    if isinstance(chunk, DataChunk):
                        # TODO: multipart
                        logging.info("Received %d bytes", len(chunk.data))
                        assert len(chunk.data) > 0
                        # First byte is packet length (including itself)
                        if chunk.data[0] == len(chunk.data):
                            self.mgr.outbox.put(chunk.data[1:])
                        else:
                            logging.error(
                                "First byte (%d) does not match length (%d)",
                                chunk.data[0],
                                len(chunk.data),
                            )
                            # TODO: What next?
                    elif chunk is None:
                        # TODO: abort if not making progress
                        break

    class Error(State):
        def __init__(self, mgr, err):
            super().__init__(mgr)
            self.err = err

        def enter(self):
            logging.error(self.err)
            self.mgr.driver.set_pwrmode(Pwrmode.POWERDOWN)
            # This should be handled by the exit actions of any states that use
            # the PA, but assert here as a fallback.
            self.mgr.enable_pa(False)


# Configuration for Cislunar Explorers
setup_cmds = {
    Reg.MODULATION: 0x04,
    Reg.ENCODING: 0x03,
    Reg.FRAMING: 0x06,
    Reg.PINFUNCSYSCLK: 0x01,
    Reg.PINFUNCDCLK: 0x01,
    Reg.PINFUNCDATA: 0x01,
    Reg.PINFUNCANTSEL: 0x01,
    Reg.PINFUNCPWRAMP: 0x07,
    Reg.PLLLOOP: 0x0B,
    Reg.PLLCPI: 0x10,
    Reg.PLLVCODIV: 0x24,
    Reg.FREQA3: 0x09,
    Reg.FREQA2: 0x1D,
    Reg.FREQA1: 0x55,
    Reg.FREQA0: 0x55,
    Reg.WAKEUPXOEARLY: 0x01,
    Reg.IFFREQ1: 0x01,
    Reg.IFFREQ0: 0x11,
    Reg.DECIMATION: 0x1E,
    Reg.RXDATARATE1: 0x50,
    Reg.RXDATARATE0: 0x00,
    Reg.MAXDROFFSET0: 0x00,
    Reg.MAXRFOFFSET2: 0x80,
    Reg.MAXRFOFFSET1: 0x00,
    Reg.MAXRFOFFSET0: 0x00,
    Reg.RXPARAMSETS: 0xF4,
    Reg.AGCGAIN0: 0xD6,
    Reg.AGCTARGET0: 0x84,
    Reg.TIMEGAIN0: 0xA9,
    Reg.DRGAIN0: 0xA3,
    Reg.FREQUENCYGAINA0: 0x46,
    Reg.FREQUENCYGAINB0: 0x0A,
    Reg.FREQUENCYGAINC0: 0x1F,
    Reg.FREQUENCYGAIND0: 0x1F,
    Reg.AMPLITUDEGAIN0: 0x06,
    Reg.FREQDEV00: 0x00,
    Reg.BBOFFSRES0: 0x00,
    Reg.AGCGAIN1: 0xD6,
    Reg.AGCTARGET1: 0x84,
    Reg.TIMEGAIN1: 0xA7,
    Reg.DRGAIN1: 0xA2,
    Reg.FREQUENCYGAINA1: 0x46,
    Reg.FREQUENCYGAINB1: 0x0A,
    Reg.FREQUENCYGAINC1: 0x1F,
    Reg.FREQUENCYGAIND1: 0x1F,
    Reg.AMPLITUDEGAIN1: 0x06,
    Reg.FREQDEV01: 0x00,
    Reg.FOURFSK1: 0x16,
    Reg.BBOFFSRES1: 0x00,
    Reg.AGCTARGET3: 0x84,
    Reg.TIMEGAIN3: 0xA6,
    Reg.DRGAIN3: 0xA1,
    Reg.FREQUENCYGAINA3: 0x46,
    Reg.FREQUENCYGAINB3: 0x0A,
    Reg.FREQUENCYGAINC3: 0x1F,
    Reg.FREQUENCYGAIND3: 0x1F,
    Reg.AMPLITUDEGAIN3: 0x06,
    Reg.FREQDEV03: 0x00,
    Reg.FOURFSK3: 0x16,
    Reg.BBOFFSRES3: 0x00,
    Reg.FSKDEV1: 0x00,
    Reg.FSKDEV0: 0x00,
    Reg.MODCFGA: 0x06,
    Reg.TXRATE1: 0x06,
    Reg.TXRATE0: 0xD4,
    Reg.TXPWRCOEFFB1: 0x02,
    Reg.TXPWRCOEFFB0: 0x07,
    Reg.PLLVCOI: 0x98,
    Reg.PLLRNGCLK: 0x05,
    Reg.BBTUNE: 0x0F,
    Reg.PKTADDRCFG: 0x01,
    Reg.PKTLENCFG: 0x80,
    Reg.PKTMAXLEN: 0xC8,
    Reg.MATCH0PAT3: 0xAA,
    Reg.MATCH0PAT2: 0xCC,
    Reg.MATCH0PAT1: 0xAA,
    Reg.MATCH0PAT0: 0xCC,
    Reg.MATCH0LEN: 0x1F,
    Reg.MATCH1PAT1: 0x55,
    Reg.MATCH1PAT0: 0x55,
    Reg.MATCH1LEN: 0x8A,
    Reg.MATCH1MAX: 0x0A,
    Reg.TMGTXBOOST: 0x5B,
    Reg.TMGTXSETTLE: 0x3E,
    Reg.TMGRXBOOST: 0x5B,
    Reg.TMGRXSETTLE: 0x3E,
    Reg.TMGRXOFFSACQ: 0x00,
    Reg.TMGRXCOARSEAGC: 0x9C,
    Reg.TMGRXRSSI: 0x03,
    Reg.TMGRXPREAMBLE2: 0x35,
    Reg.RSSIABSTHR: 0xE0,
    Reg.PKTCHUNKSIZE: 0x0D,
    Reg.PKTSTOREFLAGS: 0x54,
    Reg.PKTACCEPTFLAGS: 0x1C,
    Reg.TUNE_F00: 0x0F,
    Reg.REF: 0x03,
    Reg.XTALAMPL: 0x00,
    Reg.TUNE_F1C: 0x07,
    Reg.TUNE_F21: 0x68,
    Reg.TUNE_F22: 0xFF,
    Reg.TUNE_F23: 0x84,
    Reg.TUNE_F26: 0x98,
    Reg.TUNE_F34: 0x28,
    Reg.TUNE_F35: 0x11,
    Reg.MODCFGP: 0xE1,
}

# Settings for 500 bps (F_XTAL = 48 MHz)
datarate_cmds = {
    Reg.DECIMATION: 0x7F,
    Reg.RXDATARATE2: 0x00,
    Reg.RXDATARATE1: 0x5E,
    Reg.RXDATARATE0: 0x7C,
    Reg.AGCGAIN0: 0xE8,
    Reg.TIMEGAIN0: 0xB9,
    Reg.DRGAIN0: 0xB3,
    Reg.AGCGAIN1: 0xE8,
    Reg.TIMEGAIN1: 0xB7,
    Reg.DRGAIN1: 0xB2,
    Reg.TIMEGAIN3: 0xB6,
    Reg.DRGAIN3: 0xB1,
    Reg.TXRATE1: 0x00,
    Reg.TXRATE0: 0xAF,
    Reg.TUNE_F35: 0x12,
}
