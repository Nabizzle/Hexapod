"""
Scripts used in the communications with and between Raspberry Pis.

Functions
---------
recieveEMG: Pull EMG from the Raspberry Pi Zero W

Notes
-----
This module will not function unless it is used on a Raspberry Pi or other
board that can use the board library from Adafruit.
"""
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
from typing import Tuple


def recieveEMG() -> Tuple[float, float]:
    """
    Pull EMG from the Raspberry Pi Zero W
    
    Read in EMG values on the first two ADC channels of the Raspberry Pi
    zero through the MCP3008. This is done through a TCP IP socket.

    Returns
    -------
    [fcr_emg, edc_emg]: Tuple[float, float]
        The normalized forearm EMG values.
    """
    spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
    cs = digitalio.DigitalInOut(board.D5)
    mcp = MCP.MCP3008(spi, cs)

    # setup the EMG channels to record from
    fcr_channel = AnalogIn(mcp, MCP.P0)
    edc_channel = AnalogIn(mcp, MCP.P1)
    # get the 16 value on each EMG channel and normalize it
    fcr_emg = fcr_channel.value / 65536.0
    edc_emg = edc_channel.value / 65536.0

    return (fcr_emg, edc_emg)
