import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

def pollEMG():
    """Read in EMG values on the first two ADC channels of the Raspberry Pi
    zero through the MCP3008"""
    spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
    cs = digitalio.DigitalInOut(board.D5)
    mcp = MCP.MCP3008(spi, cs)

    # setup the EMG channels to record from
    fcr_channel = AnalogIn(mcp, MCP.P0)
    edc_channel = AnalogIn(mcp, MCP.P1)
    # get the 16 value on each EMG channel and normalize it
    fcr_emg = fcr_channel.value / 65536.0
    edc_emg = edc_channel.value / 65536.0

    fcr_emg = min(fcr_emg, 1.0)
    fcr_emg = max(fcr_emg, 0.0)

    edc_emg = min(edc_emg, 1.0)
    edc_emg = max(edc_emg, 0.0)

    return [fcr_emg, edc_emg]
