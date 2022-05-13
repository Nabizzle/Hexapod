"""
Scripts used in the communications with and between Raspberry Pis.

Functions
---------
receiveEMG:
    Pull EMG from the Raspberry Pi Zero W
emgEstablishserver:
    Open the TCPIP server to recieve EMG data
decodeEMG:
    Convert byte string EMG data to floats
emgClient:
    Send EMG data to the EMG server

Notes
-----
This module will not function unless it is used on a Raspberry Pi or other
board that can use the board library from Adafruit.
"""
import busio
import digitalio
try:
    import board
except(NotImplementedError):
    print("Could not find a board")
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
from typing import Tuple
import socket


def receiveEMG(fcr_emg: float = 0, edc_emg: float = 0, gain_fcr: float = 5,
               gain_edc: float = 5) -> Tuple[float, float]:
    """
    Pull EMG from the Raspberry Pi Zero W

    Read in EMG values on the first two ADC channels of the Raspberry Pi
    zero through the MCP3008. This is done through a TCP IP socket.

    Parameters
    ----------
    gain: int
        A multiplier of the EMG input to amplify the signal.

    Returns
    -------
    [fcr_emg, edc_emg]: Tuple[float, float]
        The normalized forearm EMG values.

    See Also
    --------
    hexapod.piToPi.emgClient
    """
    spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
    cs = digitalio.DigitalInOut(board.D5)
    mcp = MCP.MCP3008(spi, cs)

    # setup the EMG channels to record from
    fcr_channel = AnalogIn(mcp, MCP.P0)
    edc_channel = AnalogIn(mcp, MCP.P1)
    # get the 16 value on each EMG channel and normalize it
    fcr_emg = fcr_channel.value / 65536.0 * gain_fcr
    edc_emg = edc_channel.value / 65536.0 * gain_edc

    return (fcr_emg, edc_emg)


def emgEstablishServer() -> socket:
    """
    Open the TCPIP server to recieve EMG data

    Establish the TCPIP server on the hexapodNetwork wifi to recieve EMG data.

    Returns
    -------
    conn: socket
        A new socket opbject to send EMG over

    See Also
    --------
    hexapod.piToPi.emgClient
    """
    HOST = "192.168.4.1"  # Standard loopback interface address (localhost)
    PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen()
    conn, _ = s.accept()
    return conn


def decodeEMG(conn: socket, fcr_emg: float = 0,
              edc_emg: float = 0) -> Tuple[float, float]:
    """
    Convert byte string EMG data to floats

    Takes the received EMG data in the form of a byte string and converts the
    data to floats by splitting on the commas

    Parameters
    ----------
    conn: socket
        The server socket EMG data is send to
    fcr_emg: float
        The wrist flexor EMG data
    edc_emg: float
        The wrist extensor EMG data
    
    Returns
    -------
    [fcr_emg, edc_emg]: Tuple[float, float]
        The normalized forearm EMG values.

    See Also
    --------
    hexapod.piToPi.emgEstablishServer
    """
    data = conn.recv(1024)
    emg_string = data.decode('UTF-8')
    conn.sendall(data)
    emg = emg_string.split(",")
    fcr_emg = float(emg[0])
    edc_emg = float(emg[1])

    return (fcr_emg, edc_emg)


def emgClient() -> None:
    """
    Send EMG data to the EMG server

    Establishes a connection to the EMG server on the hexapodNetwork wifi and
    sends EMG data to it

    See Also
    --------
    hexapod.piToPi.emgEstablishServer
    hexapod.piToPi.receiveEMG
    """
    HOST = "192.168.4.1"  # The server's hostname or IP address
    PORT = 65432  # The port used by the server

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        while True:
            emg = receiveEMG(fcr_emg=0, edc_emg=0, gain_fcr=0.5, gain_edc=10)
            emg_string=bytes(str(emg[0]) + "," + str(emg[1]), 'ascii')
            s.sendall(emg_string)
            data = s.recv(1024)
            print(f"Received {data!r}")
