"""
Scripts used in the communications with and between Raspberry Pis.

Functions
---------
createInputs:
    Establish the analog inputs to record EMG
decodeEMG:
    Convert byte string EMG data to floats
emgClient:
    Send EMG data to the EMG server
emgEstablishServer:
    Open the TCPIP server to receive EMG data
pollEMG:
    Get EMG signals and normalize them.
receiveEMG:
    Pull EMG from the Raspberry Pi Zero W
switchMode:
    Switches walking modes if the user is co-contracting their muscles.

Notes
-----
This module will not function unless it is used on a Raspberry Pi or other
board that can use the board library from Adafruit.
"""
import busio
import digitalio
try:
    import board
except NotImplementedError:
    print("Could not find a board")
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
from typing import Tuple
import socket
from hexapod.xboxController import xboxController


def createInputs() -> Tuple[AnalogIn, AnalogIn]:
    """
    Establish the analog inputs to record EMG

    Create the analog inputs on the MCP3008 for recording EMG on the raspberry
    pi zero

    Returns
    -------
    [fcr_channel, edc_channel]: Tuple[AnalogIn, AnalogIn]
        The analog channels for recording EMG.
    See Also
    --------
    hexapod.piToPi.receiveEMG
    """
    spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
    cs = digitalio.DigitalInOut(board.D5)
    mcp = MCP.MCP3008(spi, cs)

    # setup the EMG channels to record from
    fcr_channel = AnalogIn(mcp, MCP.P0)
    edc_channel = AnalogIn(mcp, MCP.P1)

    return (fcr_channel, edc_channel)


def decodeEMG(conn: socket) -> Tuple[float, float]:
    """
    Convert byte string EMG data to floats

    Takes the received EMG data in the form of a byte string and converts the
    data to floats by splitting on the commas

    Parameters
    ----------
    conn: socket
        The server socket EMG data is send to

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


def decodeGamePad(conn: socket) -> Tuple[float, float]:
    """
    Convert byte string EMG data to floats

    Takes the received EMG data in the form of a byte string and converts the
    data to floats by splitting on the commas

    Parameters
    ----------
    conn: socket
        The server socket EMG data is send to

    Returns
    -------
    [fcr_emg, edc_emg]: Tuple[float, float]
        The normalized forearm EMG values.

    See Also
    --------
    hexapod.piToPi.emgEstablishServer
    """
    data = conn.recv(1024)
    input_string = data.decode('UTF-8')
    conn.sendall(data)
    [rs_x, rs_y, rs_t, ls_x, ls_y, ls_t, a, b, y, x, down_up_d, right_left_d,
     rt, rb, lt, lb, back, start] = input_string.split(",")

    return (float(rs_x), float(rs_y), float(rs_t), float(ls_x), float(ls_y),
            float(ls_t), float(a), float(b), float(y), float(x),
            float(down_up_d), float(right_left_d), float(rt), float(rb),
            float(lt), float(lb), float(back), float(start))


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
    HOST = "10.42.0.1"  # The server's hostname or IP address
    PORT = 65432  # The port used by the server

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        fcr_channel, edc_channel = createInputs()
        while True:
            emg = receiveEMG(fcr_channel, edc_channel, gain_fcr=0.5,
                             gain_edc=10)
            emg_string = bytes(str(emg[0]) + "," + str(emg[1]), 'ascii')
            s.sendall(emg_string)
            data = s.recv(1024)
            print(f"Received {data!r}")


def emgEstablishServer() -> socket:
    """
    Open the TCPIP server to receive EMG data

    Establish the TCPIP server on the hexapodNetwork wifi to receive EMG data.

    Returns
    -------
    conn: socket
        A new socket object to send EMG over

    See Also
    --------
    hexapod.piToPi.emgClient
    """
    HOST = "10.42.0.1"  # Standard loopback interface address (localhost)
    PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen()
    conn, _ = s.accept()
    return conn


def gamePadClient() -> None:
    """
    Send EMG data to the EMG server

    Establishes a connection to the EMG server on the hexapodNetwork wifi and
    sends EMG data to it

    See Also
    --------
    hexapod.piToPi.emgEstablishServer
    hexapod.piToPi.receiveEMG
    """
    HOST = "10.42.0.1"  # The server's hostname or IP address
    PORT = 65432  # The port used by the server

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        xbox_controller = xboxController()
        while True:
            inputs = xbox_controller.read()
            # inputs = [rs_x, rs_y, rs_t, ls_x, ls_y, ls_t, a, b, y, x,
            # down_up_d, right_left_d, rt, rb, lt, lb, back, start]
            input_string = ','.join([str(i) for i in inputs])
            input_byte_string = bytes(input_string, 'ascii')
            s.sendall(input_byte_string)
            data = s.recv(1024)
            print(f"Received {data!r}")


def gamePadEstablishServer() -> socket:
    """
    Open the TCPIP server to receive EMG data

    Establish the TCPIP server on the hexapodNetwork wifi to receive EMG data.

    Returns
    -------
    conn: socket
        A new socket object to send EMG over

    See Also
    --------
    hexapod.piToPi.emgClient
    """
    HOST = "10.42.0.1"  # Standard loopback interface address (localhost)
    PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen()
    conn, _ = s.accept()
    return conn


def pollEMG(conn: socket) -> Tuple[float, float]:
    """
    Get EMG signals and normalize them

    Queries the Raspberry pi Zero W for recorded forearm EMG values and then
    scales them from 0 to 1.

    Parameters
    ----------
    conn: socket
        The server socket EMG data is send to

    Returns
    -------
    [fcr_emg, edc_emg]: Tuple[float, float]
        The two forearm EMG signals

    Notes
    -----
    This code just scales the EMG values to set it apart from the code on the
    Raspberry Pi Zero W that records the EMG.
    """
    fcr_emg, edc_emg = decodeEMG(conn)

    fcr_emg = min(fcr_emg, 1.0)
    fcr_emg = max(fcr_emg, 0.0)

    edc_emg = min(edc_emg, 1.0)
    edc_emg = max(edc_emg, 0.0)

    return (fcr_emg, edc_emg)


def pollGamePad(conn: socket) -> Tuple[float, float]:
    """
    Get EMG signals and normalize them

    Queries the Raspberry pi Zero W for recorded forearm EMG values and then
    scales them from 0 to 1.

    Parameters
    ----------
    conn: socket
        The server socket EMG data is send to

    Returns
    -------
    [fcr_emg, edc_emg]: Tuple[float, float]
        The two forearm EMG signals

    Notes
    -----
    This code just scales the EMG values to set it apart from the code on the
    Raspberry Pi Zero W that records the EMG.
    """
    [rs_x, rs_y, rs_t, ls_x, ls_y, ls_t, a, b, y, x, down_up_d, right_left_d,
     rt, rb, lt, lb, back, start] = decodeGamePad(conn)

    return (rs_x, rs_y, rs_t, ls_x, ls_y, ls_t, a, b, y, x, down_up_d,
            right_left_d, rt, rb, lt, lb, back, start)


def receiveEMG(fcr_channel: AnalogIn, edc_channel: AnalogIn,
               gain_fcr: float = 5,
               gain_edc: float = 5) -> Tuple[float, float]:
    """
    Pull EMG from the Raspberry Pi Zero W

    Read in EMG values on the first two ADC channels of the Raspberry Pi
    zero through the MCP3008.

    Parameters
    ----------
    fcr_channel: AnalogIn
        The flexor EMG channel
    edc_channel: AnalogIn
        The extensor EMG channel
    gain_fcr: int
        A multiplier of the flexor EMG input to amplify the signal.
    gain_edc: int
        A multiplier of the extensor EMG input to amplify the signal.

    Returns
    -------
    [fcr_emg, edc_emg]: Tuple[float, float]
        The normalized forearm EMG values.

    See Also
    --------
    hexapod.piToPi.createInputs
    hexapod.piToPi.emgClient
    """
    # get the 16 value on each EMG channel and normalize it
    fcr_emg = fcr_channel.value / 65536.0 * gain_fcr
    edc_emg = edc_channel.value / 65536.0 * gain_edc

    return (fcr_emg, edc_emg)


def switchMode(conn: socket, threshold: float) -> bool:
    """
    Switches walking modes if the user is co-contracting their muscles.

    Checks if both EMG signals are above a threshold value to indicate if the
    hexapod should switch movement modes.

    Parameters
    ----------
    conn: socket
        The server socket EMG data is send to
    threshold: float
        The value above which both EMG signals need to be to cause a mode
        switch. This value should be between 0 and 1.

    Returns
    -------
    bool
        True if both EMG signals are above the threshold input.

    See Also
    --------
    pollEMG:
        Get EMG signals and normalize them

    Notes
    -----
    This function is called after every EMG based step to see if the person is
    co-contracting hard enough to switch modes.
    """
    [fcr_emg, edc_emg] = pollEMG(conn)

    return bool(fcr_emg > threshold and edc_emg > threshold)
