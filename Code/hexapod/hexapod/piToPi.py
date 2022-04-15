def pollEMG():
    # TODO: Get the recorded EMG from the raspberry pi zero and normalize it.
    fcr_emg = 1
    edc_emg = 1

    if fcr_emg > 1:
        fcr_emg = 1
    elif fcr_emg < 0:
        fcr_emg = 0

    if edc_emg > 1:
        edc_emg = 1
    elif edc_emg < 0:
        edc_emg = 0

    return [fcr_emg, edc_emg]
