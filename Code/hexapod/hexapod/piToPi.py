def pollEMG():
    # TODO: Get the recorded EMG from the raspberry pi zero and normalize it.
    fcr_emg = 1
    edc_emg = 1

    fcr_emg = min(fcr_emg, 1)
    fcr_emg = max(fcr_emg, 0)

    edc_emg = min(edc_emg, 1)
    edc_emg = max(edc_emg, 0)

    return [fcr_emg, edc_emg]
