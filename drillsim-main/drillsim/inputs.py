from drillsim.constants import *
import numpy as np


def top_drive_input(t, p):
    """
    Calculates area under the curve for ROP and RPM functions till the time t
    (Currently need to enter the manual function to speed up)
    Parameters
    ----------
    t : `float`,
        time in seconds

    Returns
    -------
    out : tuple of 'floats'
        ROP_topdrive = axial topdrive velocity from t = 0 to t = 't' seconds
        RPM_topdrive = RPM from t = 0 to t = 't' seconds
        z_top_drive = area under ROP curve from t = 0 to t = 't' seconds
        theta_top_drive = area under RPM curve from t = 0 to t = 't' seconds

    """
    ROP_in_units_1 = p[ROP_VAL_1] / CONV_1
    ROP_in_units_2 = p[ROP_VAL_2] / CONV_1

    if 0 <= t < p[A1]:
        ROP_topdrive = ((ROP_in_units_1 / p[A1])) * t
        z_top_drive = ROP_topdrive * 0.5 * t
    elif p[A1] <= t < p[A2]:
        ROP_topdrive = ROP_in_units_1
        z_top_drive = ROP_topdrive * (t - 0.5 * p[A1])
    elif p[A2] <= t < p[A3]:
        ROP_topdrive = ((ROP_in_units_1) * (p[A3] - t)) / (p[A3] - p[A2])
        z_top_drive = ROP_in_units_1 * (p[A2] - 0.5 * p[A1]) + (
            0.5 * ROP_in_units_1 * (t - p[A2]) * (2 * p[A3] - p[A2] - t)
        ) / (p[A3] - p[A2])
    elif p[A3] <= t < p[A4]:
        ROP_topdrive = ((ROP_in_units_2) * (p[A3] - t)) / (p[A4] - p[A3])
        z_top_drive = 0.5 * (p[A3] + p[A2] - p[A1]) * ROP_in_units_1 - (
            0.5 * (t - p[A3]) ** 2
        ) * (ROP_in_units_2 / (p[A4] - p[A3]))
    elif p[A4] <= t < p[A5]:
        ROP_topdrive = -(ROP_in_units_2)
        z_top_drive = (
            0.5 * (p[A3] + p[A2] - p[A1]) * ROP_in_units_1
            - 0.5 * (p[A4] - p[A3]) * ROP_in_units_2
            - (t - p[A4]) * ROP_in_units_2
        )
    elif p[A5] <= t < p[A6]:
        ROP_topdrive = ((ROP_in_units_2) * (t - p[A6])) / (p[A6] - p[A5])
        z_top_drive = (
            0.5 * (p[A3] + p[A2] - p[A1]) * ROP_in_units_1
            - 0.5 * (p[A6] - p[A3] + p[A5] - p[A4]) * ROP_in_units_2
        )
    # RPM and theta_top_drive

    RPM_in_units_1 = p[RPM_VAL_1] * CONV_2
    RPM_in_units_2 = p[RPM_VAL_2] * CONV_2

    if 0 <= t < p[B1]:
        RPM_topdrive = ((RPM_in_units_1 / p[B1])) * t
        theta_top_drive = RPM_topdrive * 0.5 * t
    elif p[B1] <= t < p[B2]:
        RPM_topdrive = RPM_in_units_1
        theta_top_drive = RPM_topdrive * (t - 0.5 * p[B1])
    elif p[B2] <= t < p[B3]:
        RPM_topdrive = ((RPM_in_units_1) * (p[B3] - t)) / (p[B3] - p[B2])
        theta_top_drive = RPM_in_units_1 * (p[B2] - 0.5 * p[B1]) + (
            0.5 * RPM_in_units_1 * (t - p[B2]) * (2 * p[B3] - p[B2] - t)
        ) / (p[B3] - p[B2])
    elif p[B3] <= t < p[B4]:
        RPM_topdrive = ((RPM_in_units_2) * (p[B3] - t)) / (p[B4] - p[B3])
        theta_top_drive = 0.5 * (p[B3] + p[B2] - p[B1]) * RPM_in_units_1 - (
            0.5 * (t - p[B3]) ** 2
        ) * (RPM_in_units_2 / (p[B4] - p[B3]))
    elif p[B4] <= t < p[B5]:
        RPM_topdrive = -(RPM_in_units_2)
        theta_top_drive = (
            0.5 * (p[B3] + p[B2] - p[B1]) * RPM_in_units_1
            - 0.5 * (p[B4] - p[B3]) * RPM_in_units_2
            - (t - p[B4]) * RPM_in_units_2
        )
    elif p[B5] <= t < p[B6]:
        RPM_topdrive = ((RPM_in_units_2) * (t - p[B6])) / (p[B6] - p[B5])
        theta_top_drive = (
            0.5 * (p[B3] + p[B2] - p[B1]) * RPM_in_units_1
            - 0.5 * (p[B6] - p[B3] + p[B5] - p[B4]) * RPM_in_units_2
        )

    z_heave = 0
    ROP_heave = 0
    if p[HEAVE_STATE] == "Y":
        if t <= p[HEAVE_DELAY]:
            z_heave = 0
            ROP_heave = 0
        else:
            z_heave = p[HEAVE_AMP] * np.sin(p[HEAVE_OMEGA] * (t - p[HEAVE_DELAY]))
            ROP_heave = (
                p[HEAVE_AMP]
                * p[HEAVE_OMEGA]
                * np.cos(p[HEAVE_OMEGA] * (t - p[HEAVE_DELAY]))
            )

    out = (ROP_topdrive + ROP_heave, RPM_topdrive, z_top_drive + z_heave, theta_top_drive)

    return out
