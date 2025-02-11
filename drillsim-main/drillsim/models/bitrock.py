from drillsim.constants import *
import numpy as np


def bit_rock(z, theta, p):
    """
    Bit-rock interaction model developed by Rajat Dixit and Paul Pastusek.
    The function also stores hole depths and bit depths during the analysis.

    Parameters
    ----------
    z : `float`,
        bit displacement in meters
    theta : `float`,
        bit angular displacement in rads
    p : `dict`,
        dictionary of important constants

    Returns
    -------
    doc : `float`
        depth of cut value

    """
    theta_prev = p[THETA_PREV][-1]
    hole_depth_prev = p[HOLE_DEPTH_PREV][-1]

    del_theta = theta - theta_prev
    bit_depth = z
    if bit_depth > hole_depth_prev:
        doc = bit_depth - hole_depth_prev
    else:
        doc = 0.0
    if del_theta >= 0.0:
        hole_depth = hole_depth_prev + doc * (del_theta / (2 * np.pi))
    else:
        hole_depth = hole_depth_prev
    theta_prev = theta

    p[THETA_PREV].append(theta_prev)
    p[HOLE_DEPTH_PREV].append(hole_depth)

    return doc
