from drillsim.constants import *
from drillsim.inputs import top_drive_input as top_drive_input
from drillsim.models.friction import Friction_imp as Friction_stribeck
from drillsim.models.coulomb import Friction_coulomb
from drillsim.models.bitrock import bit_rock as bit_rock
import numpy as np
import scipy


def Main_Func(t, x, p, fric_mod):

    """
    This function has the lumped spring mass damper dynamic equations in axial and torsional directions
    The second order ode is broken down to two first order ODE's to solve the problem
    This is the driver function to ODE solver

    Parameters
    ----------
    t : `float`,
        time in seconds

    x : `np.array`,
        global dof vector : [z,v,theta,omega]*noe

    p : `dict`,
        dictionary containing important constants

    Returns
    -------
    dx : `np.array`
        global dof_dot vector : [z_dot, v_dot, theta_dot, omega_dot]*noe

    """
    # import and store all the constants, pre assembled global matrices through dictionary
    noe = p[NOE]
    ka_array = p[GLOBAL_KA_ARRAY]
    kt_array = p[GLOBAL_KT_ARRAY]
    motor_elem = p[MOTOR_INDEX]
    k_WOB = p[K_WOB]
    k_TQ = p[K_TQ]
    MK_top_s = p[GLOBAL_MASS_INV_KA_MATRIX]
    m_inv_array = p[GLOBAL_MASS_INV_ARRAY]
    mca_array = p[GLOBAL_MASS_INV_CA_ARRAY]
    JK_top_s = p[GLOBAL_INERTIA_INV_KT_MATRIX]
    J_inv_array = p[GLOBAL_INERTIA_INV_ARRAY]
    Jct_array = p[GLOBAL_INERTIA_INV_CT_ARRAY]
    dia_pipe_equiv = p[DIA_PIPE_EQUIV]

    dx = np.zeros(4 * noe)
    Forcing_F = np.zeros(noe)
    Forcing_T = np.zeros(noe)
    _, _, z_top_drive, theta_top_drive = top_drive_input(t, p)
    Forcing_F[0] = ka_array[0] * z_top_drive
    Forcing_T[0] = kt_array[0] * theta_top_drive

    mm_omega = np.zeros(noe)
    if motor_elem != "N":
        motor_speed = 2 * np.pi * p[MOTOR_RPG] * p[MOTOR_FLOW_RATE]*(1/60)
        mm_omega[motor_elem::] = motor_speed

    z = x[0::4]
    v = x[1::4]
    theta = x[2::4] - mm_omega * t
    omega = x[3::4] - mm_omega
    doc = bit_rock(z[-1], theta[-1], p)

    Forcing_F[-1] = -k_WOB * doc  # - c_bit_axial * v[-1] * abs(np.sign(doc))
    Forcing_T[-1] = -k_TQ * doc

    if fric_mod == "stribeck":
        Friction_force, Friction_torque, p[STATIC_CHECK_PREV], new_fric_force = Friction_stribeck(
            z, v, theta, omega, Forcing_F, Forcing_T, p[STATIC_CHECK_PREV], p
        )
    elif fric_mod == "coulomb":
        Friction_force, Friction_torque, p[STATIC_CHECK_PREV], new_fric_force = Friction_coulomb(
            z, v, theta, omega, Forcing_F, Forcing_T, p[STATIC_CHECK_PREV], p
        )

    # store weight , torque , depth of cut and solution time
    # TODO: Dense outputs needn't require storing solution time

    p['new_force'].append(new_fric_force[-1])
    p[DOWNHOLE_WEIGHT].append(Forcing_F[-1])
    p[DOWNHOLE_TORQUE].append(Forcing_T[-1])
    p[DOC].append(doc)
    p[SOLUTION_TIME].append(t)
    p[FRICTION_FORCE_STORE].append(Friction_force)
    p[FRICTION_TORQUE_STORE].append(Friction_torque)
    p[STATIC_CHECK_PREV_STORE].append(p[STATIC_CHECK_PREV])

    # Governing Equations
    dx[0::4] = np.array(v)
    dx[1::4] = (
        -MK_top_s.dot(z) + m_inv_array * (Forcing_F - Friction_force) - mca_array * v
    )
    dx[2::4] = np.array(omega + mm_omega)
    dx[3::4] = (
        -JK_top_s.dot(theta)
        + (J_inv_array) * (Forcing_T - Friction_torque)
        - (Jct_array * omega) * ((0.5 * 0.0254 * dia_pipe_equiv) ** 2)
    )
    return dx
