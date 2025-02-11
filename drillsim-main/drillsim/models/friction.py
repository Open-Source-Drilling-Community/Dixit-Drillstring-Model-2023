from drillsim.constants import *
import numpy as np


def Friction_imp(z, v, theta, omega, Forcing_F, Forcing_T, static_check_prev, p):
    """
    Friction model of the system
    =============================
    This function enforces stribeck friction model on the system with following features :
    a. Friction model is coupled i.e, resultant reaction force from axial and tangential direction is taken into account
    b. No movement untill the resultant reaction force reaches static limit
    c. coloumb friction model is implemented when the drill string is in motion
    d. STATIC_CHECK_PREV is used to track the friction state for each element

    Parameters
    ----------
    z : `float`,
        displacement in meters
    v : `float`,
        velocity in m/sec
    theta : `float`,
        rotation angle in radians
    omega : `float`,
        rotational speed in rads/sec
    Forcing_F : `np.array`,
        Forcing_F vector
    Forcing_T : `np.array`,
        Forcing_T vector
    static_check_prev :`np.array`,
        A vector containing state of friction ( 0 being static friction,
        1 representing dynamic friction ) against each element
    p : `dict`,
        A dictionary containing the constant values

    Returns
    -------
    out : tuple of `np.array`
        Friction_force
        Friction_torque

    """
    Normal_force = p[NORMAL_FORCE]
    ca_array = p[GLOBAL_CA_ARRAY]
    KA_top_s = p[GLOBAL_KA_MATRIX]
    ct_array = p[GLOBAL_CT_ARRAY]
    KT_top_s = p[GLOBAL_KT_MATRIX]
    dia_pipe_equiv = p[DIA_PIPE_EQUIV]
    noe = p[NOE]
    mu_static = p[MU_STATIC]
    mu_dynamic = p[MU_DYNAMIC]
    v_cs = p[V_CS]
    motor_elem = p[MOTOR_INDEX]
    Friction_limit = mu_static * Normal_force
    v_tan = ((0.0254 * 0.5 * dia_pipe_equiv)) * (omega)
    resultant_vel = np.abs(np.sqrt(v**2 + v_tan**2))

    Friction_force = np.zeros(noe)
    Friction_torque = np.zeros(noe)

    mu_effective = mu_dynamic * np.ones(noe) + (mu_static - mu_dynamic) * np.exp(
        (-1) * (resultant_vel / v_cs)
    )

    coloumb_r = mu_effective * Normal_force
    Fd_a = KA_top_s.dot(z) + ca_array * v - Forcing_F
    Fd_t = (
        (KT_top_s.dot(theta)) * (2 / (0.0254 * dia_pipe_equiv))
        + (ct_array * omega) * (0.5 * 0.0254 * dia_pipe_equiv)
        - Forcing_T * (2 / (0.0254 * dia_pipe_equiv))
    )
    Fd_resultant = np.abs(np.sqrt(Fd_a**2 + Fd_t**2))

    comp_a = np.where(np.isnan(v / resultant_vel), 0, v / resultant_vel)
    comp_t = np.where(np.isnan(v_tan / resultant_vel), 0, v_tan / resultant_vel)

    for i in range(noe):
        if resultant_vel[i] < 1e-06 and static_check_prev[i] == 1:
            static_check_prev[i] = 0
        if Fd_resultant[i] > Friction_limit[i]:
            static_check_prev[i] = 1

    for i in range(noe):
        if Fd_resultant[i] == 0.0:
            Friction_force[i] = 0.0
            Friction_torque[i] = 0.0
            continue

        if static_check_prev[i] == 0:
            Friction_force[i] = -1.0 * Fd_a[i]
            Friction_torque[i] = -1.0 * Fd_t[i] * (0.0254 * 0.5 * dia_pipe_equiv)

        elif static_check_prev[i] == 1:
            if coloumb_r[i] < 10 ** (-10):
                coloumb_r[i] = 0
            Friction_force[i] = coloumb_r[i] * comp_a[i]

            Friction_torque[i] = (coloumb_r[i] * comp_t[i]) * (
                0.0254 * 0.5 * dia_pipe_equiv
            )
    if motor_elem != "N":
        Friction_force[motor_elem + 1] = 0
        Friction_torque[motor_elem + 1] = 0
    out = Friction_force, Friction_torque, static_check_prev
    return out
