from drillsim.constants import *
import numpy as np


def Friction_coulomb(z, v, theta, omega, Forcing_F, Forcing_T, static_check_prev, p):
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
    mu_static = p[MU_STATIC]
    mu_dynamic = p[MU_DYNAMIC]
    v_cs = p[V_CS]
    motor_elem = p[MOTOR_INDEX]
    noe = p[NOE]
    
    radius = 0.0254 * 0.5 * dia_pipe_equiv      # Radius in SI
    epsilon = 1e-6                              # Small threshold for zero velocity detection
    Friction_limit = mu_static * Normal_force
    v_tan = omega * radius
    resultant_vel = np.sqrt(v**2 + v_tan**2)
    Force_dynamic = mu_dynamic * Normal_force
    
    # Calculating forces
    Fd_a = -KA_top_s.dot(z) - ca_array * v + Forcing_F
    Fd_t = -KT_top_s.dot(theta) / radius - ct_array * omega * radius + Forcing_T / radius

    # Component adjustments
    comp_a = np.divide(v, resultant_vel, out=np.zeros_like(v), where=resultant_vel!=0)
    comp_t = np.divide(v_tan, resultant_vel, out=np.zeros_like(v_tan), where=resultant_vel!=0)
    
    # Update static check based on velocities
    static_check_prev[resultant_vel < epsilon] = 0
    static_check_prev[(resultant_vel >= epsilon) & (resultant_vel < v_cs)] = 1
    static_check_prev[resultant_vel >= v_cs] = 2

    # Combined calculation of friction force
    Friction_force = np.where(
        static_check_prev == 0,
        np.clip(Fd_a, -Friction_limit, Friction_limit),                         # Full static 
        np.where(
            static_check_prev == 1,
            np.minimum(np.abs(Fd_a), Friction_limit) * np.sign(v),              # Slow move in static
            Force_dynamic * comp_a                                              # Full dynamic
        )
    )

    Friction_torque = np.where(
        static_check_prev == 0,
        np.clip(Fd_t, -Friction_limit, Friction_limit) * radius,                # Full static 
        np.where(
            static_check_prev == 1,
            np.minimum(np.abs(Fd_t), Friction_limit)*np.sign(v_tan)*radius,     # Slow move in static
            Force_dynamic * comp_t * radius                                     # Full dynamic
        )
    )

    # Handle motor element
    if motor_elem != "N":
        Friction_force[motor_elem + 1] = 0
        Friction_torque[motor_elem + 1] = 0

    out = Friction_force, Friction_torque, static_check_prev, Friction_torque/radius
    return out