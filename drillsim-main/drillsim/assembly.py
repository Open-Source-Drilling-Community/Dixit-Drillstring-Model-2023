from drillsim.constants import *
import numpy as np
import pandas as pd
from drillsim.utilities import LoadSheet as LoadSheet
from drillsim.utilities import nearestLength as nearestLength
import scipy.sparse as sps
import scipy


def addons_BHA(df1, vals_dict):
    youngs_mod = vals_dict[E]
    torsional_mod = vals_dict[G]
    df1["moment_of_inertia"] = (
        (0.5 / 4)
        * (df1["OD (in.)"] ** 2 + df1["ID (in.)"] ** 2)
        * (df1[" Weight (lbf)"] * 0.45)
        * (0.0254**2)
    )
    df1["ka"] = (
        (0.0254**2)
        * (youngs_mod * np.pi / 4)
        * ((df1["OD (in.)"] ** 2 - df1["ID (in.)"] ** 2))
        / df1["Length (m)"]
    )
    df1["kt"] = (
        (0.0254**4)
        * (torsional_mod * np.pi / 32)
        * ((df1["OD (in.)"] ** 4 - df1["ID (in.)"] ** 4))
        / df1["Length (m)"]
    )
    return df1


def hwdp_inputs(df1, p):
    HWDP_TOT_LEN = df1[df1["BHA Element Type"] == "HWDP"]["Length (m)"].sum()
    HWDP_TOT_MASS = 0.45 * df1[df1["BHA Element Type"] == "HWDP"][" Weight (lbf)"].sum()
    HWDP_TOT_INERTIA = df1[df1["BHA Element Type"] == "HWDP"]["moment_of_inertia"].sum()
    HWDP_K_AXIAL = 1 / (np.sum(1 / df1[df1["BHA Element Type"] == "HWDP"]["ka"]))
    HWDP_K_TORSIONAL = 1 / (np.sum(1 / df1[df1["BHA Element Type"] == "HWDP"]["kt"]))
    N_HWDP, L_HWDP = nearestLength(HWDP_TOT_LEN, p[ELEM_LENGTH])
    N_HWDP = int(N_HWDP)
    HWDP_MASS_ARRAY = (HWDP_TOT_MASS / N_HWDP) * np.ones(N_HWDP)
    HWDP_INERTIA_ARRAY = (HWDP_TOT_INERTIA / N_HWDP) * np.ones(N_HWDP)
    HWDP_KA_ARRAY = np.ones(N_HWDP) * (HWDP_K_AXIAL * N_HWDP)
    HWDP_KT_ARRAY = np.ones(N_HWDP) * (HWDP_K_TORSIONAL * N_HWDP)
    L_HWDP_ARRAY = np.ones(N_HWDP) * L_HWDP
    return (
        N_HWDP,
        L_HWDP_ARRAY,
        HWDP_MASS_ARRAY,
        HWDP_INERTIA_ARRAY,
        HWDP_KA_ARRAY,
        HWDP_KT_ARRAY,
    )


def collar_inputs(df1, p):

    COLLAR_TOT_LEN = df1[df1["BHA Element Type"] == "Collar and BHA"][
        "Length (m)"
    ].sum()
    COLLAR_TOT_MASS = (
        0.45 * df1[df1["BHA Element Type"] == "Collar and BHA"][" Weight (lbf)"].sum()
    )
    COLLAR_TOT_INERTIA = df1[df1["BHA Element Type"] == "Collar and BHA"][
        "moment_of_inertia"
    ].sum()
    COLLAR_K_AXIAL = 1 / (
        np.sum(1 / df1[df1["BHA Element Type"] == "Collar and BHA"]["ka"])
    )
    COLLAR_K_TORSIONAL = 1 / (
        np.sum(1 / df1[df1["BHA Element Type"] == "Collar and BHA"]["kt"])
    )
    if p[MOTOR_STATE] == "N":
        N_COLLAR, L_COLLAR = nearestLength(COLLAR_TOT_LEN, p[ELEM_LENGTH])
        N_COLLAR = int(N_COLLAR)
        COLLAR_MASS_ARRAY = (COLLAR_TOT_MASS / N_COLLAR) * np.ones(N_COLLAR)
        COLLAR_INERTIA_ARRAY = (COLLAR_TOT_INERTIA / N_COLLAR) * np.ones(N_COLLAR)
        COLLAR_KA_ARRAY = np.ones(N_COLLAR) * (COLLAR_K_AXIAL * N_COLLAR)
        COLLAR_KT_ARRAY = np.ones(N_COLLAR) * (COLLAR_K_TORSIONAL * N_COLLAR)
        L_COLLAR_ARRAY = np.ones(N_COLLAR) * L_COLLAR
        return (
            N_COLLAR,
            L_COLLAR_ARRAY,
            COLLAR_MASS_ARRAY,
            COLLAR_INERTIA_ARRAY,
            COLLAR_KA_ARRAY,
            COLLAR_KT_ARRAY,
        )
    else:
        df_temp = df1[df1["BHA Element Type"] == "Collar and BHA"]
        list_req = df_temp["Mud Motor"].to_list()
        dummy = list(np.zeros(len(list_req)))
        flag = 0

        for i, vals in enumerate(list_req):
            if vals == "Y":
                dummy[i] = 1
                flag = 2
                continue
            dummy[i] = flag

        df_temp["dummy"] = dummy
        # Above
        COLLAR_TOT_LEN_ABV = df_temp[df_temp["dummy"] == 0]["Length (m)"].sum()
        COLLAR_TOT_MASS_ABV = (
            0.45 * df_temp[df_temp["dummy"] == 0][" Weight (lbf)"].sum()
        )
        COLLAR_TOT_INERTIA_ABV = df_temp[df_temp["dummy"] == 0][
            "moment_of_inertia"
        ].sum()
        COLLAR_K_AXIAL_ABV = 1 / (np.sum(1 / df_temp[df_temp["dummy"] == 0]["ka"]))
        COLLAR_K_TORSIONAL_ABV = 1 / (np.sum(1 / df_temp[df_temp["dummy"] == 0]["kt"]))
        N_COLLAR_ABV, L_COLLAR_ABV = nearestLength(COLLAR_TOT_LEN_ABV, p[ELEM_LENGTH])
        print("checkpoint", COLLAR_TOT_LEN_ABV)
        N_COLLAR_ABV = int(N_COLLAR_ABV)
        COLLAR_MASS_ARRAY_ABV = (COLLAR_TOT_MASS_ABV / N_COLLAR_ABV) * np.ones(
            N_COLLAR_ABV
        )
        COLLAR_INERTIA_ARRAY_ABV = (COLLAR_TOT_INERTIA_ABV / N_COLLAR_ABV) * np.ones(
            N_COLLAR_ABV
        )
        COLLAR_KA_ARRAY_ABV = np.ones(N_COLLAR_ABV) * (
            COLLAR_K_AXIAL_ABV * N_COLLAR_ABV
        )
        COLLAR_KT_ARRAY_ABV = np.ones(N_COLLAR_ABV) * (
            COLLAR_K_TORSIONAL_ABV * N_COLLAR_ABV
        )
        L_COLLAR_ARRAY_ABV = np.ones(N_COLLAR_ABV) * L_COLLAR_ABV

        # Motor
        COLLAR_TOT_LEN_MOTOR = df_temp[df_temp["dummy"] == 1]["Length (m)"].sum()
        COLLAR_TOT_MASS_MOTOR = (
            0.45 * df_temp[df_temp["dummy"] == 1][" Weight (lbf)"].sum()
        )
        COLLAR_TOT_INERTIA_MOTOR = df_temp[df_temp["dummy"] == 1][
            "moment_of_inertia"
        ].sum()
        COLLAR_K_AXIAL_MOTOR = 1 / (np.sum(1 / df_temp[df_temp["dummy"] == 1]["ka"]))
        COLLAR_K_TORSIONAL_MOTOR = 1 / (
            np.sum(1 / df_temp[df_temp["dummy"] == 1]["kt"])
        )
        N_COLLAR_MOTOR = 2
        COLLAR_MASS_ARRAY_MOTOR = np.ones(2) * (COLLAR_TOT_MASS_MOTOR / 2)
        COLLAR_INERTIA_ARRAY_MOTOR = np.ones(2) * (COLLAR_TOT_INERTIA_MOTOR / 2)
        COLLAR_KA_ARRAY_MOTOR = np.array([COLLAR_K_AXIAL_MOTOR, 10**9])
        COLLAR_KT_ARRAY_MOTOR = np.array([COLLAR_K_TORSIONAL_MOTOR, 10**7])
        L_COLLAR_ARRAY_MOTOR = np.array([COLLAR_TOT_LEN_MOTOR, 0])
        # Below
        COLLAR_TOT_LEN_BELOW = df_temp[df_temp["dummy"] == 2]["Length (m)"].sum()
        COLLAR_TOT_MASS_BELOW = (
            0.45 * df_temp[df_temp["dummy"] == 2][" Weight (lbf)"].sum()
        )
        COLLAR_TOT_INERTIA_BELOW = df_temp[df_temp["dummy"] == 2][
            "moment_of_inertia"
        ].sum()
        COLLAR_K_AXIAL_BELOW = 1 / (np.sum(1 / df_temp[df_temp["dummy"] == 2]["ka"]))
        COLLAR_K_TORSIONAL_BELOW = 1 / (
            np.sum(1 / df_temp[df_temp["dummy"] == 2]["kt"])
        )
        N_COLLAR_BELOW, L_COLLAR_BELOW = nearestLength(
            COLLAR_TOT_LEN_BELOW, p[ELEM_LENGTH]
        )
        N_COLLAR_BELOW = int(N_COLLAR_BELOW)
        COLLAR_MASS_ARRAY_BELOW = (COLLAR_TOT_MASS_BELOW / N_COLLAR_BELOW) * np.ones(
            N_COLLAR_BELOW
        )
        COLLAR_INERTIA_ARRAY_BELOW = (
            COLLAR_TOT_INERTIA_BELOW / N_COLLAR_BELOW
        ) * np.ones(N_COLLAR_BELOW)
        COLLAR_KA_ARRAY_BELOW = np.ones(N_COLLAR_BELOW) * (
            COLLAR_K_AXIAL_BELOW * N_COLLAR_BELOW
        )
        COLLAR_KT_ARRAY_BELOW = np.ones(N_COLLAR_BELOW) * (
            COLLAR_K_TORSIONAL_BELOW * N_COLLAR_BELOW
        )
        L_COLLAR_ARRAY_BELOW = np.ones(N_COLLAR_BELOW) * L_COLLAR_BELOW
        # combine
        COLLAR_MASS_ARRAY = np.concatenate(
            [COLLAR_MASS_ARRAY_ABV, COLLAR_MASS_ARRAY_MOTOR, COLLAR_MASS_ARRAY_BELOW]
        )
        COLLAR_INERTIA_ARRAY = np.concatenate(
            [
                COLLAR_INERTIA_ARRAY_ABV,
                COLLAR_INERTIA_ARRAY_MOTOR,
                COLLAR_INERTIA_ARRAY_BELOW,
            ]
        )
        COLLAR_KA_ARRAY = np.concatenate(
            [COLLAR_KA_ARRAY_ABV, COLLAR_KA_ARRAY_MOTOR, COLLAR_KA_ARRAY_BELOW]
        )

        COLLAR_KT_ARRAY = np.concatenate(
            [COLLAR_KT_ARRAY_ABV, COLLAR_KT_ARRAY_MOTOR, COLLAR_KT_ARRAY_BELOW]
        )

        N_COLLAR = N_COLLAR_ABV + N_COLLAR_MOTOR + N_COLLAR_BELOW
        L_COLLAR_ARRAY = np.concatenate(
            [L_COLLAR_ARRAY_ABV, L_COLLAR_ARRAY_MOTOR, L_COLLAR_ARRAY_BELOW]
        )
        return (
            N_COLLAR,
            L_COLLAR_ARRAY,
            COLLAR_MASS_ARRAY,
            COLLAR_INERTIA_ARRAY,
            COLLAR_KA_ARRAY,
            COLLAR_KT_ARRAY,
        )


def drillpipe_inputs(HWDP_TOT_LEN, COLLAR_TOT_LEN, p):

    dp_length = p[DEPTH_QUERY] - (HWDP_TOT_LEN + COLLAR_TOT_LEN)
    N_DP, L_DP = nearestLength(dp_length, p[ELEM_LENGTH])
    N_DP = int(N_DP)
    MASS_DP = (p[M_DP] / p[DRILLPIPE_TOT_LENGTH]) * dp_length
    L_DP_ARRAY = np.ones(N_DP) * L_DP
    DP_MASS_ARRAY = np.ones(N_DP) * (MASS_DP / N_DP)
    J_DP = (
        MASS_DP * (0.5) * ((p[OD_DP] ** 2) + (p[ID_DP] ** 2)) * (0.25) * (0.0254**2)
    )  # units of Mass Moment of Inertia in 'kg-m**2'
    DP_INERTIA_ARRAY = np.ones(N_DP) * (J_DP / N_DP)

    A_DP = (
        (np.pi / 4) * ((p[OD_DP] ** 2) - (p[ID_DP] ** 2)) * (0.0254**2)
    )  # units of Area in 'm**2'
    JP_DP = (
        (np.pi / 32) * ((p[OD_DP] ** 4) - (p[ID_DP] ** 4)) * (0.0254**4)
    )  # units of polar moment of inertia in 'm**4'
    DP_KA_ARRAY = A_DP * p[E] / L_DP_ARRAY
    DP_KT_ARRAY = JP_DP * p[G] / L_DP_ARRAY
    return (N_DP, L_DP_ARRAY, DP_MASS_ARRAY, DP_INERTIA_ARRAY, DP_KA_ARRAY, DP_KT_ARRAY)


def assembler_main(df_dict, p):
    assembly_dict = {}
    (
        N_HWDP,
        L_HWDP_ARRAY,
        HWDP_MASS_ARRAY,
        HWDP_INERTIA_ARRAY,
        HWDP_KA_ARRAY,
        HWDP_KT_ARRAY,
    ) = hwdp_inputs(df_dict["BHA_specs"], p)
    (
        N_COLLAR,
        L_COLLAR_ARRAY,
        COLLAR_MASS_ARRAY,
        COLLAR_INERTIA_ARRAY,
        COLLAR_KA_ARRAY,
        COLLAR_KT_ARRAY,
    ) = collar_inputs(df_dict["BHA_specs"], p)

    HWDP_TOT_LEN = np.sum(L_HWDP_ARRAY)
    COLLAR_TOT_LEN = np.sum(L_COLLAR_ARRAY)
    (
        N_DP,
        L_DP_ARRAY,
        DP_MASS_ARRAY,
        DP_INERTIA_ARRAY,
        DP_KA_ARRAY,
        DP_KT_ARRAY,
    ) = drillpipe_inputs(HWDP_TOT_LEN, COLLAR_TOT_LEN, p)

    global_mass_array = np.concatenate(
        [DP_MASS_ARRAY, HWDP_MASS_ARRAY, COLLAR_MASS_ARRAY]
    )
    global_inertia_array = np.concatenate(
        [DP_INERTIA_ARRAY, HWDP_INERTIA_ARRAY, COLLAR_INERTIA_ARRAY]
    )
    global_length_array = np.concatenate([L_DP_ARRAY, L_HWDP_ARRAY, L_COLLAR_ARRAY])
    global_ka_array = np.concatenate([DP_KA_ARRAY, HWDP_KA_ARRAY, COLLAR_KA_ARRAY])
    global_kt_array = np.concatenate([DP_KT_ARRAY, HWDP_KT_ARRAY, COLLAR_KT_ARRAY])

    global_ct_array = np.where(
        global_length_array == 0,
        0,
        p[CT_BOREHOLE] / global_length_array,
    )
    global_ca_array = np.where(
        global_length_array == 0,
        0,
        p[CA_BOREHOLE] / global_length_array,
    )

    global_mass_matrix = np.diag(global_mass_array)
    global_mass_inv_array = 1 / global_mass_array
    global_inertia_inv_array = 1 / global_inertia_array
    global_ka_matrix = (
        np.diag(-1 * global_ka_array[1:], 1)
        + np.diag(-1 * global_ka_array[1:], -1)
        + np.diag(global_ka_array + np.append(global_ka_array[1:], 0))
    )

    global_inertia_matrix = np.diag(global_inertia_array)
    global_kt_matrix = (
        np.diag(-1 * global_kt_array[1:], 1)
        + np.diag(-1 * global_kt_array[1:], -1)
        + np.diag(global_kt_array + np.append(global_kt_array[1:], 0))
    )
    global_mass_inv_matrix = np.diag(global_mass_inv_array)
    global_inertia_inv_matrix = np.diag(global_inertia_inv_array)
    global_mass_inv_ka_matrix = np.matmul(global_mass_inv_matrix, global_ka_matrix)
    global_inertia_inv_kt_matrix = np.matmul(
        global_inertia_inv_matrix, global_kt_matrix
    )
    global_mass_inv_ca_array = global_mass_inv_array * global_ca_array
    global_inertia_inv_ct_array = global_inertia_inv_array * global_ct_array
    global_mass_inv_ca_matrix = np.diag(global_mass_inv_ca_array)
    global_inertia_inv_ct_matrix = np.diag(global_inertia_inv_ct_array)
    assembly_dict[NOE] = len(global_length_array)
    if 0 in list(global_length_array):
        assembly_dict[MOTOR_INDEX] = list(global_length_array).index(0)
    else:
        assembly_dict[MOTOR_INDEX] = "N"
    assembly_dict[GLOBAL_MASS_ARRAY] = global_mass_array
    assembly_dict[GLOBAL_CA_ARRAY] = global_ca_array
    assembly_dict[GLOBAL_KA_ARRAY] = global_ka_array
    assembly_dict[GLOBAL_INERTIA_ARRAY] = global_inertia_array
    assembly_dict[GLOBAL_CT_ARRAY] = global_ct_array
    assembly_dict[GLOBAL_KT_ARRAY] = global_kt_array
    assembly_dict[GLOBAL_LENGTH_ARRAY] = global_length_array
    assembly_dict[GLOBAL_MASS_INV_ARRAY] = global_mass_inv_array
    assembly_dict[GLOBAL_INERTIA_INV_ARRAY] = global_inertia_inv_array
    assembly_dict[GLOBAL_INERTIA_INV_CT_ARRAY] = global_inertia_inv_ct_array
    assembly_dict[GLOBAL_MASS_INV_CA_ARRAY] = global_mass_inv_ca_array
    assembly_dict[GLOBAL_MASS_MATRIX] = global_mass_matrix
    assembly_dict[GLOBAL_INERTIA_MATRIX] = global_inertia_matrix
    assembly_dict[GLOBAL_MASS_INV_MATRIX] = global_mass_inv_matrix
    assembly_dict[GLOBAL_INERTIA_INV_MATRIX] = global_inertia_inv_matrix
    assembly_dict[GLOBAL_KA_MATRIX] = sps.csr_matrix(global_ka_matrix)
    assembly_dict[GLOBAL_KT_MATRIX] = sps.csr_matrix(global_kt_matrix)
    assembly_dict[GLOBAL_MASS_INV_CA_MATRIX] = global_mass_inv_ca_matrix
    assembly_dict[GLOBAL_INERTIA_INV_CT_MATRIX] = global_inertia_inv_ct_matrix
    assembly_dict[GLOBAL_MASS_INV_KA_MATRIX] = sps.csr_matrix(global_mass_inv_ka_matrix)
    assembly_dict[GLOBAL_INERTIA_INV_KT_MATRIX] = sps.csr_matrix(
        global_inertia_inv_kt_matrix
    )
    return assembly_dict


def survey_interpol(
    global_length_array, global_mass_array, bf, motor_index, df_dict, noe
):
    # motor_indx_arr = np.ones(noe)
    # motor_indx_arr[motor_index] = 0

    L_discretize = np.cumsum(global_length_array)
    df_new = df_dict["well_survey"]
    x = np.array(df_new["MD [m]"].to_list())
    y = np.array(df_new["Inclination [°]"].to_list())
    y_interp = scipy.interpolate.interp1d(x, y)
    theta_inclination_deg = y_interp(L_discretize)
    # Inclination Angle in 'deg'
    theta_inclination = (np.pi / 180) * theta_inclination_deg
    if motor_index == "N":
        Normal_force = bf * (global_mass_array) * 9.81 * (np.sin(theta_inclination))
    else:
        Normal_force = bf * (global_mass_array) * 9.81 * (np.sin(theta_inclination))
        Normal_force[motor_index] = (
            Normal_force[motor_index] + Normal_force[motor_index + 1]
        )
        Normal_force[motor_index + 1] = 0
    return theta_inclination, Normal_force
