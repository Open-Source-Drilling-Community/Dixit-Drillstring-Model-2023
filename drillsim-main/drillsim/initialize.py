from drillsim.constants import *
from drillsim.utilities import LoadSheet as LoadSheet


def initialize_driver(path):

    df_dict = {}
    p = {}
    for sheet in SHEET_NAMES:
        df_dict[sheet] = pd.read_excel(
            io=path,
            sheet_name=sheet,
        )

    # --- Load Advanced Sheet and extract the variables ---#
    prop_dict = LoadSheet(df_dict["Advanced"])
    p[RUN_TIME] = prop_dict["Run Time (sec)"]
    p[TRIP_LENGTH] = prop_dict["Trip Length (m)"]
    p[CCS] = prop_dict["CCS (ksi)"]
    p[DEPTH_QUERY] = prop_dict["Analysis at depth (m)"]
    p[HOLE_DIA] = prop_dict["Hole Diameter (in)"]
    p[E] = prop_dict["Young's Modulus (Pa)"]
    p[G] = prop_dict["Bulk Modulus (Pa)"]
    p[ELEM_LENGTH] = float(prop_dict["Element length (m)"])

    # --------- Load Borehole Properties --------#
    prop_dict = LoadSheet(df_dict["Borehole_Properties"])
    p[B_F] = 1 - prop_dict["Mud weight (ppg)"] / 65.4
    p[MU_STATIC] = prop_dict["Static friction coefficient"]
    p[MU_DYNAMIC] = prop_dict["Dynamic friction coefficient"]
    p[V_CS] = prop_dict["Stribeck Critical Velocity (m/sec)"]
    p[CT_BOREHOLE] = prop_dict["Torsional Drag Coefficient (N sec/m)"]

    # --------- Load Drill Pipe Specs --------#
    prop_dict = LoadSheet(df_dict["Drill_pipe_specs"])
    p[M_DP] = 0.45 * prop_dict["Total DP Weight (lbf)"]  # mass of drill-pipe in 'kg'
    p[OD_DP] = prop_dict["OD (in)"]  # units of DP Outer diameter in 'inch'
    p[ID_DP] = prop_dict["ID (in)"]
    p[OD_TJ] = prop_dict["TJ OD (in)"]
    p[DRILLPIPE_TOT_LENGTH] = prop_dict["Total length (m)"]
    p[DRILLPIPE_DIA] = OD_DP
    p[TJOINT_DIA] = OD_TJ

    # --------- Load Top drive inputs  --------#
    prop_dict = LoadSheet(df_dict["Top_drive_inputs"])
    p[ROP_VAL_1] = float(prop_dict["Top Drive Axial Velocity Magnitude 1 (m/hr)"])
    p[ROP_VAL_2] = float(prop_dict["Top Drive Axial Velocity Magnitude 2 (m/hr)"])
    p[A1] = float(prop_dict["a1"])
    p[A2] = float(prop_dict["a2"])
    p[A3] = float(prop_dict["a3"])
    p[A4] = float(prop_dict["a4"])
    p[A5] = float(prop_dict["a5"])
    p[A6] = float(prop_dict["a6"])

    p[RPM_VAL_1] = float(prop_dict["Top Drive RPM Magnitude 1 (RPM)"])
    p[RPM_VAL_2] = float(prop_dict["Top Drive RPM Magnitude 2 (RPM)"])
    p[B1] = float(prop_dict["b1"])
    p[B2] = float(prop_dict["b2"])
    p[B3] = float(prop_dict["b3"])
    p[B4] = float(prop_dict["b4"])
    p[B5] = float(prop_dict["b5"])
    p[B6] = float(prop_dict["b6"])

    # --------- Load Heave inputs  --------#
    prop_dict = LoadSheet(df_dict["Heave_inputs"])
    p[HEAVE_STATE] = prop_dict["Heave (Y/N)"]
    p[HEAVE_AMP] = prop_dict["Heave Amplitude (m)"]
    p[HEAVE_TIME_PERIOD] = prop_dict["Heave Time Period (sec)"]
    p[HEAVE_DELAY] = prop_dict["Heave delay (sec)"]
    p[HEAVE_OMEGA] = (2 * np.pi) / p[HEAVE_TIME_PERIOD]

    # --------- Load Mud motor inputs -------- #
    prop_dict = LoadSheet(df_dict["Mud_motor_inputs"])
    p[MOTOR_STATE] = prop_dict["Motor (Y/N)"]
    p[MOTOR_RPG] = prop_dict["RPG"]
    p[MOTOR_FLOW_RATE] = prop_dict["Flow rate (gpm)"]
    p[MOTOR_TORQ_RATING] = prop_dict["Operating Torque Limit (lbf-ft)"]
    p[MOTOR_PRESSURE_RATING] = prop_dict["Operating Diff Pressure Limit (psi)"]
    p[C4_MOTOR] = p[MOTOR_PRESSURE_RATING] / p[MOTOR_TORQ_RATING]

    # --------- Load steady state inputs -------- #
    prop_dict = LoadSheet(df_dict["steady_state_inputs"])
    p[WOB_SS] = prop_dict["WOB initial (lbs)"]
    p[ROP_SS] = prop_dict["ROP steady state (m/hr)"]
    p[RPM_SS] = prop_dict["RPM Steady state"]

    p[DIA_PIPE_EQUIV] = ((27.0 * p[OD_DP]) + (3 * p[OD_TJ])) / 30
    # units of pipe diameter equivalent in 'inch'
    p[AXIAL_VEL_MULTIPLIER] = ((p[OD_DP]) ** 2) / ((p[HOLE_DIA] ** 2) - (p[OD_DP] ** 2))
    # Accounting for mud velocity drag effects along axial direction
    p[DOC_SS] = (p[ROP_SS] / p[RPM_SS]) * (1 / (60 * 0.0254))
    # units of CCS of formation in ksi

    # units of k_CCS are in '1/ksi'
    p[MU_ROCK] = -0.349 * np.log(p[CCS]) + 2.0436
    # mu_rock = -0.0201 * CCS + 1.5333
    # coefficient of friction for different rock-strength
    p[K_WOB] = (
        0.8
        * (p[CCS] * 0.5)
        * (p[WOB_SS] * 4.45)
        * (1 / (p[DOC_SS] * 0.0254))
        * (p[HOLE_DIA] / 12.25)
    )
    # units of k_WOB are in '(N-rev)/(m)'

    p[K_TQ] = (p[MU_ROCK] / 3) * (p[K_WOB] / 0.8) * (p[HOLE_DIA] * 0.0254)

    p[CA_BOREHOLE] = p[CT_BOREHOLE] * (
        ((p[ROP_SS] * p[AXIAL_VEL_MULTIPLIER]) / 3600)
        / (p[RPM_SS] / 60)
        * (0.0254 * p[DIA_PIPE_EQUIV] * np.pi)
    )
    return p, df_dict
