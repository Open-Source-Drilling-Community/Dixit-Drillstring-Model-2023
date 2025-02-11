import numpy as np
import pandas as pd

WELL_SURVEY = "well_survey"
BHA_SPECS = "BHA_specs"
DRILL_PIPE_SPECS = "Drill_pipe_specs"
BOREHOLE_PROPERTIES = "Borehole_Properties"
TOP_DRIVE_INPUTS = "Top_drive_inputs"
HEAVE_INPUTS = "Heave_inputs"
MUD_MOTOR_INPUTS = "Mud_motor_inputs"
STEADY_STATE_INPUTS = "steady_state_inputs"
ADVANCED = "Advanced"

# conversion factors
CONV_1 = 3600
CONV_2 = (2 * np.pi) / 60
CONV_3 = 0.0254

SHEET_NAMES = [
    ADVANCED,
    WELL_SURVEY,
    BHA_SPECS,
    DRILL_PIPE_SPECS,
    BOREHOLE_PROPERTIES,
    TOP_DRIVE_INPUTS,
    HEAVE_INPUTS,
    MUD_MOTOR_INPUTS,
    STEADY_STATE_INPUTS,
]

RUN_TIME = "run_time"
TRIP_LENGTH = "trip_length"
CCS = "ccs"
DEPTH_QUERY = "depth_query"
HOLE_DIA = "hole_dia"
E = "E"
G = "G"
ELEM_LENGTH = "elem_length"
B_F = "buoyancy_factor"
MU_STATIC = "mu_static"
MU_DYNAMIC = "mu_dynamic"
V_CS = "v_cs"
CT_BOREHOLE = "ct_borehole"
M_DP = "Mass of drill pipe"
OD_DP = "drill_pipe_od"
ID_DP = "drill_pipe_id"
OD_TJ = "tool_joint_outer_dia"
DRILLPIPE_TOT_LENGTH = "drill pipe total length"
DRILLPIPE_DIA = "drill pipe diameter"
DRILLPIPE_DIA = "DRILL PIPE DIA"
TJOINT_DIA = "tj dia"
# DRILLPIPE_LENGTH = "drill_pipe_length"
ROP_VAL_1 = "rop val 1"
ROP_VAL_2 = "rop val 2"
RPM_VAL_1 = "rpm_val_1"
RPM_VAL_2 = "rpm_val_1"

A1 = "a1"
A2 = "a2"
A3 = "a3"
A4 = "a4"
A5 = "a5"
A6 = "a6"

B1 = "b1"
B2 = "b2"
B3 = "b3"
B4 = "b4"
B5 = "b5"
B6 = "b6"

HEAVE_STATE = "HEAVE_STATE"
HEAVE_AMP = "HEAVE_AMP"
HEAVE_TIME_PERIOD = "HEAVE_TIME_PERIOD"
HEAVE_DELAY = "HEAVE_DELAY"
HEAVE_OMEGA = "HEAVE_OMEGA"

MOTOR_STATE = "MOTOR_STATE"
MOTOR_RPG = "MOTOR_RPG"
MOTOR_FLOW_RATE = "MOTOR_FLOW_RATE"
C4_MOTOR = "c4_motor"
MOTOR_TORQ_RATING = "MotorTorqRating"
MOTOR_PRESSURE_RATING = "MotorPressureRating"

WOB_SS = "WOB_SS"
ROP_SS = "ROP_SS"
RPM_SS = "RPM_SS"

DIA_PIPE_EQUIV = "DIA_PIPE_EQUIV"
AXIAL_VEL_MULTIPLIER = "AXIAL_VEL_MULTIPLIER"

DOC_SS = "DOC_SS"

MU_ROCK = "MU_ROCK"

K_WOB = "K_WOB"

K_TQ = "K_TQ"
CA_BOREHOLE = "CA_BOREHOLE"


GLOBAL_MASS_ARRAY = "global_mass_array"
GLOBAL_CA_ARRAY = "global_ca_array"
GLOBAL_KA_ARRAY = "global_ka_array"
GLOBAL_INERTIA_ARRAY = "global_inertia_array"
GLOBAL_CT_ARRAY = "global_ct_array"
GLOBAL_KT_ARRAY = "global_kt_array"
GLOBAL_LENGTH_ARRAY = "global_length_array"
GLOBAL_MASS_INV_ARRAY = "global_mass_inv_array"
GLOBAL_INERTIA_INV_ARRAY = "global_inertia_inv_array"
GLOBAL_INERTIA_INV_CT_ARRAY = "global_inertia_inv_ct_array"
GLOBAL_MASS_INV_CA_ARRAY = "global_mass_inv_ca_array"
GLOBAL_MASS_MATRIX = "global_mass_matrix"
GLOBAL_INERTIA_MATRIX = "global_inertia_matrix"
GLOBAL_MASS_INV_MATRIX = "global_mass_inv_matrix"
GLOBAL_INERTIA_INV_MATRIX = "global_inertia_inv_matrix"
GLOBAL_KA_MATRIX = "global_ka_matrix"
GLOBAL_KT_MATRIX = "global_kt_matrix"
GLOBAL_MASS_INV_CA_MATRIX = "global_mass_inv_ca_matrix"
GLOBAL_INERTIA_INV_CT_MATRIX = "global_inertia_inv_ct_matrix"
GLOBAL_MASS_INV_KA_MATRIX = "global_mass_inv_ka_matrix"
GLOBAL_INERTIA_INV_KT_MATRIX = "global_inertia_inv_kt_matrix"

NOE = "noe"
MOTOR_INDEX = "motor_index"

THETA_INCLINATION = "theta_inclination"
NORMAL_FORCE = "normal_force"
STATIC_CHECK_PREV = "static_check_prev"


DOWNHOLE_WEIGHT = "downhole_weight"
DOWNHOLE_TORQUE = "downhole_torque"
DOC = "doc"
SOLUTION_TIME = "solution_time"


THETA_PREV = "theta_previous"
HOLE_DEPTH_PREV = "hole_depth_previous"

FRICTION_FORCE_STORE = "friction_force_store"
FRICTION_TORQUE_STORE = "friction_torque_store"

STATIC_CHECK_PREV_STORE = "static_prev_check_store"
TIME_ARRAY = "time_array"
SOLUTION_MAIN_ARRAY = "solution_numpy_array"
