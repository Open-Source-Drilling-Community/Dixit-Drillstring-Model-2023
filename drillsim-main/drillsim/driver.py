from drillsim.constants import *
from drillsim.assembly import assembler_main as assembler_main
from scipy.integrate import solve_ivp as solve_ivp
from drillsim.solver import Main_Func
from drillsim.initialize import initialize_driver as initialize_driver
from drillsim.assembly import addons_BHA as addons_BHA
from drillsim.assembly import assembler_main as assembler_main
from drillsim.assembly import survey_interpol as survey_interpol
from drillsim.solver import Main_Func as Main_Func
from drillsim.visualize import visualize_vel as visualize_vel
from drillsim.visualize import visualize_mudmotor as visualize_mudmotor
import pickle


def main_driver(input_path, output_path):
    vals_dict, df_dict = initialize_driver(input_path)
    df = addons_BHA(df_dict["BHA_specs"], vals_dict)
    assembly_dict = assembler_main(df_dict, vals_dict)

    theta_inclination, Normal_force = survey_interpol(
        assembly_dict[GLOBAL_LENGTH_ARRAY],
        assembly_dict[GLOBAL_MASS_ARRAY],
        vals_dict[B_F],
        assembly_dict[MOTOR_INDEX],
        df_dict,
        assembly_dict[NOE],
    )

    temp_dict = {}
    temp_dict[THETA_INCLINATION] = theta_inclination
    temp_dict[NORMAL_FORCE] = Normal_force

    final_dict = {**vals_dict, **assembly_dict, **temp_dict}
    noe = final_dict[NOE]
    final_dict[STATIC_CHECK_PREV] = np.zeros(noe)
    final_dict[DOWNHOLE_WEIGHT] = []
    final_dict[DOWNHOLE_TORQUE] = []
    final_dict[DOC] = []
    final_dict[SOLUTION_TIME] = []

    final_dict[THETA_PREV] = [0.0]
    final_dict[HOLE_DEPTH_PREV] = [final_dict[TRIP_LENGTH]]
    final_dict[FRICTION_FORCE_STORE] = []
    final_dict[FRICTION_TORQUE_STORE] = []
    final_dict[STATIC_CHECK_PREV_STORE] = []

    sol = solve_ivp(
        lambda t, x: Main_Func(t, x, final_dict),
        [0, int(final_dict[RUN_TIME])],
        np.zeros(4 * final_dict[NOE]),
        max_step=0.001,
        rtol=1,
        atol=1,
        method="RK45",
        dense_output=True,
    )

    time_arr = sol.t
    sol_arr = sol["y"]
    final_dict[TIME_ARRAY] = time_arr
    final_dict[SOLUTION_MAIN_ARRAY] = sol_arr

    # Store important data
    visualize_vel(time_arr, sol_arr, noe, final_dict, output_path)
    if final_dict[MOTOR_INDEX] != "N":
        visualize_mudmotor(time_arr, sol_arr, noe, final_dict, output_path)
    with open(f"{output_path}/Output.pickle", "wb") as handle:
        pickle.dump(final_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)
    return None
