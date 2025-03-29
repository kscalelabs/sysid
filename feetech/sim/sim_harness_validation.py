import json
import mujoco
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from pathlib import Path
import argparse
import pandas as pd
from scipy.interpolate import UnivariateSpline

########################################################################################    
############## This is a temporary utility, please do not judge too harshly ##############
########################################################################################

def create_error_gain_function(csv_file):
    """
    Reads error gain data from a CSV file, fits a cubic spline to the data,
    and returns a function that calculates the error gain for a given position error in radians.
    
    Parameters:
        csv_file (str): Path to the CSV file containing the data.
        
    Returns:
        function: A function that takes a position error in radians (scalar or array-like)
                  and returns the corresponding error gain. If the position error is outside the 
                  domain of the dataset, the function returns the saturation value (min or max error gain).
    """
    # Load CSV file with extra spaces in header handled
    data = pd.read_csv(csv_file, skipinitialspace=True)
    x = data['pos_err'].values  # position error in radians
    y = data['error_gain'].values

    # Sort the data by x (required for spline fitting)
    sorted_indices = np.argsort(x)
    x = x[sorted_indices]
    y = y[sorted_indices]

    # Fit a cubic spline (k=3) with s=0 to interpolate the data exactly.
    spline = UnivariateSpline(x, y, s=0, k=3)

    def error_gain_radians(radian_value):
        """
        Returns the error gain for the provided position error in radians.
        
        Parameters:
            radian_value (float or array-like): The position error in radians.
            
        Returns:
            float or np.ndarray: The error gain computed by the spline. If the input is beyond the
            domain of the data, the function returns the corresponding saturation value.
        """
        # Convert the input to a NumPy array (if it isn't already)
        radian_value = np.asarray(radian_value)
        
        # Determine the bounds of the dataset
        x_min = x.min()
        x_max = x.max()
        
        # Compute the spline value normally
        spline_val = spline(radian_value)
        
        # Saturate the spline output: if radian_value is less than x_min or greater than x_max,
        # return the corresponding min or max error_gain from the dataset.
        saturated = np.where(radian_value < x_min, y[0], spline_val)
        saturated = np.where(radian_value > x_max, y[-1], saturated)
        
        # If the input was scalar, return a scalar.
        if saturated.shape == ():
            return saturated.item()
        return saturated
    
    #print("Not Saturated: !")
    return error_gain_radians



def run_sim(log, errorgain_file):
    """
    Run simulation based on a given log dictionary.
    Returns simulation timestamps, simulated positions, real positions,
    goal positions, simulated torque commands, RMSE, and the kp value.
    """
    error_gain = create_error_gain_function(errorgain_file)

    entries = log["entries"]
    timestamps = np.array([e["timestamp"] for e in entries])
    goal_positions = np.array([e["goal_position"] for e in entries])
    real_positions = np.array([e["position"] for e in entries])
    torque_enabled_array = np.array([e["torque_enable"] for e in entries])
    dt = log["dt"]

    # --- Controller & Plant Parameters ---
    K_P = log.get("kp", 16)  # Extract kp from the log file
    print(f"K_P: {K_P}")
    """
    Parameters export for MuJoCo, actuator sts3250
    - forcerange: 8.716130441407099
    - armature: 0.03999977737144798
    - kp: 45.53297645165114
    - damping: 1.3464038511725651
    - frictionloss: 0.19999504581400715


    Parameters export for MuJoCo, actuator sts3215_12v
    - forcerange: 5.510764878546495
    - armature: 0.025896903176634425
    - kp: 29.07744066635301
    - damping: 1.1793454779199242
    - frictionloss: 0.11434146818509992
    - error_gain: 0.164787755
    """

    
    # TODO: store and read these from a config file
    
    #3250
    #max_torque = 8.716130441407099
    #armature = 0.03999977737144798
    #damping = 1.3464038511725651
    #frictionloss = 0.19999504581400715
    #max_velocity = 8.93762009169868
    #max_pwm=0.99981
    #vin = 12.1
    #kt = 1.0005874626213263
    #R = 1.3890462492623645
    

    #3215    
    max_torque = 5.466091040935576  
    armature = 0.039999999991812
    damping = 1.2305092028680242
    frictionloss = 0.16197241014297026
    max_velocity = 4.855763772519988
    max_pwm=0.99981
    vin = 12.1
    kt =  1.0000000244338463
    R = 2.2136477795617733

    # --- Determine the proper model file based on mass and length ---
    # TODO: Hacks, need to fix current data set (no arm_mass)
    mass_val = float(log["mass"])
    length_val = float(log["length"])
    if length_val == 0.1:
        arm_mass_val = 0.036
    elif length_val == 0.15:
        arm_mass_val = 0.050
    #arm_mass_val = #float(log["arm_mass"])
    # Multiply by 1000 and format as 3-digit integers:
    mass_str = f"{int(round((mass_val + arm_mass_val) * 1000)):03d}"
    print(f"mass_str: {mass_str}")
    
    # TODO: Hacks, need to add proper metadata to dataset to avoid this
    if mass_str == "1204":
        mass_str = "bcd"
    elif mass_str == "777":
        mass_str = "cd"
    elif mass_str == "1005":
        mass_str = "bc"
    elif mass_str == "571":
        mass_str = "b"
    elif mass_str == "585":
        mass_str = "b"
    elif mass_str == "791":
        mass_str = "cd"
    elif mass_str == "1019":
        mass_str = "bc"
    elif mass_str == "1218":
        mass_str = "bcd"
        
    length_str = f"{int(round(length_val * 1000)):03d}"
    model_file = f"testbed/arm_{length_str}_mass_{mass_str}/robot.xml"
    

    
    # --- Load MuJoCo Model ---
    model = mujoco.MjModel.from_xml_path(model_file)
    data = mujoco.MjData(model)
    model.opt.timestep = dt

    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "sts3215_ctrl")
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "sts3215")
    joint_qposadr = model.jnt_qposadr[joint_id]
    dof_id = model.jnt_dofadr[joint_id]

    model.dof_damping[dof_id] = damping
    model.dof_frictionloss[dof_id] = frictionloss
    model.dof_armature[dof_id] = armature
    model.actuator_ctrlrange[actuator_id, :] = [-max_torque, max_torque]

    sim_positions = []
    sim_ctrls = []
    sim_volts = []

    # --- Run Simulation Loop ---
    for t, qtarget, torque_enabled in zip(timestamps, goal_positions, torque_enabled_array):
        q = data.qpos[joint_qposadr]
        current_vel = data.qvel[dof_id]

        qtarget_smooth = np.clip(
            qtarget,
            qtarget - max_velocity * dt,
            qtarget + max_velocity * dt,
        )

        if torque_enabled:
            error = qtarget_smooth - q
            duty = np.clip(K_P * error_gain(error) * error, -max_pwm, max_pwm)
            volts = duty * vin
            torque = volts * kt / R

            sim_volts.append(volts)           
        else:
            # TODO: This might not be super interesting, we would never operate a robot with servo disabled ?? right??
            torque = 0.0
            sim_volts.append(0.0)
            # disable damping and friction when torque is not enabled (trying to simulate de-energized actuator)
            model.dof_damping[dof_id] = 0.0
            model.dof_frictionloss[dof_id] = 0.0

        data.ctrl[actuator_id] = torque
        mujoco.mj_step(model, data)

        sim_positions.append(data.qpos[joint_qposadr])
        sim_ctrls.append(torque)

    sim_positions = np.array(sim_positions)
    rmse = np.sqrt(np.mean((sim_positions - real_positions) ** 2))

    #return timestamps, sim_positions, real_positions, goal_positions, sim_ctrls, rmse, K_P
    return timestamps, sim_positions, real_positions, goal_positions, sim_ctrls, sim_volts, rmse, K_P


def create_subplot(log, axes, show_kp=False, kp=None, errorgain_file=None):
    """
    Helper to create a group of three subplots in a single cell of the grouped grid.
    It runs the simulation for a given log (already loaded) and plots:
      - Positions (simulated, real, and goal) in axes[0]
      - Velocities (simulated velocity and an approximation of the real velocity) in axes[1]
      - Voltage control output in axes[2]
    
    Parameters:
        log (dict): The simulation log dictionary.
        axes (list): List of three matplotlib axes objects.
        show_kp (bool): Whether to show the kp value in the title.
        kp (float): The kp value to show if show_kp is True.
        errorgain_file (str): Path to the CSV file with error gain data.
    """
    # Now unpacking 9 values from run_sim:
    t, sim_pos, real_pos, goal_pos, sim_ctrls, sim_volts, rmse, _ = run_sim(log, errorgain_file)
    
    # --- Positions subplot ---
    axes[0].plot(t, sim_pos, label="Sim Pos", linewidth=0.5)
    axes[0].plot(t, real_pos, '--', label="Real Pos", linewidth=0.5)
    axes[0].plot(t, goal_pos, ':', label="Goal Pos", linewidth=0.5)
    axes[0].set_title(f'Positions (kp={kp})' if show_kp else "Positions", fontsize=8)
    axes[0].tick_params(axis="both", which="major", labelsize=6)
    axes[0].grid(True, linewidth=0.5)
    axes[0].legend(fontsize=6, loc="upper right")

    # --- Voltage Control Output subplot ---
    axes[1].plot(t, sim_volts, label="Sim Voltage", linewidth=0.5)
    axes[1].set_title("Voltage Control Output", fontsize=8)
    axes[1].tick_params(axis="both", which="major", labelsize=6)
    axes[1].grid(True, linewidth=0.5)
    axes[1].legend(fontsize=6, loc="upper right")




def group_logs_by_params(logs):
    """
    Group logs by length, trajectory, mass, and kp with up to 4 repetitions per (mass, kp) combo.
    Returns a nested dictionary structured as:
    
      final_groups[length][trajectory][rep][mass][kp] = log
    """
    grouped = {}
    for log in logs:
        if log["trajectory"] == "lift_and_drop":
            print(f"Skipping {log['filename']} due to trajectory filter")
            continue
        try:
            length_val = float(log["length"])
            traj = log["trajectory"]
            mass_val = float(log["mass"])
            kp_val = float(log["kp"])
        except KeyError:
            continue  # Skip logs missing any of these keys
        
        if length_val not in grouped:
            grouped[length_val] = {}
        if traj not in grouped[length_val]:
            grouped[length_val][traj] = {}
        combo_key = (mass_val, kp_val)
        if combo_key not in grouped[length_val][traj]:
            grouped[length_val][traj][combo_key] = []
        # Limit to 4 repetitions per combination
        if len(grouped[length_val][traj][combo_key]) < 4:
            grouped[length_val][traj][combo_key].append(log)
    
    # Reorganize into final structure:
    final_groups = {}
    for length_val in grouped:
        final_groups[length_val] = {}
        for traj in grouped[length_val]:
            final_groups[length_val][traj] = {}
            # Determine the union of all mass and kp values
            all_masses = sorted({mass for (mass, _) in grouped[length_val][traj].keys()})
            all_kps = sorted({kp for (_, kp) in grouped[length_val][traj].keys()})
            # Determine maximum number of repetitions
            max_reps = max(len(lst) for lst in grouped[length_val][traj].values())
            rep_limit = min(4, max_reps)  # up to 4 reps
            for rep in range(rep_limit):
                final_groups[length_val][traj][rep] = {}
                for mass_val in all_masses:
                    final_groups[length_val][traj][rep][mass_val] = {}
                    for kp_val in all_kps:
                        combo_key = (mass_val, kp_val)
                        if combo_key in grouped[length_val][traj] and rep < len(grouped[length_val][traj][combo_key]):
                            final_groups[length_val][traj][rep][mass_val][kp_val] = grouped[length_val][traj][combo_key][rep]
    return final_groups



def build_grouped_pdf_plotbook(logdir, errorgain_file, output_pdf="sim_plotbook.pdf", mass=None, length=None):
    """
    Grouped mode: Load all JSON files (optionally filtered by mass and length),
    group them by length, trajectory, mass, and kp with up to 4 repetitions per (mass, kp) combo,
    and create pages (one per trajectory type and repetition) where rows correspond to mass values
    and columns to kp values. Each cell now displays two subplots:
      - Positions and Voltage Control Output.
    
    Parameters:
        logdir (str): Directory containing trajectory JSON files.
        errorgain_file (str): CSV file containing error gain data.
        output_pdf (str): Output PDF file name.
        mass (float, optional): Filter logs by mass.
        length (float, optional): Filter logs by length.
    """
    logs_list = []
    path = Path(logdir)
    json_files = sorted(list(path.glob("*.json")))
    if not json_files:
        print("No JSON files found in the directory:", logdir)
        return

    for json_file in json_files:
        with open(json_file) as f:
            log = json.load(f)
        # Filter based on mass and length if provided
        if mass is not None:
            if "mass" not in log or float(log["mass"]) != mass:
                print(f"Skipping {json_file} due to mass filter (expected {mass}, got {log.get('mass', None)})")
                continue
        if length is not None:
            if "length" not in log or float(log["length"]) != length:
                print(f"Skipping {json_file} due to length filter (expected {length}, got {log.get('length', None)})")
                continue
        # Save filename for reference if not already present
        if "filename" not in log:
            log["filename"] = json_file.name
        logs_list.append(log)

    grouped_logs = group_logs_by_params(logs_list)
    pdf = PdfPages(output_pdf)

    # Iterate over each (length, trajectory) group
    for length_val in sorted(grouped_logs.keys()):
        for traj in sorted(grouped_logs[length_val].keys()):
            reps_dict = grouped_logs[length_val][traj]
            # Compute the union of masses and kps across all repetitions for a consistent grid
            all_masses = sorted({mass_val for rep in reps_dict for mass_val in reps_dict[rep].keys()})
            all_kps = sorted({kp_val for rep in reps_dict for mass_val in reps_dict[rep] for kp_val in reps_dict[rep][mass_val].keys()})
            rows = len(all_masses)
            cols = len(all_kps)
            
            # Set fixed dimensions: 4 inches per column, 3 inches per row,
            # and multiply height by 2 since each cell now has 2 subplots.
            fig_width = cols * 4
            fig_height = rows * 3 * 2
            
            for rep in sorted(reps_dict.keys()):
                fig = plt.figure(figsize=(fig_width, fig_height))
                page_title = f'Length: {length_val:.2f} m, Trajectory: {traj}\nRepetition: {rep}'
                fig.suptitle(page_title, fontsize=16)
                
                # Create a subplot grid for this repetition
                for i, mass_val in enumerate(all_masses):
                    for j, kp_val in enumerate(all_kps):
                        # Create two subplots per cell: positions and voltage control output.
                        ax_pos = fig.add_subplot(rows * 2, cols, i * 2 * cols + j + 1)
                        ax_volt = fig.add_subplot(rows * 2, cols, i * 2 * cols + j + 1 + cols)
                        
                        log_entry = reps_dict[rep].get(mass_val, {}).get(kp_val, None)
                        if log_entry is not None:
                            # Pass the two axes as a list to create_subplot.
                            create_subplot(log_entry, [ax_pos, ax_volt],
                                           show_kp=(i == 0), kp=kp_val, errorgain_file=errorgain_file)
                        else:
                            ax_pos.set_title(f'kp={kp_val}\nN/A', fontsize=8)
                        
                        if j == 0:
                            ax_pos.set_ylabel(f'm={mass_val:.4f}', fontsize=8)
                        if rep == max(reps_dict.keys()):
                            ax_volt.set_xlabel('Time (s)', fontsize=8)
                        for a in [ax_pos, ax_volt]:
                            a.tick_params(axis='both', which='major', labelsize=6)
                            a.grid(True, linewidth=0.5)
                fig.tight_layout(rect=[0, 0.03, 1, 0.95])
                pdf.savefig(fig)
                plt.close(fig)

    pdf.close()
    print(f"Grouped PDF plotbook saved to {output_pdf}")




def main():
    parser = argparse.ArgumentParser(
        description="Modular simulation harness and PDF plotbook generator"
    )
    parser.add_argument("--logdir", type=str, required=True,
                        help="Directory containing trajectory JSON files")
    parser.add_argument("--errorgain_data", type=str, required=True,
                        help="csv file containing error gain data")
    parser.add_argument("--output", type=str, default="sim_plotbook.pdf",
                        help="Output PDF file name")

    args = parser.parse_args()
    build_grouped_pdf_plotbook(args.logdir, args.errorgain_data, args.output)

if __name__ == "__main__":
    main()
