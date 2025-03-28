import json
import mujoco
from mujoco import viewer
import numpy as np
import time
import pandas as pd
from scipy.interpolate import UnivariateSpline

### Simple Simulation Viewing that replaces experiemtns run on the real testbench.
### TODO:  Convert from quick and dirty to something nice and useful


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

def run_sim_view_only(filename, test_duration=None, errorgain_file=None):
    with open(filename) as f:
        log = json.load(f)

    entries = log["entries"]
    timestamps = np.array([e["timestamp"] for e in entries])
    goal_positions = np.array([e["goal_position"] for e in entries])
    real_positions = np.array([e["position"] for e in entries])
    torque_enabled_array = np.array([e["torque_enable"] for e in entries])

    dt = log["dt"]

    error_gain_func = create_error_gain_function(errorgain_file)

    # --- Controller & Plant Parameters ---
    K_P = log.get("kp", 16)
    print(f"K_P: {K_P}")

    # TODO: store and read these from a config file
    max_torque = 5.510764878546495
    armature = 0.025896903176634425
    damping = 1.1793454779199242
    frictionloss = 0.11434146818509992

    # --- Load MuJoCo Model ---
    model = mujoco.MjModel.from_xml_path("testbed/arm_100_mass_bcd/robot.xml")
    data = mujoco.MjData(model)
    model.opt.timestep = dt
    model.opt.gravity[:] = np.array([0.0, 0.0, -9.81])

    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "sts3215_ctrl")
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "sts3215")
    joint_qposadr = model.jnt_qposadr[joint_id]
    dof_id = model.jnt_dofadr[joint_id]

    model.dof_damping[dof_id] = damping
    model.dof_frictionloss[dof_id] = frictionloss
    model.dof_armature[dof_id] = armature
    model.actuator_ctrlrange[actuator_id, :] = [-max_torque, max_torque]

    print("actuator_bias:", model.actuator_biasprm[actuator_id])
    print("actuator_gainprm:", model.actuator_gainprm[actuator_id, :])
    print("Rendering:", filename)

 
    num_steps = int(test_duration / dt) if test_duration else len(goal_positions)

    with viewer.launch_passive(model, data, show_right_ui=False) as v:
        # Set camera view (optional customization)
        v.cam.azimuth = 91.4705141129032
        v.cam.elevation = -15.762663810483877
        v.cam.distance = 0.8477356443919847
        v.cam.lookat[:] = np.array([0.0, 0.0, 0.0])

        for i in range(num_steps):
            start = time.time()

            # Clamp to last valid index if we're past the recorded trajectory
            idx = min(i, len(goal_positions) - 1)
            goal = goal_positions[idx]
            torque_enabled = torque_enabled_array[idx]

            current_pos = data.qpos[joint_qposadr]

            if torque_enabled:
                error = goal - current_pos
                duty = K_P * error_gain_func(error) * error 
                #duty = K_P * error_gain * error
                torque = np.clip(duty * max_torque, -max_torque, max_torque)
            else:
                torque = 0.0
                # disable damping and friction when torque is not enabled (trying to simulate de-energized actuator)
                model.dof_damping[dof_id] = 0.0
                model.dof_frictionloss[dof_id] = 0.0


            data.ctrl[actuator_id] = torque
            mujoco.mj_step(model, data)
            v.sync()

            # Sleep to maintain real-time pacing
            elapsed = time.time() - start
            time.sleep(max(0, dt - elapsed))

        print("azimuth:", v.cam.azimuth)
        print("elevation:", v.cam.elevation)
        print("distance:", v.cam.distance)
        print("lookat:", v.cam.lookat.tolist())


# --- Run viewer-only mode for one file with extended sim time ---
run_sim_view_only("..//sts3215-12v//processed/orig/sysid_sts3215_sin_sin_2025-03-17_172517.json",test_duration=6, errorgain_file="..//sts3215-12v/sts3215-12v_ErrorGain_Measurements.csv")
run_sim_view_only("..//sts3215-12v//processed/orig/sysid_sts3215_sin_time_square_2025-03-17_164859.json",test_duration=6, errorgain_file="..//sts3215-12v/sts3215-12v_ErrorGain_Measurements.csv")

