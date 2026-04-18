# -----------------------------------------------------------
# File: ik_core.py  
# Description: Modified IK Solver that loads configuration from YAML
#
# Author: Hari Prasanth, Yeshas Thadimari, Ng Yung Chuen
# Date Modified: 30 January 2026

import os
import casadi
import numpy as np
import pinocchio as pin
import pinocchio.casadi as cpin
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation
from casadi_optimal_control_ik.utils.ik_config_loader import IKConfig
from casadi_optimal_control_ik.utils.weighted_moving_filter import WeightedMovingFilter

from termcolor import colored


def get_geom_id_by_name(geom_model, name):
    for i, obj in enumerate(geom_model.geometryObjects):
        if obj.name == name:
            return i
    raise ValueError(f"Geometry object with name '{name}' not found. Please check if the config 'collision_pairs' is defined correctly.")

def T_mat_to_euler_angles(T_mat, sequence='zyx', in_deg=False):
    """
    Converts a 4x4 homogeneous transformation matrix to Euler angles.

    Args:
        T_mat (np.ndarray): A 4x4 numpy array representing the homogeneous
                               transformation matrix.
        sequence (str, optional): The Euler angle sequence (e.g., 'zyx', 'xyz').
                                  Defaults to 'zyx'.
        in_deg (bool, optional): If True, Euler angles are returned in degrees.
                                  Otherwise, in radians. Defaults to False.

    Returns:
        np.ndarray: A 1x3 numpy array containing the Euler angles (e.g., [yaw, pitch, roll] for 'zyx').
    """
    if T_mat.shape != (4, 4):
        raise ValueError("Input matrix must be a 4x4 homogeneous transformation matrix.")

    # Extract the 3x3 rotation matrix
    rot_mat = T_mat[:3, :3]

    # Create a Rotation object from the rotation matrix
    r = Rotation.from_matrix(rot_mat)

    # Convert to Euler angles
    euler_angles = r.as_euler(sequence, degrees=in_deg)

    return euler_angles
    
    
class RobotArmIK:
    """
    Inverse Kinematics solver for the Humanoid Robot's dual arms.
    Now configurable via YAML file.
    """
    def __init__(self, config_file_path: str = None):
        """ 
        Initializes the RobotArmIK class with configuration from YAML file.
        
        Args:
            config_file_path: Path to YAML config file. If None, uses default location.
        """
        # Load configuration
        self.config = IKConfig(config_file_path)
        
        # Initialize the solver with loaded config
        self._load_robot_models()
        self._build_reduced_model()
        self._build_collision_model()
        self._add_end_effector_frames()
        self._setup_optimization_problem()
      
        ### Sample neutral position from pinocchio or self-defined
        # self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.init_data = self.config.REDUCED_REFERENCE_CONFIGURATION
        self.smooth_filter = WeightedMovingFilter(
            self.config.SMOOTHING_FILTER_WEIGHTS, 
            self.reduced_robot.model.nq
        )
        
        print(f"RobotArmIK initialized with {self.reduced_robot.model.nq} DOF")

    def _load_robot_models(self):
        """Loads the full robot model from the URDF file using config."""
        try:
            package_share_directory = get_package_share_directory(self.config.DESCRIPTION_PACKAGE_NAME)
            urdf_path = os.path.join(package_share_directory, self.config.URDF_PATH)
            self.full_robot = pin.RobotWrapper.BuildFromURDF(urdf_path, [package_share_directory])
            print(f"Successfully loaded full robot model from {urdf_path}")
        except Exception as e:
            print(f"Failed to load full robot model: {e}")
            raise
    
    def _build_reduced_model(self):
        """Build reduced model using config-specified joints to lock."""
        # Get joint IDs from names
        joints_to_lock_ids = [
            self.full_robot.model.getJointId(jname) 
            for jname in self.config.JOINTS_TO_LOCK
        ]
        
        # Or get joint info
        if self.config.VERBOSE:
            for i, name in enumerate(self.full_robot.model.names[1:], 1):
               print(f"Joint {i}: {name}")


        ### Use pin.neutral to initial initial reference configuration
        # reference_configuration = pin.neutral(self.full_robot.model)
        reference_configuration = self.config.REFERENCE_CONFIGURATION
        self.reduced_robot = self.full_robot.buildReducedRobot(
            list_of_joints_to_lock=joints_to_lock_ids,
            reference_configuration=reference_configuration
        )

        print(f"Reduced robot created with {self.reduced_robot.model.nq} DOF.")
        print("Active joints in the reduced model:")
        self.wrist_yaw_joint_ids = []
        self.wrist_pitch_joint_ids = []
        self.shoulder_roll_ids = []
        for i, name in enumerate(self.reduced_robot.model.names[1:]):
            print(f"{i} - {name}")
            if name in self.config.WRIST_YAW_JOINTS_NAME:
                self.wrist_yaw_joint_ids.append(i)
            if name in self.config.WRIST_PITCH_JOINTS_NAME:
                self.wrist_pitch_joint_ids.append(i)
            if name in self.config.SHOULDER_ROLL_JOINTS_NAME:
                self.shoulder_roll_ids.append(i)

        if self.config.VERBOSE: 
            print("Built reduced model, frames:")
            for i, f in enumerate(self.reduced_robot.model.frames):
                print(i, f.name, f.parent)
    
    ### TEST COLLISION
    def _build_collision_model(self):
        """Build collision model between probable robot links to avoid collision in IK computation."""
        try:
            package_share_directory = get_package_share_directory(self.config.DESCRIPTION_PACKAGE_NAME)
            urdf_path = os.path.join(package_share_directory, self.config.URDF_PATH)
            self.collision_model = pin.buildGeomFromUrdf(self.reduced_robot.model, urdf_path, pin.COLLISION)
            print(f"Successfully loaded robot collision model from {urdf_path}")

            if self.config.VERBOSE: 
                print("Listing loaded collision geometry names: ")
                for i, object in enumerate(self.collision_model.geometryObjects):
                    print(f"i: {i}, name: '{object.name}', parent joint: '{self.reduced_robot.model.frames[object.parentFrame].name}'")

            ### example output (on Booster T1 humanoid robot): 
            # Listing loaded collision geometry names: 
            # i: 0, name: 'Trunk_0', parent joint: 'Trunk'
            # i: 1, name: 'H2_0', parent joint: 'H2'
            # i: 2, name: 'AL3_0', parent joint: 'AL3'
            # i: 3, name: 'AL4_0', parent joint: 'AL4'
            # i: 4, name: 'left_hand_link_0', parent joint: 'left_hand_link'
            # i: 5, name: 'T1_left_Link11_0', parent joint: 'T1_left_Link11'
            # i: 6, name: 'T1_left_Link22_0', parent joint: 'T1_left_Link22'
            # i: 7, name: 'AR3_0', parent joint: 'AR3'
            # i: 8, name: 'AR4_0', parent joint: 'AR4'
            # i: 9, name: 'right_hand_link_0', parent joint: 'right_hand_link'
            # i: 10, name: 'T1_right_Link11_0', parent joint: 'T1_right_Link11'
            # i: 11, name: 'T1_right_Link22_0', parent joint: 'T1_right_Link22'
            # i: 12, name: 'Hip_Roll_Left_0', parent joint: 'Hip_Roll_Left'
            # i: 13, name: 'Hip_Yaw_Left_0', parent joint: 'Hip_Yaw_Left'
            # i: 14, name: 'Shank_Left_0', parent joint: 'Shank_Left'
            # i: 15, name: 'left_foot_link_0', parent joint: 'left_foot_link'
            # i: 16, name: 'Hip_Roll_Right_0', parent joint: 'Hip_Roll_Right'
            # i: 17, name: 'Hip_Yaw_Right_0', parent joint: 'Hip_Yaw_Right'
            # i: 18, name: 'Shank_Right_0', parent joint: 'Shank_Right'
            # i: 19, name: 'right_foot_link_0', parent joint: 'right_foot_link'

        except Exception as e:
            print(colored(f"Failed to load robot collision model: {e}", 'red'))
            raise

        self.collision_frame_pairs = []
        self.collision_model.removeAllCollisionPairs()
        collision_pairs = self.config.LINK_COLLISION_PAIRS

        # Populate collision pairs relevant according to preset user configuration
        # geom_model.addAllCollisionPairs() # do not add all collision pairs unless running whole body
        for pair in collision_pairs:
            try:
                self.collision_model.addCollisionPair(pin.CollisionPair(get_geom_id_by_name(self.collision_model, pair[0]), get_geom_id_by_name(self.collision_model, pair[1])))
                geometry = []
                geometry.append(self.collision_model.geometryObjects[get_geom_id_by_name(self.collision_model, pair[0])].geometry)
                geometry.append(self.collision_model.geometryObjects[get_geom_id_by_name(self.collision_model, pair[1])].geometry)
                radius = 0.0
                for geom in geometry:
                    geom_type = type(geom).__name__
                    if geom_type == "Sphere":
                        radius += geom.radius
                    elif geom_type == "Box":
                        radius += max(geom.halfSide)
                    elif geom_type == "Cylinder":
                        radius += geom.halfLength
                    else:
                        # print("The geometry type is:", geom_type)
                        raise ValueError("Unable to measure radius of geometry type.")
                        
                self.collision_frame_pairs.append([self.reduced_robot.model.getFrameId(pair[0].replace('_0','')), self.reduced_robot.model.getFrameId(pair[1].replace('_0','')), radius])
            except ValueError as e:
                print(colored(f"Warning: Could not find collision geometry of {pair[0]} or {pair[1]}. Please check URDF. Skipping this pair. Error: {e}", 'red'))
        if self.config.VERBOSE: 
            print("Curated collision_frame id_pairs:", self.collision_frame_pairs)
        self.safety_dist = self.config.LINK_COLLISION_SAFETY_DIST

        if self.config.VERBOSE: 
            for pair in self.collision_model.collisionPairs:
                print("collision pair:", pair.first, 'and', pair.second)

        print("Loaded collision pairs for optimization.")

    def _add_end_effector_frames(self):
        """Add end-effector frames using config-specified offsets."""
        l_hand_joint_name = self.config.L_HAND_JOINT
        r_hand_joint_name = self.config.R_HAND_JOINT

        # Create SE3 transforms using config offsets
        ee_offset_L = pin.SE3(
            self.config.LEFT_EE_ROT_OFFSET,
            self.config.LEFT_EE_TRANS_OFFSET
        )
        ee_offset_R = pin.SE3(
            self.config.RIGHT_EE_ROT_OFFSET,
            self.config.RIGHT_EE_TRANS_OFFSET
        )

        # Add frames to model
        self.reduced_robot.model.addFrame(pin.Frame(
            self.config.L_EE_FRAME_NAME, 
            self.reduced_robot.model.getJointId(l_hand_joint_name), 
            ee_offset_L, 
            pin.FrameType.OP_FRAME
        ))
        self.reduced_robot.model.addFrame(pin.Frame(
            self.config.R_EE_FRAME_NAME, 
            self.reduced_robot.model.getJointId(r_hand_joint_name), 
            ee_offset_R, 
            pin.FrameType.OP_FRAME
        ))
        
        # Get the frame IDs for the end-effectors
        self.L_EE_FRAME_ID = self.reduced_robot.model.getFrameId(self.config.L_EE_FRAME_NAME)
        self.R_EE_FRAME_ID = self.reduced_robot.model.getFrameId(self.config.R_EE_FRAME_NAME)
        
        # Initialize the data structure for the reduced robot
        self.reduced_robot.data = self.reduced_robot.model.createData()

    def _setup_optimization_problem(self):
        """Setup CasADi optimization problem using config weights."""
        # Optimization Problem
        self.opti = casadi.Opti()

        # Symbolic variables
        cmodel = cpin.Model(self.reduced_robot.model)
        cdata = cmodel.createData()
        geom_model = self.collision_model
        geom_cdata = pin.GeometryData(geom_model)

        # Optimization Parameters - using config values
        self.param_q_offset = self.opti.parameter(self.reduced_robot.model.nq)
        self.opti.set_value(self.param_q_offset, self.config.REGULARIZATION_Q_OFFSET)
        self.var_q = self.opti.variable(self.reduced_robot.model.nq) # control signal
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)
        self.param_tf_l = self.opti.parameter(4, 4) # setpoint
        self.param_tf_r = self.opti.parameter(4, 4) # setpoint
        
        # Compute end effector error functions
        q_sym = casadi.SX.sym("q", self.reduced_robot.model.nq)
        Tf_l_sym = casadi.SX.sym("tf_l", 4, 4)
        Tf_r_sym = casadi.SX.sym("tf_r", 4, 4)
        cpin.framesForwardKinematics(cmodel, cdata, q_sym)
        cpin.updateFramePlacements(cmodel, cdata)
        L_hand_id = self.reduced_robot.model.getFrameId(self.config.L_EE_FRAME_NAME)
        R_hand_id = self.reduced_robot.model.getFrameId(self.config.R_EE_FRAME_NAME)

        # Error functions
        translational_error = casadi.Function("t_err", [q_sym, Tf_l_sym, Tf_r_sym], [casadi.vertcat(
          cdata.oMf[L_hand_id].translation - Tf_l_sym[:3,3], 
          cdata.oMf[R_hand_id].translation - Tf_r_sym[:3,3]
            )]
        )
        rotational_error = casadi.Function("r_err", [q_sym, Tf_l_sym, Tf_r_sym], [casadi.vertcat(
          cpin.log3(cdata.oMf[L_hand_id].rotation @ Tf_l_sym[:3,:3].T), 
          cpin.log3(cdata.oMf[R_hand_id].rotation @ Tf_r_sym[:3,:3].T)
            )],
        )

        # Compute self-collision distances manually
        col_dists = []
        for f1, f2, radius in self.collision_frame_pairs:
            dist = casadi.sqrt(casadi.sumsqr(cdata.oMf[f1].translation - cdata.oMf[f2].translation))
            col_dists.append(dist - radius)
        col_dists = casadi.vertcat(*col_dists)

        # CasADI MX Function
        col_dists_fn = casadi.Function("col_dists", [q_sym], [col_dists])
        col_dists_mx = col_dists_fn(self.var_q)

        # Cost functions with config weights
        translational_cost = casadi.sumsqr(translational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        rotation_cost = casadi.sumsqr(rotational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        regularization_cost = casadi.sumsqr(self.var_q - self.param_q_offset)
        smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)
        # collision_cost = casadi.sumsqr(casadi.fmax(0, self.safety_dist - col_distances))

        # Wrist yaw cost
        wrist_yaw_cost = casadi.sumsqr(self.var_q[self.wrist_yaw_joint_ids[0]]) + casadi.sumsqr(self.var_q[self.wrist_yaw_joint_ids[1]])
        
        # Wrist pitch cost
        wrist_pitch_cost = casadi.sumsqr(self.var_q[self.wrist_pitch_joint_ids[0]]) + casadi.sumsqr(self.var_q[self.wrist_pitch_joint_ids[1]])

        # Shoulder roll cost
        shoulder_roll_cost = casadi.sumsqr(self.var_q[self.shoulder_roll_ids[0]]) + casadi.sumsqr(self.var_q[self.shoulder_roll_ids[1]])

        # Total cost using config weights
        self.cost = (
            self.config.WEIGHT_TRANSLATION * translational_cost +
            self.config.WEIGHT_ROTATION * rotation_cost +
            self.config.WEIGHT_REGULARIZATION * regularization_cost +
            self.config.WEIGHT_SMOOTHNESS * smooth_cost +
            self.config.WEIGHT_WRIST_YAW * wrist_yaw_cost +
            self.config.WEIGHT_WRIST_PITCH * wrist_pitch_cost +
            self.config.WEIGHT_SHOULDER_ROLL * shoulder_roll_cost
            # self.config.WEIGHT_COLLISION * collision_cost
        )
        self.opti.minimize(self.cost)
        
        # Constraints
        self.opti.subject_to(self.opti.bounded(
          self.reduced_robot.model.lowerPositionLimit, 
          self.var_q, 
          self.reduced_robot.model.upperPositionLimit
        ))

        # Self-collision constraint
        self.opti.subject_to(self.opti.bounded(
          self.safety_dist, 
          col_dists_mx,
          casadi.inf
        ))

        # Solver with config options
        solver_opts = {
            'ipopt': {
                'print_level': self.config.loader.solver_print_level,
            }, 
            'print_time': self.config.loader.solver_print_time
        }
        self.opti.solver("ipopt", solver_opts)
        
        print("Optimization problem setup complete with config weights:")
        print(f"  Translation: {self.config.WEIGHT_TRANSLATION}")
        print(f"  Rotation: {self.config.WEIGHT_ROTATION}")
        print(f"  Regularization: {self.config.WEIGHT_REGULARIZATION}")
        print(f"  Smoothness: {self.config.WEIGHT_SMOOTHNESS}")
        print(f"  Wrist Yaw: {self.config.WEIGHT_WRIST_YAW}")
        # print(f"  Collision: {self.config.WEIGHT_COLLISION}")
    
    def solve_ik(self, left_wrist_pose, right_wrist_pose):
        """Solve IK for given target poses."""
        self.opti.set_initial(self.var_q, self.init_data)

        self.opti.set_value(self.param_tf_l, left_wrist_pose)
        self.opti.set_value(self.param_tf_r, right_wrist_pose)
        self.opti.set_value(self.var_q_last, self.init_data)

        try: # TO-DO: consider using while loop for solving until no more collision
            sol = self.opti.solve()
            sol_q = self.opti.value(self.var_q)
            if self.config.DEBUG: 
                print("Optimal cost:", sol.value(self.cost))
            # Verify collision using pinocchio outside of casadi
            col_dists = self.validate_collision(sol_q)
            if np.any(col_dists < 0.0):
                print(colored(f"Collision distances: {col_dists}", 'red'))
                print(colored("⚠️ WARNING: Collision boundaries violated!", 'red'))
        except Exception as e:
            print(colored(f"IK solver failed: {e}. Using debug value.", 'red'))
            sol_q = self.opti.debug.value(self.var_q)

        # Apply smoothing filter
        self.smooth_filter.add_data(sol_q)
        filtered_q = self.smooth_filter.filtered_data
        
        # Optional: Apply workspace limits check if configured
        if self.config.workspace_limits is not None:
            if not self._check_workspace_limits(filtered_q):
                print(colored("Warning: Solution violates workspace limits", 'red'))
        
        # Optional: Apply max joint delta check if configured  
        if self.config.max_joint_delta is not None:
            joint_delta = np.abs(filtered_q - self.init_data)
            if np.any(joint_delta > self.config.max_joint_delta):
                print(colored(f"Warning: Large joint movement detected: max delta = {np.max(joint_delta):.3f}", 'red'))
        
        self.init_data = filtered_q
        return filtered_q
    
    def _check_workspace_limits(self, q) -> bool:
        """Check if solution respects workspace limits."""
        if self.config.workspace_limits is None:
            return True
            
        # Compute forward kinematics
        pin.framesForwardKinematics(self.reduced_robot.model, self.reduced_robot.data, q)
        
        # Check left end-effector
        left_pos = self.reduced_robot.data.oMf[self.L_EE_FRAME_ID].translation
        right_pos = self.reduced_robot.data.oMf[self.R_EE_FRAME_ID].translation
        
        limits = self.config.workspace_limits
        
        for pos in [left_pos, right_pos]:
            if (pos[0] < limits['x_min'] or pos[0] > limits['x_max'] or
                pos[1] < limits['y_min'] or pos[1] > limits['y_max'] or
                pos[2] < limits['z_min'] or pos[2] > limits['z_max']):
                return False
        
        return True

    def validate_collision(self, q):
        """Full precise Pinocchio + FCL check (slow but accurate)."""
        model = self.reduced_robot.model
        data = self.reduced_robot.data
        cmodel = self.collision_model
        cdata = pin.GeometryData(cmodel)

        pin.framesForwardKinematics(model, data, q)
        pin.updateGeometryPlacements(model, data, cmodel, cdata)
        pin.computeDistances(cmodel, cdata)

        return np.array([res.min_distance for res in cdata.distanceResults])
    
    def update_config(self, new_config_file_path: str):
        """Update configuration from a new YAML file."""
        self.config = IKConfig(new_config_file_path)
        print(f"Configuration updated from {new_config_file_path}")
        # Note: You might want to reinitialize the optimization problem
        # depending on which parameters changed


# Usage example:
if __name__ == "__main__":
    # Initialize with default config
    ik_solver = RobotArmIK()
    
    print("IK Solver initialized successfully!")
