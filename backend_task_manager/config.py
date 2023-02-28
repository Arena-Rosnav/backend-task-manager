from dotenv import dotenv_values
import os

from backend_task_manager.constants import Docker


config = {
    **os.environ,
    **dotenv_values(".env")
}

default_robot_values = {
    "bodies": {
        "type": "dynamic",
        "name": Docker.ROBOT_BASE_FRAME
    },
    "plugins": {
        "diff_drive": {
            "type": "DiffDrive",
            "name": "diff_drive",
            "body": Docker.ROBOT_BASE_FRAME,
            "odom_frame_id": "odom",
            "odom_pub": "odom",
            "twist_sub": "cmd_vel",
            "pub_rate": 10
        },
        "laser": {
            "type": "Laser",
            "name": "static_laser",
            "frame": "laser_link",
            "topic": "scan",
            "body": Docker.ROBOT_BASE_FRAME,
            "broadcast_tf": True,
            "noise_std_dev": 0.0,
            "update_rate": 10
        },
        "model_tf_publisher": {
            "type": "ModelTfPublisher",
            "name": "tf_publisher",
            "publish_tf_world": False
        }
    }
}


def default_global_costmap(robot_radius): return {
    "global_frame": "map",
    "robot_base_frame": Docker.ROBOT_BASE_FRAME,
    "static_map": True,
    "width": 25.0,
    "height": 25.0,
    "obstacle_range": 2.5,
    "raytrace_range": 3.0,
    "footprint": [
        [robot_radius, robot_radius],
        [robot_radius, -robot_radius],
        [-robot_radius, -robot_radius],
        [-robot_radius, robot_radius]
    ],
    "footprint_padding": 0.1,
    "inflation_radius": 0.2,
    "observation_sources": "scan",
    "scan": {
        "topic": "scan",
        "sensor_frame": Docker.ROBOT_SENSOR_FRAME,
        "data_type": "LaserScan",
        "clearing": True,
        "marking": True,
        "inf_is_valid": True,
        "max_obstacle_height": 2.0,
        "min_obstacle_height": 0.0,
    }
}


def default_local_costmap(robot_radius): return {
    "global_frame": "odom",
    "robot_base_frame": Docker.ROBOT_BASE_FRAME,
    "update_frequency": 5.0,
    "publish_frequency": 5.0,
    "transform_tolerance": 0.3,
    "static_map": True,
    "rolling_window": True,
    "width": 4.0,
    "height": 4.0,
    "resolution": 0.05,
    "obstacle_range": 2.5,
    "raytrace_range": 3.0,
    "footprint": [
        [robot_radius, robot_radius],
        [robot_radius, -robot_radius],
        [-robot_radius, -robot_radius],
        [-robot_radius, robot_radius]
    ],
    "footprint_padding": 0.1,
    "inflation_radius": 0.2,
    "observation_sources": "scan",
    "scan":
    {
        "topic": "scan",
        "sensor_frame": Docker.ROBOT_SENSOR_FRAME,
        "data_type": "LaserScan",
        "clearing": True,
        "marking": True,
        "inf_is_valid": True,
        "max_obstacle_height": 2.0,
        "min_obstacle_height": 0.0,
    }
}


default_hyperparam_values = {
    "debug_mode": False,
    "n_envs": 1,
    "no_gpu": False,
    "monitoring": {
        "use_wandb": False,
        "eval_log": False
    },
    "task_mode": "staged",
    "n_timesteps": 40000000,
    "max_num_moves_per_eps": 250,

    "goal_radius": 0.3,

    "callbacks": {
        "periodic_eval": {
            "max_num_moves_per_eps": 350,
            "n_eval_episodes": 100,
            "eval_freq": 15000
        },

        "training_curriculum": {
            "training_curriculum_file": "map1small.yaml",
            "curr_stage": 1,
            "threshold_type": "succ",
            "upper_threshold": 0.8,
            "lower_threshold": 0.6
        },

        "stop_training": {
            "threshold_type": "succ",
            "threshold": 0.9
        }
    },

    "rl_agent": {
        "architecture_name": "AGENT_25",
        "reward_fnc": "rule_05",
        "resume": None,
        "discrete_action_space": False,
        "normalize": False,
        "ppo": {
            "batch_size": 9600,
            "gamma": 0.99,
            "n_steps": 1200,
            "ent_coef": 0.005,
            "learning_rate": 0.0003,
            "vf_coef": 0.22,
            "max_grad_norm": 0.5,
            "gae_lambda": 0.95,
            "m_batch_size": 20,
            "n_epochs": 3,
            "clip_range": 0.22,
        }
    }
}

default_map_world_values = {
    "properties": {
        "velocity_iterations": 10,
        "position_iterations": 10
    },
    "layers": {
        "name": "static",
        "map": "map.yaml",
        "color": [0, 1, 0, 1]
    }
}
