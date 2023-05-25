import os
import subprocess

from backend_task_manager.constants import Docker, Type
from backend_task_manager.config import config
from backend_task_manager.database import Database


def training_startup_command(user_id, task_id, robot, map):
    base_path = config["BASE_PATH"]

    return (
        # -it for interactive bash shell -d for detached mode (seperate from current terminal session)
        # f"docker run -it --rm -d --name {task_id} "
        f"docker run -it --rm --name {task_id} "
        # For the entry file
        f"-v {os.path.join(base_path, 'docker', 'training')}:/root/startup "
        # For created agent
        f"-v {os.path.join(base_path, 'data', user_id, task_id)}:/root/src/planners/rosnav/agents "
        # For robot model
        f"{robot_volume(base_path, task_id, robot['type'], robot.get('userId'))} "
        # For map
        f"{map_volume(base_path, task_id, map['type'], map.get('userId'))} "
        # For training configs
        f"-v {os.path.join(base_path, 'data', task_id, 'config', 'training_config.yaml')}:/root/src/arena-rosnav/training/configs/training_config.yaml "
        # For network
        f"{network_volume(base_path, task_id)} "
        f"-l {task_id} arena-rosnav ./startup/entry.sh "
        # Arguments for entrypoint
        f"{default_entrypoint_params(task_id)} "
        f"{config['FINISH_TASK_ENDPOINT']} {config['NEW_BEST_MODEL_ENDPOINT']} {robot['name']}"
    )


def evaluation_startup_command(user_id, task_id, robot, planner):
    base_path = config["BASE_PATH"]

    return (
        f"docker run -it --rm -d --name {task_id} --net=host "
        # For the entry file
        f"-v {os.path.join(base_path, 'docker', 'evaluation')}:/root/startup "
        # For robot model
        f"{robot_volume(base_path, task_id, robot['type'], robot.get('userId'))} "
        # For Planner
        f"{planner_volume(base_path, user_id, task_id, robot, planner)} "
        # For data recording
        f"-v {os.path.join(base_path, 'data', user_id, task_id)}:/root/src/arena-evaluation/data "
        f"-l {task_id} arena-rosnav ./startup/entry.sh "
        # Arguments for entrypoint
        f"{default_entrypoint_params(task_id)} "
        f"{config['FINISH_TASK_ENDPOINT']} {robot['name']} "
    )


def planner_volume(base_path, user_id, task_id, robot, planner):
    # If the planner is not a rosnav planner nothing has to be done
    if planner.get("key") != "rosnav" and planner.get("name").lower() != "rosnav":
        return ""

    # If the planner is rosnav but not the public rosnav type and not created by
    # a specific user nothing has to be done
    if not planner.get("userId"):
        return ""

    model_base_path = os.path.join(
        base_path, 'data', user_id, str(planner["fromTask"]))

    # agent name defaults to robot model name.
    # set the agents directory to the robots dir
    agent_dir = [f.path for f in os.scandir(model_base_path) if f.is_dir()][0]

    return f"-v {os.path.join(model_base_path, agent_dir)}:/root/planners/rosnav/agents/{robot['name']}"


def default_entrypoint_params(task_id):
    return f"{task_id} {config['APP_TOKEN_KEY']} {config['APP_TOKEN']} {config['API_BASE_URL']} "


def robot_volume(base_path, task_id, robot_access_type, robot_user_id):
    # If the robot is Public and no user created it, it is a default robot and
    # Therefore already included in our arena-simulation-setup repo
    if robot_access_type == Type.PUBLIC and robot_user_id == None:
        return ""

    return f"-v {os.path.join(base_path, 'data', task_id, 'robot')}:/root/src/utils/arena-simulation-setup/robot/{Docker.NAME_OF_MODEL} "


def map_volume(base_path, task_id, map_access_type, map_user_id):
    # If the map is Public and no user created it, it is a default map and
    # Therefore already included in our arena-simulation-setup repo
    if map_access_type == Type.PUBLIC and map_user_id == None:
        return ""

    return f"-v {os.path.join(base_path, 'data', task_id, 'maps')}:/root/src/utils/arena-simulation-setup/maps/{Docker.NAME_OF_MAP} "


def network_volume(base_path, task_id):
    # Puts created file from file_creator.py into docker container
    return f"-v {os.path.join(base_path, 'data', task_id, 'network')}:/root/planners/rosnav/agents/{Docker.NAME_OF_NETWORK}"


def get_docker_logs(task_id, amount=100):
    return subprocess.check_output([f"docker logs {str(task_id)} | tail -n {amount}"], shell=True)


def update_task_logs(task_id):
    try:
        logs = get_docker_logs(task_id)

        Database.update_task_log(task_id, bytes.decode(logs))
    except:
        pass
