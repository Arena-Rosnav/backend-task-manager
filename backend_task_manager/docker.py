import os
import subprocess

from backend_task_manager.constants import Docker, Type
from backend_task_manager.config import config
from backend_task_manager.database import Database


def training_startup_command(user_id, task_id, robot, map):
    base_path = config["BASE_PATH"]

    return (
        f"docker run -it --rm -d --name {task_id} "
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
        f"-l {task_id} arena-rosnav ./startup/entry.sh "
        # Arguments for entrypoint
        f"{default_entrypoint_params(task_id)} "
        f"{config['FINISH_TASK_ENDPOINT']} {config['NEW_BEST_MODEL_ENDPOINT']} {robot['name']}"
    )


def evaluation_startup_command(robot_array, task):
    base_path = config["BASE_PATH"]
    evaluation_file = task.user_id + "_evaluation.yaml"
    return (
        f"docker run -it --rm -d --name {task.task_id} --net=host "
        # For the entry file
        f"-v {os.path.join(base_path, 'docker', 'evaluation')}:/root/startup "
        # For robot_schema.yaml
        f"-v {os.path.join(base_path, 'data', task.task_id, 'robot_setup')}:/root/src/arena-rosnav/task_generator/robot_setup "  
        # For robot model
        f"{robot_volume_evaluation(base_path, task.task_id, robot_array)} "
        # For Planner
        f"{planner_volume(base_path, task)} "
        # For data recording
        f"-v {os.path.join(base_path, 'data', task.user_id, task.task_id)}:/root/src/arena-evaluation/data "
        f"-l {task.task_id} arena-rosnav ./startup/entry.sh "
        # Arguments for entrypoint
        f"{default_entrypoint_params(task.task_id)} "
        f"{config['FINISH_TASK_ENDPOINT']} {evaluation_file} "
    )


def planner_volume(base_path, task):
    ret_str = ""
    for task_robot in task.robots:
        # If the planner is not a rosnav planner nothing has to be done
        if task_robot.planner.get("key") != "rosnav" and task_robot.planner.get("name").lower() != "rosnav":
            continue

        # If the planner is rosnav but not the public rosnav type and not created by
        # a specific user nothing has to be done
        if not task_robot.planner.get("userId"):
            continue

        model_base_path = os.path.join(
            base_path, 'data', task.user_id, str(task_robot.planner["fromTask"]))

        # agent name defaults to robot model name.
        # set the agents directory to the robots dir
        agent_dir = [f.path for f in os.scandir(model_base_path) if f.is_dir()][0]
        ret_str += f"-v {os.path.join(model_base_path, agent_dir)}:/root/planners/rosnav/agents/{task_robot.robot['name']} "
    return ret_str

def default_entrypoint_params(task_id):
    return f"{task_id} {config['APP_TOKEN_KEY']} {config['APP_TOKEN']} {config['API_BASE_URL']} "


def robot_volume(base_path, task_id, robot_access_type, robot_user_id):
    # If the robot is Public and no user created it, it is a default robot and
    # Therefore already included in our arena-simulation-setup repo
    if robot_access_type == Type.PUBLIC and robot_user_id == None:
        return ""

    return f"-v {os.path.join(base_path, 'data', task_id, 'robot')}:/root/src/utils/arena-simulation-setup/robot/{Docker.NAME_OF_MODEL} "

def robot_volume_evaluation(base_path, task_id, robot_array):
    # If the robot is Public and no user created it, it is a default robot and
    # Therefore already included in our arena-simulation-setup repo

    for robot in robot_array:
        if not (robot['type'] == Type.PUBLIC and robot.get('userId') == None):
            return f"-v {os.path.join(base_path, 'data', task_id, 'robot')}:/root/src/utils/arena-simulation-setup/robot/{Docker.NAME_OF_MODEL} "
    return ""
    

def map_volume(base_path, task_id, map_access_type, map_user_id):
    # If the map is Public and no user created it, it is a default map and
    # Therefore already included in our arena-simulation-setup repo
    if map_access_type == Type.PUBLIC and map_user_id == None:
        return ""

    return f"-v {os.path.join(base_path, 'data', task_id, 'maps')}:/root/src/utils/arena-simulation-setup/maps/{Docker.NAME_OF_MAP} "


def get_docker_logs(task_id, amount=100):
    return subprocess.check_output([f"docker logs {str(task_id)} | tail -n {amount}"], shell=True)


def update_task_logs(task_id):
    try:
        logs = get_docker_logs(task_id)

        Database.update_task_log(task_id, bytes.decode(logs))
    except:
        pass
