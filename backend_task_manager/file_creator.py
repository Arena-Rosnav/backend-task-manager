import yaml
import os
import json
import base64
from io import BytesIO
from PIL import Image

from backend_task_manager.config import (
    default_robot_values,
    default_hyperparam_values,
    config,
    default_global_costmap,
    default_local_costmap,
    default_map_values,
    default_map_world_values
)

from backend_task_manager.constants import Docker, Type


class FileCreator:
    def __init__(self, task_id, user_id):
        self.task_id = task_id
        self.user_id = user_id

        self._create_dir(self.task_id)
        self._create_dir(self.user_id)

    def _create_dir(self, id, additional_paths=""):
        try:
            os.mkdir(os.path.join(
                config["BASE_PATH"], "data", id, additional_paths))
        except:
            pass

    def _write_yaml(self, data, name, id, additional_paths=""):
        self._create_dir(id, additional_paths)

        with open(os.path.join(config["BASE_PATH"], "data", id, additional_paths, name + ".yaml"), "w") as f:
            yaml.dump(data, f)

    def _write_json(self, data, name, id):
        with open(os.path.join(config["BASE_PATH"], "data", id, name + ".json"), "w") as f:
            json.dump(data, f)

    def _write_png(self, data, id, additional_paths=""):
        img_data = base64.b64decode(data["mapImg"])

        self._create_dir(id, additional_paths)

        with open(os.path.join(config["BASE_PATH"], "data", id, additional_paths, "map.png"), "wb") as f:
            img = Image.open(BytesIO(img_data))
            img.save(f)

    def create_reward_file(self, reward_data):
        pass

    def create_robot_file(self, robot_data):
        # Create robot.model.yaml

        if robot_data["type"] == Type.PUBLIC and robot_data.get("userId") == None:
            return

        robot = {}

        bodies = robot_data["bodies"]

        for i in range(len(bodies["footprints"])):
            bodies["footprints"][i]["sensor"] = False

        robot["bodies"] = {
            **bodies,
            **default_robot_values["bodies"]
        }

        plugins = []

        plugins.append(default_robot_values["plugins"]["diff_drive"])

        laser = robot_data["laser"]

        plugins.append({
            **default_robot_values["plugins"]["laser"],
            "origin": laser["origin"],
            "range": laser["range"],
            "angle": laser["angle"],
        })

        plugins.append(default_robot_values["plugins"]["model_tf_publisher"])

        robot["plugins"] = plugins

        self._write_yaml(robot, robot["name"] +
                         ".model", self.task_id, "robot")

        # Create model_params.yaml

        model_params = {
            "robot_model": robot["name"],
            "robot_base_frame": Docker.ROBOT_BASE_FRAME,
            "robot_sensor_frame": Docker.ROBOT_SENSOR_FRAME,
            "robot_radius": robot_data["radius"],  # TODO
            "is_holonomic": robot_data["isHolonomic"],
            "actions": {
                "continuous": {
                    **FileCreator.create_angular_range_params(robot_data.get("angularRange")),
                    "angular_range": robot_data["angularRange"],
                    "linear_range": FileCreator.create_linear_range_params(
                        robot_data["linearRangeX"],
                        robot_data.get("linearRangeY")
                    )
                }
            },
            "laser": {
                "range": laser["range"],
                "angle": laser["angle"],
                "num_beams": int((laser["angle"]["max"] - laser["angle"]["min"]) / laser["angle"]["increment"]),
                "update_rate": Docker.ROBOT_LASER_UPDATE_RATE
            }
        }

        self._write_yaml(model_params, "model_params", self.task_id, "robot")

        # Create Costmaps

        self._write_yaml(default_global_costmap(
            robot_data["radius"]), "global_costmap_params", self.task_id, "robot/costmaps")
        self._write_yaml(default_local_costmap(
            robot_data["radius"]), "local_costmap_params", self.task_id, "robot/costmaps")

    def create_linear_range_params(linear_range_x, linear_range_y):
        if linear_range_y == None:
            return linear_range_x

        return {
            "x": linear_range_x,
            "y": linear_range_y
        }

    def create_angular_range_params(angular_range):
        if angular_range:
            return {
                "angular_range": angular_range
            }

        return {}

    def create_hyperparams_file(self, hyperparams):
        hyperparams_file = default_hyperparam_values

        hyperparams_file["callbacks"]["periodic_eval"]["max_num_moves_per_eps"] = hyperparams["max_num_moves_per_eps"]
        hyperparams_file["callbacks"]["periodic_eval"]["n_eval_episodes"] = hyperparams["n_eval_episodes"]
        hyperparams_file["callbacks"]["periodic_eval"]["eval_freq"] = hyperparams["eval_freq"]

        hyperparams_file["rl_agent"]["ppo"]["batch_size"] = hyperparams["batch_size"]
        hyperparams_file["rl_agent"]["ppo"]["learning_rate"] = hyperparams["learning_rate"]
        hyperparams_file["rl_agent"]["ppo"]["n_epochs"] = hyperparams["n_epochs"]

        self._write_yaml(hyperparams_file, "training_config",
                         self.task_id, "config")

    def create_network_architecture_file(self, network_architecture_data):
        self._write_json(network_architecture_data, "network_architecture")
        pass

    def create_scenario_file(self, scenario_data):
        pass

    def create_map_file(self, map_data):
        map_world_file = default_map_world_values

        self._write_yaml(map_world_file, "map.world", self.task_id, "maps")
        self._write_png(map_data, self.task_id, "maps")

        map_file = {
            **default_map_values,
            "resolution": map_data["resolution"],
            "type": map_data["type"]
        }

        self._write_yaml(map_file, "map", self.task_id, "maps")
