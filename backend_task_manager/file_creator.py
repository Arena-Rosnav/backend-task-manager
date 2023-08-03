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
    default_map_world_values,
    default_scenario_values
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

  def _write_json(self, data, name, id, additional_paths=""):
    self._create_dir(id, additional_paths)
    
    with open(os.path.join(config["BASE_PATH"], "data", id, additional_paths, name + ".json"), "w") as f:
      json.dump(data, f)

  def _write_png(self, data, id, additional_paths=""):
    img_data = base64.b64decode(data["mapImg"])

    self._create_dir(id, additional_paths)

    with open(os.path.join(config["BASE_PATH"], "data", id, additional_paths, "map.png"), "wb") as f:
      img = Image.open(BytesIO(img_data))
      img.save(f)

  def create_reward_file(self, reward_data):
    pass

    def create_evaluation_file(self, evaluation_data):
        # Create robot_setup
        model_params = {"robots": []}
        for evaluation_robot in evaluation_data:
            new_robot = {
                "model": evaluation_robot.robot["name"], 
                "planner": evaluation_robot.planner["key"],
                "amount": evaluation_robot.amount
            }
            if evaluation_robot.planner["key"] == "rosnav":
                new_robot["agent"] = evaluation_robot.robot["name"]

            model_params["robots"].append(new_robot)
        self._write_yaml(model_params, self.user_id + "_evaluation", self.task_id, "robot_setup")

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

  def create_scenario_file(self, scenario_data, map_data):
    robot = scenario_data["robot"]

    scenario_file = {
      "robots": [{
        "start": FileCreator.get_array_from_coord(robot[0], map_data["resolution"], map_data["height"]),
        "goal": FileCreator.get_array_from_coord(robot[1], map_data["resolution"], map_data["height"])
      }],
      "obstacles": {
        "dynamic": [],
        "static": []
      },
      ** default_scenario_values
    }

    self._write_json(scenario_file, Docker.NAME_OF_SCENARIO, self.task_id, "scenarios")

  @staticmethod
  def get_array_from_coord(coord_dict, map_resolution, map_height):
    return [coord_dict["x"] * map_resolution, abs(map_height - coord_dict["y"]) * map_resolution, 0]

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

  def create_plotting_file(self, plotting_data):
    # TODO: create plotting yaml

    # with open('/docker/plotting/plotting_data_template.yaml') as f:
    #   config_yaml = yaml.safe_load(f)
    #   config_yaml["show_plots"] = False
    #   config_yaml["save_location"] = "plotting_2023"
    #   config_yaml["datasets"] = plotting_data["datasets"]

    #   config_yaml["results"]["plot"] = plotting_data["results"]["plot"]
    #   config_yaml["x"] = plotting_data["x"]

    # self._write_yaml(config_yaml, "plot", self.task_id, "plots")

    config_data = {
        # 'show_plots': plotting_data.show_plots,
        'show_plots': False,
        # 'save_location': plotting_data.save_location,
        # TODO task ID
        'save_location': Docker.SAVE_LOCATION_PLOT,
        'datasets': plotting_data.datasets,
        'results': {
            'plot': plotting_data.results.plot,
            'title': plotting_data.results.title,
            'save_name': plotting_data.results.save_name,
            'differentiate': plotting_data.results.differentiate,
            'plot_args': plotting_data.results.plot_args
        },
        'single_episode_line': [],
        'single_episode_distribution': [],
        'aggregated_distribution': [],
        'aggregated_line': [],
        'all_episodes_categorical': [],
        'all_episodes_distribution': [],
        'episode_plots_for_namespaces': {
            'desired_results': plotting_data.episode_plots_for_namespaces.desired_results,
            'should_add_obstacles': plotting_data.episode_plots_for_namespaces.should_add_obstacles,
            'should_add_collisions': plotting_data.episode_plots_for_namespaces.should_add_collisions,
            'title': plotting_data.episode_plots_for_namespaces.title,
            'save_name': plotting_data.episode_plots_for_namespaces.save_name
        },
        'create_best_plots': {
            'should_add_obstacles': plotting_data.create_best_plots.should_add_obstacles,
            'should_add_collisions': plotting_data.create_best_plots.should_add_collisions,
            'title': plotting_data.create_best_plots.title,
            'save_name': plotting_data.create_best_plots.save_name
        }
    }

    for item in plotting_data.single_episode_line:
      line_plot = {
          'data_key': item.data_key,
          'step_size': item.step_size,
          'differentiate': item.differentiate,
          'episode': item.episode,
          'title': item.title,
          'save_name': item.save_name,
          'plot_args': item.plot_args
      }
      config_data['single_episode_line'].append(line_plot)

    for item in plotting_data.single_episode_distribution:
      distribution_plot = {
          'data_key': item.data_key,
          'episode': item.episode,
          'differentiate': item.differentiate,
          'plot_key': item.plot_key,
          'title': item.title,
          'save_name': item.save_name,
          'plot_args': item.plot_args
      }
      config_data['single_episode_distribution'].append(distribution_plot)

    for item in plotting_data.aggregated_distribution:
      aggregated_dist_plot = {
          'data_key': item.data_key,
          'differentiate': item.differentiate,
          'aggregate': item.aggregate,
          'plot_key': item.plot_key,
          'title': item.title,
          'save_name': item.save_name,
          'plot_args': item.plot_args
      }
      config_data['aggregated_distribution'].append(aggregated_dist_plot)

    for item in plotting_data.aggregated_line:
      aggregated_line_plot = {
          'data_key': item.data_key,
          'differentiate': item.differentiate,
          'aggregate': item.aggregate,
          'title': item.title,
          'save_name': item.save_name,
          'plot_args': item.plot_args
      }
      config_data['aggregated_line'].append(aggregated_line_plot)

    for item in plotting_data.all_episodes_categorical:
      all_episodes_cat_plot = {
          'data_key': item.data_key,
          'differentiate': item.differentiate,
          'plot_key': item.plot_key,
          'title': item.title,
          'save_name': item.save_name,
          'plot_args': item.plot_args
      }
      config_data['all_episodes_categorical'].append(all_episodes_cat_plot)

    for item in plotting_data.all_episodes_distribution:
      all_episodes_dist_plot = {
          'data_key': item.data_key,
          'differentiate': item.differentiate,
          'plot_key': item.plot_key,
          'title': item.title,
          'save_name': item.save_name,
          'plot_args': item.plot_args
      }
      config_data['all_episodes_distribution'].append(all_episodes_dist_plot)

    self._write_yaml(config_data, "plot_declaration", self.task_id, "plot")
    # with open(os.path.join(config["BASE_PATH"] + "/docker/plotting/plotting_data.yaml"), "w") as config_file:
    #   yaml.dump(config_data, config_file)
