from pymongo.database import Database
import os
import subprocess
from colorama import Fore, Style
import signal
import traceback

from backend_task_manager.database import Database
from backend_task_manager.file_creator import FileCreator
from backend_task_manager.constants import TaskStatus, NotificationType, ExecutableType
from backend_task_manager.docker import training_startup_command, evaluation_startup_command
import backend_task_manager.utils as utils


def colored(color, text):
    return f"{color}{text}{Style.RESET_ALL}"


class Task:
    def __init__(self, task):
        self.task_id = str(task["_id"])
        self.user_id = str(task["userId"])

        self.type = str(task["type"])
        self.name = str(task["name"])

        self.status = str(task["status"])

        # Training and Eval Specific params

        self.robot_id = str(task.get("robotId"))
        self.hyperparams_id = str(task.get("hyperparamsId"))
        self.planner_id = str(task.get("plannerId"))

        # Docker Pid

        self.docker_pid = task.get("dockerPid")


class TaskManager:
    def start_training_callback(self, task):
        print(colored(Fore.GREEN, "[START]"), colored(Fore.BLUE, "[TRAINING]"), task.name)

        # Get task from Database
        # reward = Database.get_reward_from_id(task.reward_id)
        robot = Database.get_robot_from_id(task.robot_id)
        hyperparams = Database.get_hyperparams_from_id(task.hyperparams_id)
        # network_architecture = Database.get_network_architecture_from_id(
        #      task.network_architecture_id
        # )

        ## Check if necessary task is set
        utils.check_parameters(
            # reward, 
            robot, 
            hyperparams
            # network_architecture,
        )
        
        file_creator = FileCreator(task.task_id, task.user_id)
        file_creator.create_robot_file(robot)
        file_creator.create_hyperparams_file(hyperparams)
        # file_creator.create_network_architecture_file(network_architecture)

        startup_command = training_startup_command(task.user_id, task.task_id, robot)

        print(startup_command)

        self.start_task(task.task_id, startup_command)

        Database.insert_new_task_notification(
            task.task_id, 
            task.user_id, 
            NotificationType.TRAINING_STARTED
        )

    def start_evaluation(self, task):
        print(colored(Fore.GREEN, "[START]"), colored(Fore.MAGENTA, "[EVALUATION]"), task.name)

        # Get task from Database
        # reward = Database.get_reward_from_id(task.reward_id)
        robot = Database.get_robot_from_id(task.robot_id)
        planner = Database.get_planner_from_id(task.planner_id)
        # network_architecture = Database.get_network_architecture_from_id(
        #      task.network_architecture_id
        # )

        ## Check if necessary task is set
        utils.check_parameters(
            # reward, 
            robot, 
            # network_architecture,
        )
        
        file_creator = FileCreator(task.task_id, task.user_id)
        file_creator.create_robot_file(robot)
        # file_creator.create_network_architecture_file(network_architecture)

        startup_command = evaluation_startup_command(
            task.user_id, 
            task.task_id, 
            robot,
            planner
        )

        self.start_task(task.task_id, startup_command)

        Database.insert_new_task_notification(
            task.task_id, 
            task.user_id, 
            NotificationType.EVALUATION_STARTED
        )

    def stop_task(self, task, status):
        print(colored(Fore.RED, f"[{status.upper()}]"), task.name)
        
        pid = task.docker_pid

        try:
            subprocess.Popen([f"docker stop {task.task_id}"], shell=True)
            os.killpg(os.getpgid(pid), signal.SIGTERM)

            Database.update_task(
                task.task_id, 
                { "status": status, "updatedAt": Database.utc_now() }
            )
        except:
            traceback.print_exc()

    ## UTILS ##

    def start_task(self, task_id, startup_command):
        process = subprocess.Popen(
            [startup_command], 
            shell=True, 
            preexec_fn=os.setsid,
        )

        Database.start_task(task_id, { "dockerPid": process.pid })

    ## SCHEDULING ## 

    def schedule_new_task(self):
        scheduled_tasks = Database.get_scheduled_tasks()

        for scheduled_task in scheduled_tasks:

            task = Database.get_task(scheduled_task["_id"])

            Database.delete_scheduled_task(scheduled_task["_id"])

            if not task:
                return

            self.multiplex_request(Task(task), scheduled_task["type"])

    def multiplex_request(self, task, type):
        if type == ExecutableType.START_TRAINING:
            return self.start_training(task)
        if type == ExecutableType.START_EVALUATION:
            return self.start_evaluation(task)

        if type in [ExecutableType.ABORT_EVALUATION, ExecutableType.ABORT_TRAINING]:
            return self.stop_task(task, TaskStatus.ABORTED)

        if type in [ExecutableType.FINISH_EVALUATION, ExecutableType.FINISH_TRAINING]:
            return self.stop_task(task, TaskStatus.FINISHED)

        if type == ExecutableType.UPLOAD_DATA:
            return 
        if type == ExecutableType.UPLOAD_LOG:
            return


if __name__ == "__main__":
    print("START TASK MANAGER")

    task_manager = TaskManager()

    task_manager.schedule_new_task()