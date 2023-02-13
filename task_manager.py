from pymongo.database import Database
import os
import subprocess
from colorama import Fore, Style
import signal
import traceback

from backend_task_manager.database import Database
from backend_task_manager.file_creator import FileCreator
from backend_task_manager.config import config
from backend_task_manager.constants import TaskStatus, NotificationType, ExecutableType, DownloadType
from backend_task_manager.docker import training_startup_command, evaluation_startup_command
import backend_task_manager.utils as utils
from backend_task_manager.s3 import S3


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

    ## UPLOAD DATA ##

    def upload_data(self, task):
        dir_path = os.path.join(config["BASE_PATH"], "data", task.user_id, task.task_id)
        
        print(dir_path)

        if not os.path.exists(dir_path):
            return

        print(colored(Fore.BLUE, "[UPLOAD]"), colored(Fore.CYAN, "[DATA]"), task.name)

        file_content = utils.get_dir_as_zip(dir_path)

        self.handle_upload(
            task.task_id,
            task.user_id,
            "data.zip",
            file_content,
            NotificationType.DATA_DOWNLOAD_READY,
            DownloadType.DATA
        )

    def upload_log(self, task):
        file_path = os.path.join(config["BASE_PATH"], "data", task.task_id, "output.txt")

        if not os.path.exists(file_path):
            return
        
        print(colored(Fore.BLUE, "[UPLOAD]"), colored(Fore.YELLOW, "[LOG]"), task.name)
        
        with open(file_path) as f:
            content = f.read()

        self.handle_upload(
            task.task_id,
            task.user_id,
            "log.txt",
            content,
            NotificationType.LOG_DOWNLOAD_READY,
            DownloadType.LOG
        )

    ## UTILS ##

    def start_task(self, task_id, startup_command):
        process = subprocess.Popen(
            [startup_command], 
            shell=True, 
            preexec_fn=os.setsid,
        )

        Database.start_task(task_id, { "dockerPid": process.pid })

    def handle_upload(self, task_id, user_id, file_name, content, notification_type, download_type):
        # Upload Data
        S3.upload_data(task_id, content, file_name)

        # Create Notification
        Database.insert_new_task_notification(
            task_id,
            user_id,
            notification_type
        )

        # Update Download entry
        Database.update_download(
            task_id, 
            user_id, 
            download_type
        )

    ## SCHEDULING ## 

    def schedule_new_task(self):
        scheduled_tasks = Database.get_scheduled_tasks()
        running_tasks = Database.get_running_tasks()

        for scheduled_task in scheduled_tasks:

            task = Database.get_task(scheduled_task["taskId"])

            if not task:
                return
            
            if (
                scheduled_task["type"] in [ExecutableType.START_TRAINING, ExecutableType.START_EVALUATION]
                and running_tasks >= 10
            ): 
                continue

            Database.delete_scheduled_task(scheduled_task["taskId"])

            try:
                self.multiplex_request(Task(task), scheduled_task["type"])
            except:
                pass

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
            return self.upload_data(task)
        if type == ExecutableType.UPLOAD_LOG:
            return self.upload_log(task)


if __name__ == "__main__":
    print("START TASK MANAGER")

    task_manager = TaskManager()

    task_manager.schedule_new_task()