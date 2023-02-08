from pymongo.database import Database
import rospy
import os
import subprocess
import signal

from backend_task_manager.database import Database
from backend_task_manager.msg import StartTraining, TaskId, StartEvaluation
from backend_task_manager.file_creator import FileCreator
from backend_task_manager.constants import TaskStatus, NotificationType
from backend_task_manager.docker import training_startup_command, evaluation_startup_command
import backend_task_manager.utils as utils


class TaskManager:
    def __init__(self):
        rospy.Subscriber("/training/start", StartTraining, self.start_training_callback, queue_size=10)
        rospy.Subscriber("/training/finish", TaskId, self.finish_task_callback, queue_size=10)
        rospy.Subscriber("/training/abort", TaskId, self.abort_task_callback, queue_size=10)

        rospy.Subscriber("/evaluation/start", StartEvaluation, self.start_evaluation_callback, queue_size=10)
        rospy.Subscriber("/evaluation/abort", TaskId, self.abort_task_callback, queue_size=10)
        rospy.Subscriber("/evaluation/finish", TaskId, self.finish_task_callback)

    def start_training_callback(self, data):
        rospy.loginfo("Start new training")

        # Get data from Database
        # reward = Database.get_reward_from_id(data.reward_id)
        robot = Database.get_robot_from_id(data.robot_id)
        hyperparams = Database.get_hyperparams_from_id(data.hyperparams_id)
        # network_architecture = Database.get_network_architecture_from_id(
        #      data.network_architecture_id
        # )

        ## Check if necessary data is set
        utils.check_parameters(
            # reward, 
            robot, 
            hyperparams
            # network_architecture,
        )
        
        file_creator = FileCreator(data.task_id, data.user_id)
        file_creator.create_robot_file(robot)
        file_creator.create_hyperparams_file(hyperparams)
        # file_creator.create_network_architecture_file(network_architecture)

        startup_command = training_startup_command(data.user_id, data.task_id, robot)

        print(startup_command)

        self.start_task(data.task_id, startup_command)

        Database.insert_new_task_notification(
            data.task_id, 
            data.user_id, 
            NotificationType.TRAINING_STARTED
        )

    def start_evaluation_callback(self, data):
        rospy.loginfo("Start new evaluation")

        # Get data from Database
        # reward = Database.get_reward_from_id(data.reward_id)
        robot = Database.get_robot_from_id(data.robot_id)
        planner = Database.get_planner_from_id(data.planner_id)
        # network_architecture = Database.get_network_architecture_from_id(
        #      data.network_architecture_id
        # )

        ## Check if necessary data is set
        utils.check_parameters(
            # reward, 
            robot, 
            # network_architecture,
        )
        
        file_creator = FileCreator(data.task_id, data.user_id)
        file_creator.create_robot_file(robot)
        # file_creator.create_network_architecture_file(network_architecture)

        startup_command = evaluation_startup_command(
            data.user_id, 
            data.task_id, 
            robot,
            planner
        )

        self.start_task(data.task_id, startup_command)

        Database.insert_new_task_notification(
            data.task_id, 
            data.user_id, 
            NotificationType.EVALUATION_STARTED
        )

    def finish_task_callback(self, data):
        task_id = data.task_id

        self.stop_task(task_id, TaskStatus.FINISHED)

    def abort_task_callback(self, data):
        task_id = data.task_id

        self.stop_task(task_id, TaskStatus.ABORTED)


    # UTILS


    def start_task(self, task_id, startup_command):
        process = subprocess.Popen(
            [startup_command], 
            shell=True, 
            preexec_fn=os.setsid,
        )

        Database.start_task(task_id, { "dockerPid": process.pid })

    def stop_task(self, task_id, status):
        task = Database.get_task(task_id)

        if not task or not task.get("dockerPid"):
            return False
        
        pid = task["dockerPid"]

        print("STOPPING TASK")

        subprocess.Popen([f"docker stop {task_id}"], shell=True)
        os.killpg(os.getpgid(pid), signal.SIGTERM)

        Database.update_task(
            task_id, 
            { "status": status, "updatedAt": Database.utc_now() }
        )


if __name__ == "__main__":
    rospy.init_node("training_manager")

    task_manager = TaskManager()

    while not rospy.is_shutdown():
        rospy.spin()