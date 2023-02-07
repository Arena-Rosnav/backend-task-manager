from pymongo.database import Database
import rospy
import os

from task_manager.database import Database
from task_manager.msg import StartTraining, TaskId
from task_manager.config import config
from task_manager.file_creator import FileCreator
from task_manager.constants import TaskType
from task_manager.task_manager import TaskManager
from task_manager.docker import training_startup_command
import task_manager.utils as utils


class TrainingManager:
    def __init__(self):
        rospy.Subscriber("/training/start", StartTraining, self.start_training_callback, queue_size=10)
        rospy.Subscriber("/training/finish", TaskId, self.finish_training_callback, queue_size=10)
        rospy.Subscriber("/training/abort", TaskId, self.abort_training_callback, queue_size=10)

        self.task_manager = TaskManager(TaskType.TRAINING)

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

        # NOT USER ID BUT TASK ID

        self.task_manager.add_new_task(
            data.task_id,
            data.user_id, 
            data.name,
            {
                "robotId": data.robot_id,
                "hyperparamsId": data.hyperparams_id
            }
        )
        
        file_creator = FileCreator(data.task_id, data.user_id)
        file_creator.create_robot_file(robot)
        file_creator.create_hyperparams_file(hyperparams)
        # file_creator.create_network_architecture_file(network_architecture)

        startup_command = training_startup_command(data.user_id, data.task_id, robot)

        self.task_manager.start_task(data.task_id, data.user_id, startup_command)

    def finish_training_callback(self, data):
        task_id = data.task_id

        self.task_manager.finish_task(task_id)

    def abort_training_callback(self, data):
        task_id = data.task_id

        self.task_manager.abort_task(task_id)

    def terminate_all_processes(self):
        self.task_manager.terminate_all_tasks()


if __name__ == "__main__":
    rospy.init_node("training_manager")

    task_manager = TrainingManager()

    while not rospy.is_shutdown():
        rospy.spin()

    task_manager.terminate_all_processes()