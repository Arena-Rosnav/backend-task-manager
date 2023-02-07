from pymongo.database import Database
import rospy
import os

from task_manager.database import Database
from task_manager.msg import StartEvaluation, TaskId
from task_manager.config import config
from task_manager.file_creator import FileCreator
from task_manager.constants import TaskType
from task_manager.task_manager import TaskManager
from task_manager.docker import evaluation_startup_command
import task_manager.utils as utils


class EvaluationManager:
    def __init__(self):
        rospy.Subscriber("/evaluation/start", StartEvaluation, self.start_evaluation_callback, queue_size=10)
        rospy.Subscriber("/evaluation/abort", TaskId, self.abort_evaluation_callback, queue_size=10)
        rospy.Subscriber("/evaluation/finish", TaskId, self.finish_evaluation_callback)

        self.task_manager = TaskManager(TaskType.EVALUATION)

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

        self.task_manager.add_new_task(
            data.task_id,
            data.user_id, 
            data.name,
            {
                "robotId": data.robot_id
            }
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

        print(startup_command)

        self.task_manager.start_task(data.task_id, data.user_id, startup_command)

    def finish_evaluation_callback(self, data):
        task_id = data.task_id

        self.task_manager.finish_task(task_id)

    def abort_evaluation_callback(self, data):
        task_id = data.task_id

        self.task_manager.abort_task(task_id)

    def terminate_all_processes(self):
        self.task_manager.terminate_all_tasks()


if __name__ == "__main__":
    rospy.init_node("evaluation_manager")

    task_manager = EvaluationManager()

    while not rospy.is_shutdown():
        rospy.spin()

    task_manager.terminate_all_processes()