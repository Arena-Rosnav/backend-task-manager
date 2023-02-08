import rospy

from backend_task_manager.msg import StartTraining, TaskId, StartEvaluation

from backend_task_manager.constants import ExecutableType
from backend_task_manager.database import Database
import backend_task_manager.utils as utils


def wait_for_publisher(pub):
    while pub.get_num_connections() <= 0:
        pass


def publish(pub, msg):
    wait_for_publisher(pub)

    pub.publish(msg)    


class TaskScheduler:
    def __init__(self):
        rospy.Timer(rospy.Duration(0.5), TaskScheduler.schedule_new_task)

    def schedule_new_task(_):
        scheduled_task = Database.get_scheduled_task()

        task = Database.get_task(scheduled_task["_id"])

        print(task)

        TaskScheduler.multiplex_request(task, scheduled_task["type"])

    def multiplex_request(task, type):
        if type == ExecutableType.START_TRAINING:
            return TaskScheduler.send_start_training(task)
        if type == ExecutableType.START_EVALUATION:
            return TaskScheduler.send_start_evaluation(task)

        if type == ExecutableType.ABORT_EVALUATION:
            return TaskScheduler.send_task_trigger(task, "/evaluation/abort")
        if type == ExecutableType.ABORT_TRAINING:
            return TaskScheduler.send_task_trigger(task, "/training/abort")

        if type == ExecutableType.FINISH_EVALUATION:
            return TaskScheduler.send_task_trigger(task, "/evaluation/finish")
        if type == ExecutableType.FINISH_TRAINING:
            return TaskScheduler.send_task_trigger(task, "/training/finish")

    def send_task_trigger(task, namespace):
        utils.check_parameters(str(task["_id"]), task["userId"])

        publisher = rospy.Publisher(namespace, TaskId, queue_size=10)
        
        msg = TaskId()
        msg.task_id = str(task["_id"])

        publish(publisher, msg)

    def send_start_evaluation(task):
        utils.check_parameters(task["robotId"], task["plannerId"])

        publisher = rospy.Publisher("/evaluation/start", StartEvaluation, queue_size=10)

        msg = StartEvaluation()
        msg.user_id = str(task["userId"])
        msg.task_id = str(task["_id"])
        msg.name = task["name"]

        msg.robot_id = task["robotId"]
        msg.planner_id = task["plannerId"]

        print(msg)

        publish(publisher, msg)

    def send_start_training(task):
        utils.check_parameters(task["robotId"], task["hyperparamsId"])

        publisher = rospy.Publisher("/training/start", StartTraining, queue_size=10)

        msg = StartTraining()
        msg.user_id = str(task["userId"])
        msg.task_id = str(task["_id"])
        msg.name = task["name"]

        msg.robot_id = task["robotId"]
        msg.hyperparams_id = task["hyperparamsId"]
        # msg.network_architecture_id = task.network_architecture_id

        # msg.reward_id = task.reward_id

        publish(publisher, msg)


if __name__ == "__main__":
    rospy.init_node("task_multiplexer")
    print("starting node")

    task_scheduler = TaskScheduler()

    while not rospy.is_shutdown():
        rospy.spin()

    # args = parse_args()

    # multiplex_request(args)

    # print("Startet Node", args)
