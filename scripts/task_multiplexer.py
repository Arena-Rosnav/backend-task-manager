import rospy
import argparse

from task_manager.database import Database

from task_manager.msg import StartTraining, TaskId, StartEvaluation
from task_manager.constants import ExecutableType
import task_manager.utils as utils


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--type", choices=[
        ExecutableType.START_TRAINING, 
        ExecutableType.ABORT_TRAINING,
        ExecutableType.FINISH_TRAINING,

        ExecutableType.START_EVALUATION,
        ExecutableType.ABORT_EVALUATION,
        ExecutableType.FINISH_EVALUATION,

    ])
    parser.add_argument("--user_id")
    parser.add_argument("--task_id")

    parser.add_argument("--name", default=None)
    parser.add_argument("--robot_id", default=None)
    parser.add_argument("--planner_id", default=None)
    parser.add_argument("--hyperparams_id", default=None)
    parser.add_argument("--reward_id", default=None)
    parser.add_argument("--scenario_id", default=None)
    parser.add_argument("--network_architecture_id", default=None)

    known_args, _ = parser.parse_known_args()

    return known_args


def wait_for_publisher(pub):
    while pub.get_num_connections() <= 0:
        pass


def publish(pub, msg):
    wait_for_publisher(pub)

    pub.publish(msg)    


def send_start_training(args):
    utils.check_parameters(args.robot_id, args.hyperparams_id)

    publisher = rospy.Publisher("/training/start", StartTraining, queue_size=10)

    msg = StartTraining()
    msg.user_id = args.user_id
    msg.task_id = args.task_id
    msg.name = args.name

    msg.robot_id = args.robot_id
    msg.hyperparams_id = args.hyperparams_id
    # msg.network_architecture_id = args.network_architecture_id

    # msg.reward_id = args.reward_id

    publish(publisher, msg)


def send_start_evaluation(args):
    utils.check_parameters(args.robot_id, args.planner_id)

    publisher = rospy.Publisher("/evaluation/start", StartEvaluation, queue_size=10)

    msg = StartEvaluation()
    msg.user_id = args.user_id
    msg.task_id = args.task_id
    msg.name = args.name

    msg.robot_id = args.robot_id
    msg.planner_id = args.planner_id

    print(msg)

    publish(publisher, msg)


def send_task_trigger(args, namespace):
    utils.check_parameters(args.task_id, args.user_id)

    publisher = rospy.Publisher(namespace, TaskId, queue_size=10)
    
    msg = TaskId()
    msg.task_id = args.task_id

    publish(publisher, msg)


def multiplex_request(args):
    if args.type == ExecutableType.START_TRAINING:
        return send_start_training(args)
    if args.type == ExecutableType.START_EVALUATION:
        return send_start_evaluation(args)

    if args.type == ExecutableType.ABORT_EVALUATION:
        return send_task_trigger(args, "/evaluation/abort")
    if args.type == ExecutableType.ABORT_TRAINING:
        return send_task_trigger(args, "/training/abort")

    if args.type == ExecutableType.FINISH_EVALUATION:
        return send_task_trigger(args, "/evaluation/finish")
    if args.type == ExecutableType.FINISH_TRAINING:
        return send_task_trigger(args, "/training/finish")

    


if __name__ == "__main__":
    rospy.init_node("task_multiplexer")
    print("starting node")

    args = parse_args()

    multiplex_request(args)

    print("Startet Node", args)
