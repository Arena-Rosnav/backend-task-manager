class TaskType:
    TRAINING = "training"
    EVALUATION = "evaluation"


class TaskStatus:
    RUNNING = "running"
    ABORTED = "aborted"
    FINISHED = "finished"


class ExecutableType:
    START_TRAINING = "start_training"
    ABORT_TRAINING = "abort_training"
    FINISH_TRAINING = "finish_training"

    START_EVALUATION = "start_evaluation"
    ABORT_EVALUATION = "abort_evaluation"
    FINISH_EVALUATION = "finish_evaluation"

    UPLOAD_DATA = "upload_data"
    UPLOAD_LOG = "upload_log"


class Docker:
    NAME_OF_MODEL = "robot"
    ROBOT_BASE_FRAME = "base_link"
    ROBOT_SENSOR_FRAME = "laser_link"
    ROBOT_LASER_UPDATE_RATE = 10


class Type:
    PRIVATE = "private"
    PUBLIC = "public"

class NotificationType:
    TRAINING_FINISHED = "training_finished"
    EVALUATION_FINISHED = "evaluation_finished"
    NEW_BEST_MODEL = "new_best_model"
    EVALUATION_STARTED = "evaluation_started"
    TRAINING_STARTED = "training_started"