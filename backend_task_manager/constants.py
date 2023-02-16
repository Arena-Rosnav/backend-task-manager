class TaskType:
    TRAINING = "training"
    EVALUATION = "evaluation"


class TaskStatus:
    PENDING = "pending"
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

    LOG_DOWNLOAD_READY = "log_download_ready"
    DATA_DOWNLOAD_READY = "data_download_ready"
    
    PREPARE_DOWNLOAD_DATA = "prepare_download_data"
    PREPARE_DOWNLOAD_LOG = "prepare_download_log"

    DATA_DOWNLOAD_EXPIRED = "data_download_expired"
    LOG_DOWNLOAD_EXPIRED = "log_download_expired"


class DownloadStatus:
    PENDING = "pending"
    READY = "ready"


class DownloadType:
    LOG = "log"
    DATA = "data"