import os

from backend_task_manager.database import Database
from backend_task_manager.config import config


def update_logs():
    open_tasks = Database.get_open_tasks()

    base_path = config["BASE_PATH"]

    for task in open_tasks:
        file_path = os.path.join(base_path, "data", str(task["_id"]), "output.txt")
        
        if (
            not os.path.exists(file_path)
            or not os.path.isfile(file_path)
        ):
            continue

        with open(file_path) as file:
            lines = file.readlines()

            last_lines = lines[-min([100, len(lines) - 1]):]

            Database.update_task_log(task["_id"], last_lines)


if __name__ == "__main__":
    update_logs()