import subprocess

from backend_task_manager.database import Database
from backend_task_manager.docker import update_task_logs


def update_logs():
    open_tasks = Database.get_open_tasks()

    for task in open_tasks:
        try:
            is_running = subprocess.check_output([f"docker ps -a | grep {str(task['_id'])}"], shell=True)

            if not is_running:
                continue

            update_task_logs(task["_id"])
        except:
            pass


if __name__ == "__main__":
    update_logs()