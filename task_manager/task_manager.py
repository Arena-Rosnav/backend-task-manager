import os
import signal
import subprocess

from task_manager.database import Database
from task_manager.constants import TaskStatus, TaskType


class TaskManager:
    def __init__(self, task_type):
        self.running_tasks = {}
        self.task_type = task_type

    def add_new_task(self, task_id, user_id, name, task_parameters={}):
        return Database.create_new_task(task_id, self.task_type, user_id, name, task_parameters)

    def start_task(self, task_id, user_id, startup_command):
        process = subprocess.Popen(
            [startup_command], 
            shell=True, 
            preexec_fn=os.setsid,
        )

        Database.update_task(task_id, { "dockerPid": process.pid })
        
        self.running_tasks[task_id] = {
            "user_id": user_id,
            "pid": process.pid
        }

    def _stop_task(self, task_id, status):
        running_task = self.running_tasks.pop(task_id, None)

        if not running_task:
            return

        print("STOPPING TASK")

        subprocess.Popen([f"docker stop {task_id}"], shell=True)
        os.killpg(os.getpgid(running_task["pid"]), signal.SIGTERM)

        Database.update_task(
            task_id, 
            { "status": status, "updatedAt": Database.utc_now() }
        )

    def finish_task(self, task_id):
        self._stop_task(task_id, TaskStatus.FINISHED)

    def abort_task(self, task_id):
        self._stop_task(task_id, TaskStatus.ABORTED)

    def terminate_all_tasks(self):
        print("RUNNING GARBAGE COLLECTOR")

        for task in self.running_tasks.keys():
            os.killpg(os.getpgid(task["pid"]), signal.SIGTERM)