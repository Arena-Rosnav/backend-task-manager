from pymongo import MongoClient
from bson import ObjectId
from datetime import datetime, timezone, timedelta

from backend_task_manager.config import config
from backend_task_manager.constants import TaskStatus, DownloadStatus


client = MongoClient(config.get("DB_HOST"))

db = client.arena_benchmark


class Database:
    #### GETTERS ##

    def get_reward_from_id(reward_id):
        reward = db.rewards.find_one({"_id": ObjectId(reward_id)})

        return reward

    def get_robot_from_id(robot_id):
        robot = db.robots.find_one({"_id": ObjectId(robot_id)})

        return robot

    def get_hyperparams_from_id(hyperparams_id):
        hyperparams = db.hyperparams.find_one(
            {"_id": ObjectId(hyperparams_id)})

        return hyperparams

    def get_planner_from_id(planner_id):
        planner = db.planners.find_one({"_id": ObjectId(planner_id)})

        return planner

    def get_network_architecture_from_id(network_architecture_id):
        network_architecture = db.network_architectures.find_one(
            {
                "_id": ObjectId(network_architecture_id)
            }
        )

        return network_architecture

    def get_map_from_id(map_id):
        map = db.maps.find_one({"_id": ObjectId(map_id)})

        return map

    def get_scenario_from_id(scenario_id):
        scenario = db.scenarios.find_one({"_id": ObjectId(scenario_id)})

        return scenario

    #### TASKS ##

    def update_task(task_id, update):
        db.tasks.update_one({
            "_id": ObjectId(task_id)
        }, {
            "$set": {
                **update,
                "updatedAt": Database.utc_now()
            }
        })

    def get_scheduled_tasks():
        tasks = list(db.scheduledTasks.find({}))

        return tasks

    def delete_scheduled_task(id):
        db.scheduledTasks.delete_one({
            "taskId": ObjectId(id)
        })

    def get_task(task_id):
        return db.tasks.find_one({
            "_id": ObjectId(task_id)
        })

    def get_open_tasks():
        return db.tasks.find({
            "status": TaskStatus.RUNNING
        }, {
            "_id": True
        })

    def start_task(task_id, additional_args):
        db.tasks.update_one({
            "_id": ObjectId(task_id)
        }, {
            "$set": {
                **additional_args,
                "updatedAt": Database.utc_now(),
                "status": TaskStatus.RUNNING
            }
        })

    def get_running_tasks():
        return db.tasks.count_documents({
            "status": TaskStatus.RUNNING
        })

    #### TASK PROCESS ##

    def update_task_log(task_id, log):
        db.taskLogs.update_one(
            {
                "_id": ObjectId(task_id),
            },
            {
                "$set": {
                    "log": log,
                    "updatedAt": Database.utc_now()
                }
            },
            upsert=True
        )

    #### UTILS ##

    def utc_now():
        return datetime.now(timezone.utc)

    # NOTIFICATIONS

    def insert_new_task_notification(task_id, user_id, status):
        db.notifications.insert_one({
            "taskId": ObjectId(task_id),
            "userId": ObjectId(user_id),
            "createdAt": Database.utc_now(),
            "status": status,
            "read": False
        })

    # DOWNLOADS

    def update_download(task_id, user_id, type):
        db.downloads.update_one({
            "taskId": ObjectId(task_id),
            "userId": ObjectId(user_id),
            "type": type
        }, {
            "$set": {
                "updatedAt": Database.utc_now(),
                "status": DownloadStatus.READY,
                "expiresAt": Database.utc_now() + timedelta(days=30)
            }
        })

    def get_expired_downloads(now):
        return list(
            db.downloads.find({
                "expiresAt": {
                    "$lt": now
                }
            })
        )

    def delete_expired_downloads(now):
        db.downloads.delete_many({
            "expiresAt": {
                "$lt": now
            }
        })
