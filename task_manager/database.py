from pymongo import MongoClient
from bson import ObjectId
from datetime import datetime, timezone

from task_manager.config import config
from task_manager.constants import TaskStatus


client = MongoClient(config.get("DB_HOST"), int(config.get("DB_PORT")))

db = client.arena_benchmark


class Database:
    #### GETTERS ##
    
    def get_reward_from_id(reward_id):
        reward = db.rewards.find_one({ "_id": ObjectId(reward_id) })

        return reward

    def get_robot_from_id(robot_id):
        robot = db.robots.find_one({ "_id": ObjectId(robot_id) })

        return robot

    def get_hyperparams_from_id(hyperparams_id):
        hyperparams = db.hyperparams.find_one({ "_id": ObjectId(hyperparams_id) })

        return hyperparams
    
    def get_planner_from_id(planner_id):
        planner = db.planners.find_one({ "_id": ObjectId(planner_id) })

        return planner

    def get_network_architecture_from_id(network_architecture_id):
        network_architecture = db.network_architecture.find_one(
            {
                "_id": ObjectId(network_architecture_id)
            }
        )

        return network_architecture

    def is_task_from_user(task_id, user_id):
        return not db.tasks.find_one({
            "userId": ObjectId(user_id),
            "_id": ObjectId(task_id)
        }) == None


    #### TASKS ##

    def create_new_task(task_id, task_identifier, user_id, name, task_parameters):
        now = Database.utc_now()
        
        db.tasks.insert_one({
            "_id": ObjectId(task_id),
            "userId": ObjectId(user_id),
            "type": task_identifier,
            "name": name,
            "createdAt": now,
            "updatedAt": now,
            "status": TaskStatus.RUNNING,
            **task_parameters
        })

    def update_task(task_id, update):
        db.tasks.update_one({
            "_id": ObjectId(task_id)
        },{
            "$set": update
        })

    #### TASK PROCESS ##

    def utc_now():
        return datetime.now(timezone.utc)
