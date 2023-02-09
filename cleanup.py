from backend_task_manager.s3 import S3
from backend_task_manager.database import Database
from backend_task_manager.constants import DownloadType, NotificationType


def clean_expired_downloads():
    now = Database.utc_now()

    expired_downloads = Database.get_expired_downloads(now)

    objects = []

    for download in expired_downloads:
        objects.append(
            f"{download['taskId']}_"
            f"{'log.txt' if download['type'] == DownloadType.LOG else 'data.zip'}"
        )

    S3.delete_many(objects)
    Database.delete_expired_downloads(now)
    
    for download in expired_downloads:
        Database.insert_new_task_notification(
            download["taskId"],
            download["userId"],
            NotificationType.LOG_DOWNLOAD_EXPIRED 
                if download["type"] == DownloadType.LOG 
                else NotificationType.DATA_DOWNLOAD_EXPIRED 
        )


def clean_finished_task_data():
    ## TODO
    ##
    pass


if __name__ == "__main__":
    clean_expired_downloads()

    clean_finished_task_data()