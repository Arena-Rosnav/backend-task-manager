sudo systemctl stop task_manager.service
sudo systemctl stop log_updater.service
sudo systemctl stop cleanup.service

task_manager_service_result_path=/etc/systemd/system/task_manager.service
log_updater_service_result_path=/etc/systemd/system/log_updater.service
cleanup_service_result_path=/etc/systemd/system/cleanup.service

rm $task_manager_service_result_path
rm /etc/systemd/system/task_manager.timer
rm $log_updater_service_result_path
rm /etc/systemd/system/log_updater.timer
rm $cleanup_service_result_path
rm /etc/systemd/system/cleanup.timer
