copy () {
    sudo cp $1 $2
}

replace() {
    sudo sed -i 's,'$1','$2',' $3
}

pythonpath=$(which python)

task_manager_service_result_path=/etc/systemd/system/task_manager.service
log_updater_service_result_path=/etc/systemd/system/log_updater.service
cleanup_service_result_path=/etc/systemd/system/cleanup.service

# Task Manager

copy services/task_manager.service $task_manager_service_result_path

replace '<WorkDir>' $PWD $task_manager_service_result_path
replace '<PythonDir>' $pythonpath $task_manager_service_result_path

copy services/task_manager.timer /etc/systemd/system/task_manager.timer

# Log Updater

copy services/log_updater.service $log_updater_service_result_path

replace '<WorkDir>' $PWD $log_updater_service_result_path
replace '<PythonDir>' $pythonpath $log_updater_service_result_path

copy services/log_updater.timer /etc/systemd/system/log_updater.timer

# Cleanup

copy services/cleanup.service $cleanup_service_result_path

replace '<WorkDir>' $PWD $cleanup_service_result_path
replace '<PythonDir>' $pythonpath $cleanup_service_result_path

copy services/cleanup.timer /etc/systemd/system/cleanup.timer

# Start Services

sudo systemctl daemon-reload

sudo systemctl start task_manager.service
sudo systemctl start log_updater.service
sudo systemctl start cleanup.service