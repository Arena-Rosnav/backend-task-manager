source ./devel/setup.sh

# 1. Argument: Task Id
# 2. Argument: App Token key
# 3. Argument: App Token
# 4. Argument: Base Url
# 5. Argument: Task finished Endpoint
# 6. Argument: New best model Endpoint
# 7. Argument: Robot model name

roslaunch arena_bringup start_training.launch \
    is_webapp_docker:=true task_id:=$1 \
    app_token_key:=$2 app_token:=$3 \
    base_url:=$4 task_finished_endpoint:=$5 \
    new_best_model_endpoint:=$6 \
    model:=$7 &

python3 src/arena-rosnav/training/scripts/train_agent.py --agent AGENT_22

# docker run -v $(echo $ARENA_BENCHMARK)/backend_api/docker/evaluation:/root/startup -ti arena-rosnav  