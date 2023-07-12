source ./devel/setup.sh

# 1. Argument: Task Id
# 2. Argument: App Token key
# 3. Argument: App Token
# 4. Argument: Base Url
# 5. Argument: Task finished Endpoint
# 6. Argument: Robot model name

roslaunch arena_bringup start_arena.launch \
    visualization:=none record_data:=true \
    is_webapp_docker:=true task_id:=$1 \
    app_token_key:=$2 app_token:=$3 \
    base_url:=$4 task_finished_endpoint:=$5 \
    model:=$6 local_planner:=$7 map_file:=$8 \
    scenario_file:=$9.json task_mode:=scenario