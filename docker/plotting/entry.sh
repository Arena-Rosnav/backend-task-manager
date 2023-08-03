source ./devel/setup.sh

#git clone "https://github.com/Arena-Rosnav/arena-evaluation.git" "/root/arena-evaluation"
# mv /root/startup/plotting_data.yaml /root/arena-evaluation/plot_declarations/plotting_data.yaml
# Env variablen an arena-evaluation übergeben?
# mv "/root/startup/.env" "/root/arena-evaluation/.env"

export TASK_ID=$1
export APP_TOKEN_KEY=$2
export APP_TOKEN=$3
export API_BASE_URL=$4
export FINISH_TASK_ENDPOINT=$5

python3 /root/src/arena-evaluation/create_plots.py plotting_data.yaml --is_webapp_docker true
# Send plots to the server?