source ./devel/setup.sh

#git clone "https://github.com/Arena-Rosnav/arena-evaluation.git" "/root/arena-evaluation"
# mv /root/startup/plotting_data.yaml /root/arena-evaluation/plot_declarations/plotting_data.yaml
# Env variablen an arena-evaluation übergeben?
# mv "/root/startup/.env" "/root/arena-evaluation/.env"
python3 /root/src/arena-evaluation/create_plots.py plotting_data.yaml
# Send plots to the server?