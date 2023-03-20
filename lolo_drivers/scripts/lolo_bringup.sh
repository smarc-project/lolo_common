SESSION=lolo_bringup

# Lidingo
#UTM_ZONE=34
#UTM_BAND=V

# Kristineberg
#UTM_ZONE=32
#UTM_BAND=V

# Rest of Sweden
UTM_ZONE=33
UTM_BAND=V

#Ip adress of the captain
CAPTAIN_IP=192.168.1.90

# IP Addresses to connect to neptus
# The IP of the computer running neptus
NEPTUS_IP=10.8.0.22 #  Niklas Dell computer
#NEPTUS_IP=10.8.0.82 # Ozers
#NEPTUS_IP=10.8.0.34 # Aldo

# IP of LOLO
LOLO_IP=10.8.0.86
# Port for the imc-ros-bridge, usually doesnt change from 6002.
BRIDGE_PORT=6002

tmux -2 new-session -d -s $SESSION

tmux rename-window "roscore"
tmux send-keys "roscore" C-m

tmux new-window -n 'core'
tmux send-keys "sleep 5; roslaunch lolo_drivers lolo_core.launch utm_zone:=$UTM_ZONE utm_band:=$UTM_BAND captain_ip:=$CAPTAIN_IP" C-m

tmux new-window -n 'bt'
tmux send-keys "sleep 5; roslaunch bt_mission mission.launch waypoint_tolerance:=5 neptus_addr:=$NEPTUS_IP bridge_addr:=$LOLO_IP bridge_port:=$BRIDGE_PORT robot_name:=lolo imc_id:=6 imc_system_name:=lolo" C-m

tmux new-window -n 'sidescan'
tmux send-keys "sleep 5; roslaunch lolo_sidescan real.launch" C-m

tmux new-window -n 'mbes_node'
tmux send-keys "sleep 5; roslaunch r2sonic_mbes r2sonic_mbes.launch" C-m

tmux new-window -n 'ptcloud_to_pcd'
tmux send-keys "sleep 5; rosrun pcl_ros pointcloud_to_pcd input:=/lolo/mbes_pointcloud _prefix:=/xavier_ssd/LOGS/pcd/" C-m

tmux new-window -n 'bathy_node'
tmux send-keys "sleep 5; roslaunch r2sonic_mbes r2sonic_bathy.launch"

tmux new-window -n 'bathy_node'
tmux send-keys "sleep 5; rosrun r2sonic_mbes image_array_node" C-m

tmux new-window -n 'odom_to_angles'
tmux send-keys "sleep 5; rosrun lolo_drivers odom_to_angles.py " C-m

tmux new-window -n 'logging'
tmux send-keys "cd /xavier_ssd/LOGS/" C-m
tmux send-keys "sleep 5; rosbag record -a --split --duration=1800" C-m

#tmux new-window -n 'subnero'
#tmux send-keys "sleep 5; roslaunch unetstack_interface simulation.launch " C-m

# Set default window
tmux select-window -t $SESSION:0


# Attach to session
tmux -2 attach-session -t $SESSION
