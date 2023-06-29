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

tmux -2 new-session -d -s $SESSION

tmux rename-window "roscore"
tmux send-keys "roscore" C-m

tmux new-window -n 'core'
tmux send-keys "sleep 5; roslaunch lolo_drivers lolo_core.launch utm_zone:=$UTM_ZONE utm_band:=$UTM_BAND captain_ip:=$CAPTAIN_IP" C-m

tmux new-window -n 'action'
tmux send-keys "sleep 5; roslaunch lolo_action_servers lolo_actions.launch robot_name:=lolo" C-m

tmux new-window -n 'ins_driver'
tmux send-keys "sleep 1; roslaunch ixblue_ins_driver ixblue_ins_driver.launch" C-m

tmux new-window -n 'bt'
tmux send-keys "sleep 8; roslaunch smarc_bt mission.launch robot_name:=lolo" C-m

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

#tmux new-window -n 'odom_to_angles'
#tmux send-keys "sleep 5; rosrun lolo_drivers odom_to_angles.py " C-m

tmux new-window -n 'logging'
tmux send-keys "cd /xavier_ssd/LOGS/" C-m
tmux send-keys "sleep 5; rosbag record -a --split --duration=1800" C-m

#tmux new-window -n 'subnero'
#tmux send-keys "sleep 5; roslaunch unetstack_interface simulation.launch " C-m

# Set default window
tmux select-window -t $SESSION:0


# Attach to session
tmux -2 attach-session -t $SESSION
