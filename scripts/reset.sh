rosservice call /gazebo/reset_simulation "{}"
curl localhost:5000/reset
bash judge/test_scripts/init_single_play.sh judge/marker_set/sim.csv localhost:5000 you enemy
#bash scripts/start.sh
