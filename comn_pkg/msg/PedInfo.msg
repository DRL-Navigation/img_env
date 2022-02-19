# use state representation from Mit
# notice, IN MIT, robots' x-axis pointing toward the agent’s goal, ours 
# robots' x-axis pointing toward it's heading 
# observable s
float32 px
float32 py
float32 vx
float32 vy
float32 r_
float32 d_
float32 co_r

# hidden states
float32 goal_x
float32 goal_y
float32 v_pref
float32 theta
