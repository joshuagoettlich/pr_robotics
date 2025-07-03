
Real Robot:
  docker compose run --rm gui_container bash
  cd src/pr_robotics/scripts
  python3 Kinematics.py
  
  docker exec -it "tab" bash
  roscore

Simulation:

  docker compose run --rm gui_container bash
  roslaunch pr_robotics "tab"

  #new terminal

  docker exec -it "tab"
  cd src/pr_robotics/scripts
  python3  sim_ik_easy.py
