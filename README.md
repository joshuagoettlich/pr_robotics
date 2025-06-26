
docker compose run --rm gui_container bash
cd src/pr_robotics/scripts
python3 Kinematics.py

docker exec -it "tab" bash
roscore


# dependencies 
pip install python-docx
pip install ezdxf
pip install comtypes pillow 




# for Linux 
sudo apt-get install python3-tk
