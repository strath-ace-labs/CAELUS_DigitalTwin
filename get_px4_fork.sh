PX4_DIR=./Dependencies/PX4-Autopilot
read -e -p "Insert the folder containing the CAELUS fork for px4: (default $PX4_DIR [downloaded from repo if not present])" input
PX4_DIR="${input:-$PX4_DIR}"
echo $PX4_DIR

echo "[🍴] Fethcing CAELUS fork for PX4-Autopilot"
if [ ! -d $PX4_DIR ]; then
    git clone https://github.com/strathclyde-artificial-intelligence/PX4-Autopilot $PX4_DIR --recursive
    (cd $PX4_DIR; make)
fi

echo "[🚚] Installing PX4-Autpilot python dependencies"
pip3 install -r px4_python_requirements.txt

echo -e "Make sure to issue `${RED}export PX4_ROOT_FOLDER=$PX4_DIR${NC}` before starting the digital twin."