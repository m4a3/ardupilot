# This script launches three instances of AP and sets the aproprate setup commands, it also loads the same params to each vehicle
# use sleeps to ensure we launch in the correct order

VEHICLE="ArduCopter"
LOCATION="ParkDavis"

x-terminal-emulator -e "sleep 5; ./Tools/autotest/sim_vehicle.py -w -v $VEHICLE -L $LOCATION -f json:0.0.0.0 -I1"

x-terminal-emulator -e "sleep 2; ./Tools/autotest/sim_vehicle.py -w -v $VEHICLE -L $LOCATION -f json:0.0.0.0 -I2"

./Tools/autotest/sim_vehicle.py -v $VEHICLE -L $LOCATION -w --console --map --slave 2
