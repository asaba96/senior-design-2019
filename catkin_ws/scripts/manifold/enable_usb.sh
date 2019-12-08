cp m1/UserConfig.txt /home/dji/mbz2020_ws/src/mbz2020_core/src/mbz2020_onboard-sdk-ros/Onboard-SDK-3.8.1/utility/bin/linux/x86-64/.

cd /home/dji/mbz2020_ws/src/mbz2020_core/src/mbz2020_onboard-sdk-ros/Onboard-SDK-3.8.1/utility/bin/linux/x86-64/

./M210ConfigTool --usb-port /dev/ttyUSB1 --config-file UserConfig.txt --usb-connected-flight on

cd ~/mbz2020_ws
