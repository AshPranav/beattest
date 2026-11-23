source ~/.espressif/tools/activate_idf_*.sh
cd ~/projects/<project-1-folder>
idf.py set-target esp32s3
idf.py -p /dev/ttyACM0 flash monitor