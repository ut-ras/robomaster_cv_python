#!/bin/bash

#copy service file to /etc/systemd/system
cp cv_startup.service /etc/systemd/system
#give all users read/write/execute permissions
chmod 777 /etc/systemd/system/cv_startup.service
#reload all systemd daemons
sudo systemctl daemon-reload
#enable cv_startup.service in systemd
sudo systemctl enable cv_startup.service