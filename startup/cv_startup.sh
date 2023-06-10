#!/bin/bash

#copy service file to /etc/systemd/system
cp cv_startup.service /etc/systemd/system
chmod 777 /etc/systemd/system/cv_startup.service
systemctl daemon-reload
systemctl enable cv_startup.service