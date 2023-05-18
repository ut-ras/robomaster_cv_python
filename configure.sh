#!/bin/bash
cp ./startUp.service /etc/systemd/system/startUp.service
chmod 664 /etc/systemd/system/startUp.service
systemctl daemon-reload
systemctl enable startUp.service