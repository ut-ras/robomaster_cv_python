#!/bin/bash
cp ./river.service /etc/systemd/system/river.service
chmod 664 /etc/systemd/system/river.service
systemctl daemon-reload
systemctl enable river.service