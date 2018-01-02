chmod 644 pixyservice.service
cp pixyservice.service /lib/systemd/system
echo systemctl daemon-reload
echo systemctl enable pixyservice.service

