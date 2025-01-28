#!/bin/bash

echo "=========================="
echo "Starting App mpctesting for {APP_PRETTY_NAME}"


systemctl start mpctesting
systemctl start rosnodeChecker
