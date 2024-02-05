#!/bin/bash

# Get the local IP address
ip_address=$(hostname -I | awk '{print $1}')

# Check if the IP address is empty
if [ -z "$ip_address" ]; then
    echo "Error: Unable to determine the IP address. Check your network configuration."
    exit 1
fi

# Check if the IP address is reachable
if ! ping -c 1 -W 1 "$ip_address" > /dev/null 2>&1; then
    echo "Error: The IP address ($ip_address) is not reachable. Check your network connectivity."
    exit 1
fi

# Set the ROS_MASTER_URI
ros_master_uri="http://${ip_address}:11311"
export ROS_MASTER_URI=${ros_master_uri}

# Set the ROS_IP
export ROS_IP=${ip_address}

# Check if ROS_MASTER_URI is already set in .bashrc
if grep -q "ROS_MASTER_URI" "$HOME/.bashrc"; then
    # Replace existing ROS_MASTER_URI
    sed -i "s|export ROS_MASTER_URI=.*|export ROS_MASTER_URI=${ros_master_uri}|" "$HOME/.bashrc"
else
    # Add ROS_MASTER_URI to .bashrc
    echo "export ROS_MASTER_URI=${ros_master_uri}" >> "$HOME/.bashrc"
fi

# Check if ROS_IP is already set in .bashrc
if grep -q "ROS_IP" "$HOME/.bashrc"; then
    # Replace existing ROS_IP
    sed -i "s|export ROS_IP=.*|export ROS_IP=${ip_address}|" "$HOME/.bashrc"
else
    # Add ROS_IP to .bashrc
    echo "export ROS_IP=${ip_address}" >> "$HOME/.bashrc"
fi

# Source the updated .bashrc
# source "$HOME/.bashrc"

echo "ROS_MASTER_URI set to: $ros_master_uri"
echo "ROS_IP set to: $ip_address"
