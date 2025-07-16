#!/bin/bash

# Network Setup Script for Camera USB-Ethernet Adapter
# This script configures the network for the camera's USB-ethernet adapter

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print with timestamp
log() {
    echo -e "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

# Function to check if script is run as root
check_root() {
    if [ "$EUID" -ne 0 ]; then
        log "${RED}Please run as root (use sudo)${NC}"
        exit 1
    fi
}

# Function to find USB ethernet adapter
find_usb_ethernet() {
    # Look for interfaces starting with 'enx' or 'eth'
    interfaces=$(ip link show | grep -E '^[0-9]+: (enx|eth)' | cut -d: -f2 | tr -d ' ')
    
    if [ -z "$interfaces" ]; then
        log "${RED}No USB ethernet adapter found${NC}"
        exit 1
    fi
    
    # If multiple interfaces found, let user choose
    if [ $(echo "$interfaces" | wc -l) -gt 1 ]; then
        log "${YELLOW}Multiple interfaces found. Please select one:${NC}"
        select interface in $interfaces; do
            if [ -n "$interface" ]; then
                echo "$interface"
                return
            fi
        done
    else
        echo "$interfaces"
    fi
}

# Main setup function
setup_network() {
    local interface=$1
    log "${YELLOW}Configuring interface: $interface${NC}"

    # Flush existing IP configuration
    log "Flushing existing IP configuration..."
    ip addr flush dev "$interface"
    
    # Add new IP address
    log "Setting IP address to 192.168.100.100/24..."
    ip addr add 192.168.100.100/24 dev "$interface"
    
    # Bring interface up
    log "Bringing interface up..."
    ip link set "$interface" up
    
    # Wait for interface to come up
    sleep 2
    
    # Verify interface is up
    if ip addr show "$interface" | grep -q "192.168.100.100"; then
        log "${GREEN}Network configuration successful${NC}"
    else
        log "${RED}Failed to configure network${NC}"
        exit 1
    fi
}

# Test connection function
test_connection() {
    log "Testing connection to camera (192.168.100.99)..."
    if ping -c 3 192.168.100.99 > /dev/null 2>&1; then
        log "${GREEN}Connection test successful${NC}"
    else
        log "${RED}Warning: Could not ping camera. Please check physical connection${NC}"
    fi
}

# Main execution
main() {
    log "Starting network setup..."
    
    # Check if running as root
    check_root
    
    # Find USB ethernet adapter
    interface=$(find_usb_ethernet)
    if [ -z "$interface" ]; then
        log "${RED}No interface selected${NC}"
        exit 1
    fi
    
    # Setup network
    setup_network "$interface"
    
    # Test connection
    test_connection
    
    log "${GREEN}Setup complete!${NC}"
}

# Run main function
main