#!/bin/bash
# This script installs startup scripts for Klipper_mcu firmware
# on a raspberry pi host


# Step 1: Do main install
install_main()
{
    if ! systemctl --all --type service | grep -q "klipper"; then
        echo "The main klipper service is not yet installed."
        echo "This script does not install the main klipper service, you must do that separately."
    fi
}

# Step 2: Install startup script
install_script()
{
    report_status "Installing host mcu start script..."
    sudo cp "${SRCDIR}/scripts/klipper-raspi-host-mcu-start.sh" /etc/init.d/klipper_raspi_host_mcu
    sudo update-rc.d klipper_raspi_host_mcu defaults
}

# Helper functions
report_status()
{
    echo -e "\n\n###### $1"
}

verify_ready()
{
    if [ "$EUID" -eq 0 ]; then
        echo "This script must not run as root"
        exit -1
    fi
}

# Force script to exit if an error occurs
set -e

# Find SRCDIR from the pathname of this script
SRCDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/.. && pwd )"

# Run installation steps defined above
verify_ready
install_main
install_script
