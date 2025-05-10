#!/bin/bash

# Define paths
HOME_DIR="$HOME"
PARAMS_FILE="$HOME_DIR/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/params.py"
BACKUP_FILE="${PARAMS_FILE}.bak"
VSLAM_SRC="$HOME_DIR/VSLAM-UAV/Iris_VSLAM"
VSLAM_DST="$HOME_DIR/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/assets/Robots/Iris_VSLAM"

# Initialize status tracking
STATUS=0
ERROR_MSGS=()

# Backup params.py
cp "$PARAMS_FILE" "$BACKUP_FILE"
if [ $? -eq 0 ]; then
    echo "Backup created at $BACKUP_FILE"
else
    echo "Failed to create backup"
    ERROR_MSGS+=("Backup step failed")
    STATUS=1
fi

# Add custom VSLAM UAV model to params.py
NEW_LINE='ROBOTS = {"Iris": ROBOTS_ASSETS + "/Iris/iris.usd", "Iris VSLAM": ROBOTS_ASSETS + "/Iris_VSLAM/iris_vslam.usd"}'
sed -i "/^ROBOTS = {\"Iris\":/c\\$NEW_LINE" "$PARAMS_FILE"
if [ $? -eq 0 ]; then
    echo "Updated ROBOTS line in $PARAMS_FILE"
else
    echo "Failed to update ROBOTS line"
    ERROR_MSGS+=("ROBOTS line update failed")
    STATUS=1
fi

# Remove existing Iris_VSLAM directory if it exists
if [ -d "$VSLAM_DST" ]; then
    rm -rf "$VSLAM_DST"
    if [ $? -ne 0 ]; then
        echo "Failed to remove existing $VSLAM_DST"
        ERROR_MSGS+=("Directory cleanup failed")
        STATUS=1
    fi
fi

# Copy Iris_VSLAM asset to the simulator assets directory
cp -r "$VSLAM_SRC" "$VSLAM_DST"
if [ $? -eq 0 ]; then
    echo "Copied $VSLAM_SRC to $VSLAM_DST"
else
    echo "Failed to copy $VSLAM_SRC"
    ERROR_MSGS+=("Directory copy failed")
    STATUS=1
fi

# Final summary
echo ""
if [ $STATUS -eq 0 ]; then
    echo "All operations completed successfully."
else
    echo "One or more steps failed:"
    for msg in "${ERROR_MSGS[@]}"; do
        echo " - $msg"
    done
    exit 1
fi
