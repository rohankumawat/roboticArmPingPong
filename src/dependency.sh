#!/bin/bash
echo "This will install the dependencies for Rago".
echo "This requires sudo privileges."
echo "REQUIRED : libopencv-dev"

# Update package lists
sudo apt-get update



# Install OpenCV
sudo apt-get install -y libopencv-dev
