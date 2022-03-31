#!/bin/bash

# Exit immediatelly if a command exits with a non-zero status
set -e

# Executes a command when DEBUG signal is emitted in this script - should be after every line
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG

# Executes a command when ERR signal is emmitted in this script
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd "$MY_PATH"

# Do the pip installation
my_pip=pip2
distro=`lsb_release -r | awk '{ print $2 }'`
if [ "$distro" = "18.04" ]; then
  sudo apt install -y python-pip
  my_pip=pip2
else
  sudo apt install -y python3-pip
  my_pip=pip3
fi

# Check if toppra i shere
toppra_count=$($my_pip list | grep toppra | wc -l)
if [ "$toppra_count" -ne "0" ]; then
  echo "Toppra already installed"
  exit 0
fi

# General dependencies
sudo apt-get install -y \
    libblas-dev \
    liblapack-dev \
    libumfpack5 \
    libsuitesparse-dev

# Get toppra
git clone https://github.com/hungpham2511/toppra
cd toppra
git checkout 8df858b08175d4884b803bf6ab7f459205e54fa2

if [ "$distro" = "18.04" ]; then
  # Install only required packages with Python2.7
  echo "Toppra: Requirements for 18.04"
  sudo apt install -y \
    python \
    python-pip \
    python-numpy \
    python-setuptools \
    cython

  # Use custom requirements file for toppra
  pip2 install -r $MY_PATH/../toppra_requirements2.7.txt --user --no-deps
  
  echo "Python version: $(python --version)"
  # Explicitly use python2.7 during installation
  pip2 install . --user --no-deps
else
  echo "Toppra: Requirements for 20.04"
  sudo apt install -y \
    python \
    python3-pip \
    python3-setuptools
  pip3 install -r requirements.txt --user
  
  echo "Python version: $(python --version)"
  python3 setup.py install --user 
fi

cd ..
rm -rf $MY_PATH/toppra

echo "Toppra installed"
