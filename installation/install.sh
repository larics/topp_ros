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

toppra_count=$(pip list | grep toppra | wc -l)
if [ "$toppra_count" -ne "0" ]; then
  echo "Toppra already installed"
  exit 0
fi


git clone https://github.com/hungpham2511/toppra
cd toppra
git checkout 8df858b08175d4884b803bf6ab7f459205e54fa2

distro=`lsb_release -r | awk '{ print $2 }'`
if [ "$distro" = "18.04" ]; then
  # Install only required packages with Python2.7
  echo "Toppra: Requirements for 18.04"
  cd ..
  pip install -r requirements2.7.txt --user
  cd toppra
else
  echo "Toppra: Requirements for 20.04"
  pip install -r requirements.txt --user
fi

python setup.py install --user
cd ..
rm -rf toppra

echo "Toppra installed"
