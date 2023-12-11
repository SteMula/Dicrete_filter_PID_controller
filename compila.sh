#!/bin/bash
cd /home/stefano/Desktop/Dicreti\ Filter\ with\ PID\ controller/
# Add write access to all files in the current folder and its subfolders
sudo chmod -R +w *
rm -r build -f

cd /home/stefano/Desktop/Dicreti\ Filter\ with\ PID\ controller/plot
rm *.png
cd /home/stefano/Desktop/Dicreti\ Filter\ with\ PID\ controller/data
find . -type f -name 'scope_data_stefano_[0-9].tsv' -exec rm {} +
cd /home/stefano/Desktop/Dicreti\ Filter\ with\ PID\ controller/
#clear
cmake -S . -B build
sudo cmake --build build --target install
assignment_implement-controller

cd plot 
./plot.sh
