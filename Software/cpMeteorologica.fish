#!/usr/bin/env fish
rsync -av --exclude=".*/" --exclude="README.md" /home/luka/Documents/PlatformIO/Projects/LoRaMini/ /home/luka/Meteorologica/Software
mv /home/luka/Meteorologica/Software/src/LoRaMini.cpp //home/luka/Meteorologica/Software/src/Meteorologica.cpp



