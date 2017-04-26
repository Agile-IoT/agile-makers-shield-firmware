#!/bin/bash


############################################################################
# Copyright (c) 2016, 2017 Libelium Comunicaciones Distribuidas S.L.       #
#                                                                          #
# All rights reserved. This program and the accompanying materials         #
# are made available under the terms of the Eclipse Public License v1.0    #
# and Eclipse Distribution License v1.0 which accompany this distribution. #
#                                                                          #
# The Eclipse Public License is available at                               #
#    http://www.eclipse.org/legal/epl-v10.html                             #
# and the Eclipse Distribution License is available at                     #
#   http://www.eclipse.org/org/documents/edl-v10.php.                      #
#                                                                          #
# Contributors:                                                            #
#    David Palomares - Initial implementation                              #
############################################################################


# Loads an hex file with the AGILE firmware to the AGILE Maker's Shield

# Script vars
PRINTHELP=false

# Start
while [[ $# > 0 ]]
do
   key="$1"
   case $key in
      -f|--file)
         hex_file="$2"
         shift
         ;;
      -h|--help)
         PRINTHELP=true
         ;;
      *) # Unknown option
         echo -e "Error: Unknown option: $1"
         PRINTHELP=true
         ;; 
   esac
   shift
done

if [ $PRINTHELP = true ]
then
   echo -e "\033[1m\nNAME\033[0m"
   echo -e "\tload_agile_firmware.sh - AGILE Maker's Shield firmware loader"
   echo -e "\033[1m\nSYNOPSIS\033[0m"
   echo -e "\tload_agile_firmware.sh [OPTION]..."
   echo -e "\033[1m\nDESCRIPTION\033[0m"
   echo -e "\tLoads an hex file with the AGILE firmware to the AGILE Maker's Shield.\n"
   echo -e "\tYou must enable the ICSP switch in your shield and have sudo privileges .\n"
   echo -e "\033[1m\t-f, --file\033[0m"
   echo -e "\t\tpath to the hex firmware file."
   echo -e "\033[1m\t-h, --help\033[0m"
   echo -e "\t\tprint this help and exit."
   echo -e ""
   exit
fi

sudo avrdude -P /dev/spidev0.0 -p atmega2560 -c linuxspi -v -U flash:w:$hex_file


