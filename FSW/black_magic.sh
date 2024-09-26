#!/bin/bash

cd ..

if [ "$1" == "read_raw_bluetooth" ]
then
    echo "Reading raw bluetooth"
    python3 -m FSW.functional.read_raw_bluetooth_from_array &
fi

python3 -m FSW.cv &
python3 -m FSW.main
