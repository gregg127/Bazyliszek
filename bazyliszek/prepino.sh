
#!/bin/bash
############
## CONFIG ##

#ino file path
ino="https://raw.githubusercontent.com/gregg127/Bazyliszek/tests/bazyliszek/bazyliszek.ino"

#urls of external libs
liblist="https://lidar.filipow.eu/files/liblist.txt"

#url of makefile
mkfile="https://lidar.filipow.eu/files/Makefile"

## CONFIG ##
############

name="Dex Arduino config:"
if [ "$EUID" -ne 0 ]
        then echo "$name ROOT required"
        exit
else
        if [ "$#" -ne 1 ]; then
                echo "$name provide folder name to create"
                exit
        else
        rm -rf "$1"
        mkdir "$1"
        cd "$1"
        rm -rf sketchbook
        mkdir sketchbook
        mkdir tmp
        printf "\n$name Installing soft\n\n"
        apt-get update --fix-missing
        apt-get install arduino-mk arduino gcc-avr avr-libc avrdude unzip
        printf "\n$name All required soft installed\n\n"
        cd tmp
        wget -O liblist.txt "$liblist"
        libname=""
        cat liblist.txt | while read line
        do
                wget -O lib.zip "$line"
                printf "\n\n $name getting lib from $line\n\n"
                unzip -o lib.zip -d /home/pi/arduino_libs
                rm lib.zip
        done
        cd /home/pi/"$1"/sketchbook
        wget -O sketch.ino $ino
        wget $mkfile
        make
        cd /home/pi/"$1"
        rm -rf tmp
fi
fi
