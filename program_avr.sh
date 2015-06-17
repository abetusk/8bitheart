#!/bin/bash

if [ "$1" = '' ]; then echo provide hex file; exit; fi
#echo running "avrdude -c usbtiny -p t13 -U flash:w:$1"
#avrdude -c usbtiny -p t13 -U flash:w:$1

echo running "avrdude -c usbtiny -p m328 -U flash:w:$1"
avrdude -c usbtiny -p m328 -U flash:w:$1


