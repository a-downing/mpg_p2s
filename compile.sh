#!/bin/bash

halcompile mpg_p2s.comp

gcc -g -I/usr/include -I/usr/include/linuxcnc -URTAPI -U__MODULE__ -DULAPI -Wl,-rpath,/lib -L/lib -c mpg_p2s.c
g++ -g -std=c++17 -c mpg_logic.cpp

g++ mpg_p2s.o mpg_logic.o -llinuxcnchal -lusb-1.0 -lnml -llinuxcnc -ltirpc -o mpg_p2s
