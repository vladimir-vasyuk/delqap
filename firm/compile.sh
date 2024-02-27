#!/bin/sh
./macro11 -o firmware.obj -l firmware.lst firmware.mac
./rt11obj2bin firmware.obj > firmware.map
srec_cat firmware.obj.bin -binary --byte-swap 2 -fill 0x00 0x0000 0x1000 -o firmware.mif -Memory_Initialization_File 16 -obs=2
#srec_cat ksm-firmware.obj.bin -binary --byte-swap 2 -o ksm-firmware.mem -Vmem 16 -obs=2
