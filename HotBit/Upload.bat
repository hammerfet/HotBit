:: Fire up the st-link tool and upload our .bin file
ST-LINK_CLI -c -Q -P "Debug\HotBit.bin" 0x08000000 -V "while_programming" -Run
