target remote :3333
symbol-file /path/to/program.elf
mon reset halt
flushregs
thb app_main