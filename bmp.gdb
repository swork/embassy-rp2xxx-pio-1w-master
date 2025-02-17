target extended-remote ../t1

# print demangled symbols
set print asm-demangle on

# set backtrace limit to not have infinite backtrace loops
set backtrace limit 32

# scan
mon a

# Here down depends on the scan working...
att 1
mon rtt
mon erase
load
r
