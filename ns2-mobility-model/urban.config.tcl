# set number of nodes
set opt(nn) 30

# set activity file
set opt(af) $opt(config-path)
append opt(af) /urban.activity.tcl

# set mobility file
set opt(mf) $opt(config-path)
append opt(mf) /urban.mobility.tcl

# set start/stop time
set opt(start) 0.0
set opt(stop) 206.0

# set floor size
set opt(x) 997.4
set opt(y) 998.35
set opt(min-x) 14.95
set opt(min-y) -8.25

