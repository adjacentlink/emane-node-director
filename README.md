emane-node-director
==

emane-node-director is a command line application for manipulating NEM
position, orientation, velocity (POV) antenna selection and
pointing at runtime. NEM states can be snapshot in a stop motion
workflow to create mobility scenarios in [EEL file](https://github.com/adjacentlink/emane/wiki/EEL-Generator)
format. emane-node-director requires an EEL file as input to set
initial conditions. If the EEL file has multiple time steps, NEMs can be
manually stepped forward and backward to each scenario state.

The example EEL file below contains initial antenna selections for 3
NEMs (1-3) plus POV settings at two timepoints T=0.0 and 1.0.

```
# example.eel
# -----------
#
# optional nem to node mapping embedded in comments.
# emane-node-director will list the node names along with
# the nem id in its tables if it finds this list and
# if a mapping entry exists for all nems in the file
#
# nem:1  node:node-001
# nem:2  node:node-002
# nem:3  node:node-003

0.0 nem:1 antennaprofile 1,60.0,3.0
0.0 nem:2 antennaprofile 1,180.0,2.0
0.0 nem:3 antennaprofile 2,240.0,1.0

0.0 nem:1 location gps 24.645396,-82.820650,300.0
0.0 nem:1 orientation 3,4,0
0.0 nem:1 velocity 180,0,5.0

0.0 nem:2 location gps 24.606149,-82.820650,300.0
0.0 nem:2 orientation 4,3,0
0.0 nem:2 velocity 0,0,5.0

0.0 nem:3 location gps 24.645396,-82.851483,300.0
0.0 nem:3 orientation 3,4,0
0.0 nem:3 velocity 180,0,5.0

1.0 nem:1 location gps 24.645391,-82.820645,300.0
1.0 nem:1 orientation 3,4,0
1.0 nem:1 velocity 180,0,5.0

1.0 nem:2 location gps 24.606144,-82.820645,300.0
1.0 nem:2 orientation 4,3,0
1.0 nem:2 velocity 0,0,5.0

1.0 nem:3 location gps 24.645391,-82.851478,300.0
1.0 nem:3 orientation 3,4,0
1.0 nem:3 velocity 180,0,5.0
```

Use the `eventservicedevice` argument to specify the network
device where EMANE instances are [listening for events](https://github.com/adjacentlink/emane/wiki/Configuring-the-Emulator#eventservicedevice). On launch, the
Director reads the input EEL file and transmits all events
associated with the first timepoint. By default, the Director
prints the current POV and antenna state at start and after
each command issued.

```
me@host$ emane-node-director --eventservicedevice eno1 example.eel

time: 0.0
---------

location
--------
                       lat        lon    alt     az   el  speed  pitch  roll  yaw  tracking
nodeid node
1      node-001  24.645396 -82.820650  300.0  180.0  0.0    5.0    3.0   4.0  0.0         0
2      node-002  24.606149 -82.820650  300.0    0.0  0.0    5.0    4.0   3.0  0.0         0
3      node-003  24.645396 -82.851483  300.0  180.0  0.0    5.0    3.0   4.0  0.0         0

pointing
--------
                 ant_num     az   el  tracking
nodeid node
1      node-001        1   60.0  3.0         0
2      node-002        1  180.0  2.0         0
3      node-003        2  240.0  1.0         0

EMANE Node Director. Type help or ? to list commands.

director>
```

A short help description is available for each command:

```
director> help

Documented commands (type help <topic>):
========================================
azimuth    exit  move    movewith  point    reset  select  step   yaw
elevation  help  moveto  pitch     pointat  roll   show    write

director> help move

        Move one or more NEMs [n]orth, [s]outh, [e]ast, [w]est
        [u]p or [d]own by 1 or more steps.

        move NEMIds n|s|e|w|u|d [steps]

director>
```


In general, the Director works in a Read-Update-Publish
loop. Individual commands change the current state of one or more NEMs, the
new state is calculated, published to the EMANE Event Multicast Group
and printed to screen.

Commands fall into four general categories. These are best understood
with reference to the EMANE Wiki for [Platform Orientation](https://github.com/adjacentlink/emane/wiki/Platform-Orientation), [Antenna Profiles](https://github.com/adjacentlink/emane/wiki/Antenna-Profile) and [EEL File Format](https://github.com/adjacentlink/emane/wiki/EEL-Generator).

The *Move Commands* manipulate NEM latitude, longitude or altitude in
steps (`move`), by moving one NEM to the location of another (`moveto`) or by associating
a set of NEMs to move as a rigid frame with a leader (`movewith`).

The *Velocity Commands* manipulate the `azimuth` or `elevation` components of
NEM velocity in steps.

The *Orientation Commands* manipulate the `pitch`, `roll` or `yaw` components
of the NEM orientation in steps.

The *Antenna Commands* set a NEM's current antenna profile index (`select`)
or manipulate the azimuth and elevation pointing of the antenna in steps
(`point`) or by aiming the antenna of one NEM towards another (`pointat`).

The *State Commands* manipulate the state of all NEMs in
aggregate. `show` the current state or `reset` all NEMs to their
intial condition from the input EEL File. `step` between the state
of EEL File timepoints. `write` the current state of all NEMs to
the State File at the next time step.

