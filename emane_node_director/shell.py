#!/usr/bin/env python
#
# Copyright (c) 2021 - Adjacent Link LLC, Bridgewater, New Jersey
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.
# * Neither the name of Adjacent Link LLC nor the names of its
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

from __future__ import absolute_import, division, print_function
import cmd
from collections import defaultdict

from emane_node_director.antennapointer import AntennaPointer
from emane_node_director.eelwriter import EELWriter
from emane_node_director.nodetracker import NodeTracker
from emane_node_director.pathlosscalculator import PathlossCalculator

try:
    from emane.events import PathlossEvent,LocationEvent,AntennaProfileEvent
except:
    from emanesh.events import PathlossEvent,LocationEvent,AntennaProfileEvent


class Shell(cmd.Cmd):
    intro = 'EMANE Node Director. Type help or ? to list commands.\n'
    prompt = 'director> '

    def __init__(self, service, args):
        cmd.Cmd.__init__(self)

        self._tracker = NodeTracker(args.eelfile)

        self._pointer = AntennaPointer(args.eelfile, self._tracker)

        self._pathloss_calc = PathlossCalculator(args, self._tracker, self._pointer)

        self._service = service

        self._altstep = args.altstep

        self._latlonstep = args.latlonstep

        self._anglestep = args.anglestep

        self._writetime = 0.0

        self._timestep = args.timestep

        self._statefile = args.statefile

        self._statefd = None

        self._quiet = args.quiet

        self._direction_handler = {
            'n':self.move_north,
            's':self.move_south,
            'e':self.move_east,
            'w':self.move_west,
            'u':self.move_up,
            'd':self.move_down
        }

        self._azimuth_handler = {
            'cw':self.azimuth_clockwise,
            'cc':self.azimuth_counter_clockwise
        }

        self._elevation_handler = {
            'u':self.elevation_up,
            'd':self.elevation_down
        }

        self._pitch_handler = {
            'u':self.pitch_up,
            'd':self.pitch_down,
        }

        self._roll_handler = {
            'cw':self.roll_clockwise,
            'cc':self.roll_counter_clockwise,
        }

        self._yaw_handler = {
            'cw':self.yaw_clockwise,
            'cc':self.yaw_counter_clockwise,
        }

        self._pointing_handler = {
            'u':self.point_up,
            'd':self.point_down,
            'cw':self.point_clockwise,
            'cc':self.point_counter_clockwise,
        }

        # send initial positions
        self._send()


    def _send(self):
        event = LocationEvent()

        for nem, loc in self._tracker.current.iterrows():
            event.append(nem,
                         latitude=loc.lat,
                         longitude=loc.lon,
                         altitude=loc.alt,
                         azimuth=loc.az,
                         elevation=loc.el,
                         magnitude=loc.speed,
                         pitch=loc.pitch,
                         roll=loc.roll,
                         yaw=loc.yaw)

        self._service.publish(0, event)

        event = AntennaProfileEvent()

        for nem, pnt in self._pointer.current.iterrows():
            event.append(nemId=nem, profile=int(pnt.ant_num), azimuth=pnt.az, elevation=pnt.el)

        self._service.publish(0, event)

        pathloss_events = defaultdict(lambda: PathlossEvent())

        for (nem1, nem2), row in self._pathloss_calc.current.iterrows():
            pathloss_events[nem2].append(nem1, forward=row.pathloss)

        for nem2, event in pathloss_events.items():
            self._service.publish(nem2, event)

        if not self._quiet:
            self.do_show()


    def do_exit(self, arg):
        """
        Close and exit.

        exit
        """
        return True


    def do_reset(self, arg):
        """
        Reset all NEMs to their initial state.

        reset
        """
        # TODO, handle restting just individual nem args
        self._tracker.reset()

        self._pointer.reset()

        self._send()


    def do_show(self, arg=None):
        """
        Show the current state of one or more NEMs.

        show [NEMIds]
        """
        current_time = self._tracker.current_time
        current_pos = self._tracker.current
        current_dir = self._pointer.current
        current_pathloss = self._pathloss_calc.current

        current_time_str = 'time: %.1f' % current_time

        print()
        print('-' * len(current_time_str))
        print(current_time_str)
        print('-' * len(current_time_str))
        if arg:
            nemstr = arg.split()[0]
            print(current_pos.loc[self._tracker.nemstr_to_nemlist(nemstr)])
            print()
            print(current_dir.loc[self._pointer.nemstr_to_nemlist(nemstr)])
            print()
            print(current_pathloss.loc[self._pointer.nemstr_to_nemlist(nemstr)])
            print()
        else:
            print(current_pos)
            print()
            if current_dir.empty:
                print('No antenna pointing data')
            else:
                print(current_dir)
            print()
            if current_pathloss.empty:
                print('No pathloss data')
            else:
                print(current_pathloss)
        print()



    def do_move(self, arg):
        """
        Move one or more NEMs [n]orth, [s]outh, [e]ast, [w]est
        [u]p or [d]own by 1 or more steps.

        move NEMIds n|s|e|w|u|d [steps]
        """
        toks = arg.split()

        nemlist = []

        try:
            nemlist = self._tracker.nemstr_to_nemlist(toks[0])
        except:
            self.help_move()
            return

        if not nemlist:
            print('No matching nems found for %s.' % toks[0])
            return

        direction = toks[1].lower()

        if not direction in self._direction_handler:
            print('Undefined direction "%s".' % toks[1])
            return

        steps = 1 if len(toks) < 3 else int(toks[2])

        self._direction_handler[direction](nemlist, steps)
        self._send()


    def move_east(self, nemlist, steps):
        step = self._latlonstep * steps
        self._tracker.move_lon(nemlist, step)


    def move_west(self, nemlist, steps):
        step = -self._latlonstep * steps
        self._tracker.move_lon(nemlist, step)


    def move_north(self, nemlist, steps):
        step = self._latlonstep * steps
        self._tracker.move_lat(nemlist, step)


    def move_south(self, nemlist, steps):
        step = -self._latlonstep * steps
        self._tracker.move_lat(nemlist, step)


    def move_up(self, nemlist, steps):
        step = self._altstep * steps
        self._tracker.move_alt(nemlist, step)


    def move_down(self, nemlist, steps):
        step = -self._altstep * steps
        self._tracker.move_alt(nemlist, step)


    def do_moveto(self, arg):
        """
        Move one NEM (src) to the position of another NEM (dst).

        moveto srcNEMId dstNEMId
        """
        if len(arg) < 2:
            self.help_moveto()
            return

        toks = arg.split()

        srcnems = []
        dstnems = []

        try:
            srcnems = self._tracker.nemstr_to_nemlist(toks[0])
            dstnems = self._tracker.nemstr_to_nemlist(toks[1])
        except:
            self.help_moveto()
            return

        if not srcnems:
            print('Unknown source "%s"' % toks[0])
            return

        if not dstnems:
            print('Unknown destination "%s"' % toks[1])
            return

        if len(dstnems) > 2:
            print('Too many destination nems')
            return

        dstnem = dstnems.pop()

        self._tracker.moveto(srcnems, dstnem)
        self._send()


    def do_movewith(self, arg):
        """
        Set one or more NEMs (followers) to move with another NEM (leader).

        movewith followerNEMIds leaderNEMId
        """
        if len(arg) < 2:
            self.help_movewith()
            return

        toks = arg.split()

        srcnems = []
        dstnems = []

        try:
            srcnems = self._tracker.nemstr_to_nemlist(toks[0])
            dstnems = self._tracker.nemstr_to_nemlist(toks[1])
        except:
            self.help_movewith()
            return

        if not srcnems:
            print('Unknown source "%s"' % toks[0])
            return

        if not dstnems:
            print('Unknown destination "%s"' % toks[1])
            return

        if len(dstnems) > 2:
            print('Too many destination nems')
            return

        dstnem = dstnems.pop()

        self._tracker.movewith(srcnems, dstnem)


    def do_azimuth(self, arg):
        """
        Adjust the azimuth component of one or more NEMs velocity
        vector [cw] clockwise or [cc] counterclockwise by one or more
        steps.

        azimuth NEMIds cw|cc [steps]
        """
        toks = arg.split()

        try:
            nemlist = self._tracker.nemstr_to_nemlist(toks[0])
        except:
            self.help_azimuth()
            return

        if not nemlist:
            print('No matching nems found for %s.' % toks[0])
            return

        azimuth = toks[1].lower()

        if not azimuth in self._azimuth_handler:
            print('Unrecognized azimuth "%s".' % toks[1])
            return

        steps = 1 if len(toks) < 3 else int(toks[2])

        self._azimuth_handler[azimuth](nemlist, steps)
        self._send()


    def do_elevation(self, arg):
        """
        Adjust the elevation component of one or more NEMs velocity vector [u]p
        or [d]own by one or more steps.

        elevation NEMIds u|d [steps]
        """
        toks = arg.split()

        try:
            nemlist = self._tracker.nemstr_to_nemlist(toks[0])
        except:
            self.help_elevation()
            return

        if not nemlist:
            print('No matching nems found for %s.' % toks[0])
            return

        elevation = toks[1].lower()

        if not elevation in self._elevation_handler:
            print('Unrecognized elevation "%s".' % toks[1])
            return

        steps = 1 if len(toks) < 3 else int(toks[2])

        self._elevation_handler[elevation](nemlist, steps)
        self._send()


    def do_point(self, arg):
        """
        Adjust antenna pointing of one or more nems [u]p, [d]own, [cw] clockwise
        or [cc] counter clockwise by one or more steps.

        point NEMIds u|d|cw|cc [steps]
        """
        toks = arg.split()

        try:
            nemlist = self._pointer.nemstr_to_nemlist(toks[0])
        except:
            self.help_point()
            return

        if not nemlist:
            print('No antenna information found for nem %s.' % toks[0])
            return

        pointing = toks[1].lower()

        if not pointing in self._pointing_handler:
            print('Undefined pointing "%s".' % toks[1])
            return

        steps = 1 if len(toks) < 3 else int(toks[2])

        self._pointing_handler[pointing](nemlist, steps)
        self._send()


    def elevation_up(self, nemlist, steps):
        step = self._anglestep * steps
        self._tracker.orient_elevation(nemlist, step)

    def elevation_down(self, nemlist, steps):
        step = -self._anglestep * steps
        self._tracker.orient_elevation(nemlist, step)

    def azimuth_clockwise(self, nemlist, steps):
        step = self._anglestep * steps
        self._tracker.orient_azimuth(nemlist, step)

    def azimuth_counter_clockwise(self, nemlist, steps):
        step = -self._anglestep * steps
        self._tracker.orient_azimuth(nemlist, step)


    def pitch_up(self, nemlist, steps):
        step = self._anglestep * steps
        self._tracker.pitch(nemlist, step)

    def pitch_down(self, nemlist, steps):
        step = -self._anglestep * steps
        self._tracker.pitch(nemlist, step)


    def roll_clockwise(self, nemlist, steps):
        step = self._anglestep * steps
        self._tracker.roll(nemlist, step)

    def roll_counter_clockwise(self, nemlist, steps):
        step = -self._anglestep * steps
        self._tracker.roll(nemlist, step)


    def yaw_clockwise(self, nemlist, steps):
        step = self._anglestep * steps
        self._tracker.yaw(nemlist, step)

    def yaw_counter_clockwise(self, nemlist, steps):
        step = -self._anglestep * steps
        self._tracker.yaw(nemlist, step)


    def point_up(self, nemlist, steps):
        step = self._anglestep * steps
        self._pointer.point_elevation(nemlist, step)

    def point_down(self, nemlist, steps):
        step = -self._anglestep * steps
        self._pointer.point_elevation(nemlist, step)

    def point_clockwise(self, nemlist, steps):
        step = self._anglestep * steps
        self._pointer.point_azimuth(nemlist, step)

    def point_counter_clockwise(self, nemlist, steps):
        step = -self._anglestep * steps
        self._pointer.point_azimuth(nemlist, step)


    def do_pitch(self, arg):
        """
        Adjust the pitch of one or more NEMs [u]p or [d]own
        by one or more steps.

        pitch NEMIds u|d [steps]
        """
        toks = arg.split()

        try:
            nemlist = self._tracker.nemstr_to_nemlist(toks[0])
        except:
            self.help_pitch()
            return

        if not nemlist:
            print('No matching nems found for %s.' % toks[0])
            return

        direction = toks[1].lower()

        if not direction in self._pitch_handler:
            print('Undefined pitch direction "%s".' % toks[1])
            return

        steps = 1 if len(toks) < 3 else int(toks[2])

        self._pitch_handler[direction](nemlist, steps)
        self._send()


    def do_roll(self, arg):
        """
        Adjust the roll of one or more NEMs [cw] clockwise or [cc]
        counter-clockwise by one or more steps.

        roll NEMIds cw|cc [steps]
        """
        toks = arg.split()

        try:
            nemlist = self._tracker.nemstr_to_nemlist(toks[0])
        except:
            self.help_roll()
            return

        if not nemlist:
            print('No matching nems found for %s.' % toks[0])
            return

        direction = toks[1].lower()

        if not direction in self._roll_handler:
            print('Undefined roll direction "%s".' % toks[1])
            return

        steps = 1 if len(toks) < 3 else int(toks[2])

        self._roll_handler[direction](nemlist, steps)
        self._send()


    def do_yaw(self, arg):
        """
        Adjust the yaw of one or more NEMs [cw] clockwise or [cc]
        counter-clockwise by one or more steps.

        yaw NEMIds cw|cc [steps]
        """
        toks = arg.split()

        try:
            nemlist = self._tracker.nemstr_to_nemlist(toks[0])
        except:
            self.help_yaw()
            return

        if not nemlist:
            print('No matching nems found for %s.' % toks[0])
            return

        direction = toks[1].lower()

        if not direction in self._yaw_handler:
            print('Undefined yaw direction "%s".' % toks[1])
            return

        steps = 1 if len(toks) < 3 else int(toks[2])

        self._yaw_handler[direction](nemlist, steps)
        self._send()


    def do_select(self, arg):
        """
        Select the current antenna index for one or more NEMs.

        select NEMIds AntennaId
        """
        toks = arg.split()

        if not len(toks) == 2:
            self.help_select()
            return

        nemlist = []

        try:
            nemlist = self._pointer.nemstr_to_nemlist(toks[0])
        except:
            self.help_select()
            return

        if not nemlist:
            print('No matching nems found for %s.' % toks[0])
            return

        try:
            ant_num = int(toks[1])
            self._pointer.select(nemlist, ant_num)
            self._send()
        except Exception as e:
            print(e)
            return


    def do_pointat(self, arg):
        """
        Point the antenna of one or more NEMs (src) at another NEM
        (dst).  Make the selection sticky with the track argument -
        when the dstNEM moves, the srcNEMs automatically update
        pointing to follow.

        pointat srcNEMIds dstNEMId [track]
        """
        toks = arg.split()

        if len(toks) < 2:
            self.help_pointat()
            return

        srcnems = []
        dstnems = []

        try:
            srcnems = self._pointer.nemstr_to_nemlist(toks[0])
            dstnems = self._tracker.nemstr_to_nemlist(toks[1])
        except:
            self.help_pointat()
            return

        if not srcnems:
            print('No valid srcNEMIds in "%s"' % toks[0])
            return

        if not len(dstnems) == 1:
            print('invalid dstNEMId argument "%s"' % toks[1])
            return

        dstnem = dstnems.pop()

        track = False

        if len(toks) > 2:
            track = toks[2].lower() == 'track'

        self._pointer.point_at(srcnems, dstnem, track)

        self._send()


    def do_write(self, arg):
        """
        Write the current NEMs state to the State File at the
        next timestep.

        write
        """
        if not self._statefd:
            self._statefd = open(self._statefile, 'w+')

        EELWriter().write(self._writetime, self._statefd, self._tracker.current, self._pointer.current)

        self._writetime += self._timestep


    def do_step(self, arg):
        """
        When launched with an EELFILE with more than one timepoint,
        step forward or backwards by one or more time steps. A positive
        timesteps value N move forward N state in the EEL file,
        a negative value moves backward in time.

        step [timesteps]
        """
        steps = 1

        toks = arg.split()

        if toks:
            try:
                steps = int(toks[0])
            except:
                print('Invalid step "%s"' % toks[0])
                return

        self._tracker.step(steps)

        self._pointer.step(steps)

        self._send()

