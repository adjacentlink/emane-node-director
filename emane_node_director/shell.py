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

        self._orientation_handler = {
            'u':self.orient_up,
            'd':self.orient_down,
            'cw':self.orient_clockwise,
            'cc':self.orient_counter_clockwise,
        }

        self._pitch_handler = {
            'cw':self.pitch_clockwise,
            'cc':self.pitch_counter_clockwise,
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


    def help_exit(self):
        print('exit')

    def do_exit(self, arg):
        return True


    def help_reset(self):
        print('reset [NEMIds]')

    def do_reset(self, arg):
        # TODO, handle restting just individual nem args
        self._tracker.reset()

        self._pointer.reset()

        self._send()


    def help_show(self):
        print('show [NEMIds]')

    def do_show(self, arg=None):
        current_pos = self._tracker.current
        current_dir = self._pointer.current
        current_pathloss = self._pathloss_calc.current

        print()
        if arg:
            nemstr = arg.split()[0]
            print(current_pos.loc[self._tracker.nemstr_to_nemlist(nemstr)])
            print()
            print(current_dir.loc[self._pointer.nemstr_to_nemlist(nemstr)])
        else:
            print(current_pos)
            print()
            print(current_dir)
            print()
            print(current_pathloss)
        print()



    def help_move(self):
        print('move NEMIds n|s|e|w|u|d [steps]')

    def do_move(self, arg):
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


    def help_moveto(self):
        print('moveto srcNEMId dstNEMId')

    def do_moveto(self, arg):
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


    def help_movewith(self):
        print('movewith slaveNEMIds masterNEMId')

    def do_movewith(self, arg):
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


    def help_orient(self):
        print('orient NEMId u|d|cw|cc steps')


    def do_orient(self, arg):
        toks = arg.split()

        try:
            nemlist = self._tracker.nemstr_to_nemlist(toks[0])
        except:
            self.help_orient()
            return

        if not nemlist:
            print('No matching nems found for %s.' % toks[0])
            return

        orientation = toks[1].lower()

        steps = 1 if len(toks) < 3 else int(toks[2])

        self._orientation_handler[orientation](nemlist, steps)
        self._send()


    def help_point(self):
        print('point NEMId u|d|cw|cc steps')

    def do_point(self, arg):
        toks = arg.split()

        try:
            nemlist = self._pointer.nemstr_to_nemlist(toks[0])
        except:
            self.help_point()
            return

        if not nemlist:
            print('No matching nems found for %s.' % toks[0])
            return

        pointing = toks[1].lower()

        if not pointing in self._pointing_handler:
            print('Undefined pointing "%s".' % toks[1])
            return

        steps = 1 if len(toks) < 3 else int(toks[2])

        self._pointing_handler[pointing](nemlist, steps)
        self._send()


    def orient_up(self, nemlist, steps):
        step = self._anglestep * steps
        self._tracker.orient_elevation(nemlist, step)

    def orient_down(self, nemlist, steps):
        step = -self._anglestep * steps
        self._tracker.orient_elevation(nemlist, step)

    def orient_clockwise(self, nemlist, steps):
        step = self._anglestep * steps
        self._tracker.orient_azimuth(nemlist, step)

    def orient_counter_clockwise(self, nemlist, steps):
        step = -self._anglestep * steps
        self._tracker.orient_azimuth(nemlist, step)


    def pitch_clockwise(self, nemlist, steps):
        step = self._anglestep * steps
        self._tracker.pitch(nemlist, step)

    def pitch_counter_clockwise(self, nemlist, steps):
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


    def help_pitch(self):
        print('pitch NEMId cw|cc steps')

    def do_pitch(self, arg):
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


    def help_roll(self):
        print('roll NEMId cw|cc steps')

    def do_roll(self, arg):
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


    def help_yaw(self):
        print('yaw NEMId cw|cc steps')

    def do_yaw(self, arg):
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


    def help_select(self):
        print('select NEMIds AntennaId')

    def do_select(self, arg):
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


    def help_pointat(self):
        print('pointat srcNEMIds dstNEMId [track]')

    def do_pointat(self, arg):
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


    def help_write(self):
        print('write')

    def do_write(self, arg):
        if not self._statefd:
            self._statefd = open(self._statefile, 'w+')

        EELWriter().write(self._writetime, self._statefd, self._tracker.current, self._pointer.current)

        self._writetime += self._timestep


    def help_step(self):
        print('step [steps]')

    def do_step(self, arg):
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

