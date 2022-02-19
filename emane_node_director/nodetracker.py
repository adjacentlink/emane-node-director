#!/usr/bin/env python
#
# Copyright (c) 2020 - Adjacent Link LLC, Bridgewater, New Jersey
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
from collections import defaultdict
import os
from pandas import DataFrame


class NodeTracker(object):
    def __init__(self, eelfile):
        self._eelfile = eelfile

        self._observers = set([])

        self._states = self._eelfile_to_dataframe_states(eelfile)

        self.reset()


    def add_observer(self, observer):
        self._observers.update([observer])


    def update(self):
        for observer in self._observers:
            observer.update()


    def _eelfile_to_dataframe_states(self, eelfile):
        states = []

        rows = defaultdict(lambda: [0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0])

        # eelfile must be present
        if not os.path.exists(eelfile):
            raise RuntimeError('EEL file "%s" does not exist' % eelfile)

        last_eventtime = None

        # process eel lines
        lineno = 0
        for line in open(eelfile, 'r'):
            lineno += 1

            line = line.strip()

            # skip blank lines
            if len(line) == 0:
                continue

            # skip comment lines
            if line[0] == '#':
                continue

            toks = line.split()

            # skip non-blank lines with too few tokens
            if len(toks)>0 and len(toks)<3:
                raise RuntimeError('Malformed EEL line %s:%d' %
                                   (eelfile, lineno))

            eventtime = float(toks[0])
            moduleid = toks[1]
            eventtype = toks[2]
            eventargs = ','.join(toks[3:])
            eventargs = eventargs.split(',')

            # ignore other events
            if not (eventtype == 'location' or eventtype == 'velocity' or eventtype == 'orientation'):
                continue

            if not eventtime == last_eventtime:
                if last_eventtime is not None:
                    state_df = DataFrame(list(rows.values()),
                                         columns=['nem','lat','lon','alt','az','el','speed','pitch','roll','yaw','tracking'])
                    state_df.set_index('nem', inplace=True)
                    state_df.sort_index(inplace=True)
                    states.append((last_eventtime, state_df))
                last_eventtime = eventtime

            # -Inf   nem:45 location gps 40.025495,-74.315441,3.0
            # <time> nem:<Id> velocity <azimuth>,<elevation>,<magnitude>
            # <time> nem:<Id> orientation <pitch>,<roll>,<yaw>
            event_nem = int(moduleid.split(':')[1])

            if eventtype == 'location':
                lat, lon, alt = list(map(float, eventargs[1:]))
                row = rows[event_nem]
                row[0] = event_nem
                row[1] = lat
                row[2] = lon
                row[3] = alt
            elif eventtype == 'velocity':
                az, el, speed = list(map(float, eventargs))
                row[0] = event_nem
                row[4] = az
                row[5] = el
                row[6] = speed
            elif eventtype == 'orientation':
                pitch, roll, yaw = list(map(float, eventargs))
                row[0] = event_nem
                row[7] = pitch
                row[8] = roll
                row[9] = yaw

        state_df = DataFrame(list(rows.values()),
                             columns=['nem','lat','lon','alt','az','el','speed','pitch','roll','yaw','tracking'])
        state_df.set_index('nem', inplace=True)
        state_df.sort_index(inplace=True)
        states.append((last_eventtime, state_df))

        return states


    def nemstr_to_nemlist(self, nemstr):
        nems=[]

        if not nemstr:
            return nems

        if len(nemstr.strip()) == 0:
            return nems

        nemranges = nemstr.split(',')

        for nemrange in nemranges:
            endpoints = nemrange.split('-')

            startendpoint = int(endpoints[0])

            stopendpoint = int(endpoints[-1])

            newnems = []

            if startendpoint > stopendpoint:
                newnems = [ i for i in range(startendpoint,stopendpoint-1,-1) ]
            else:
                newnems = [ i for i in range(startendpoint,stopendpoint+1) ]

            for i in newnems:
                if not i in nems:
                    nems.append(i)

        known_nems = self._state_df.index.unique()

        found_nems = [ nem for nem in nems if nem in known_nems ]

        return found_nems


    def reset(self, index=0):
        # start back at first state
        self._stateidx = index
        self._state_time, self._state_df = self._states[self._stateidx]

        # delta positions are 0 to start. just copy the state_df
        # positions and subtract it off to get all 0s
        self._delta_df = self._state_df.copy()
        self._delta_df -= self._state_df

        self.update()


    def move_lon(self, nemlist, step):
        for nem in nemlist:
            lon = self.current.loc[nem].lon

            delta = 0.0
            # restrict current lon to range [-180.0, 180.0]
            # TODO, need to deal with wrap around at 180 to -180
            if lon + step > 180.0:
                delta = (180.0 - lon)
            elif lon + step < -180.0:
                delta = (-180.0 - lon)
            else:
                delta = step

            self._delta_df.loc[(nem,'lon')] += delta

            # now also check if another nem is tracking this one
            # and move it too
            for trackingnem,row in self.current.iterrows():
                delta2 = delta

                if row.tracking == nem:
                    lon2 = row.lon

                    if lon2 + step > 180.0:
                        delta2 = (180.0 - lon2)
                    elif lon2 + step < -180.0:
                        delta2 = (-180.0 - lon2)
                    else:
                        delta2 = step

                    self._delta_df.loc[(trackingnem,'lon')] += delta2

        self.update()


    def move_lat(self, nemlist, step):
        for nem in nemlist:
            lat = self.current.loc[nem].lat

            delta = 0.0

            # restrict current lat to range [-90.0, 90.0]
            if lat + step > 90.0:
                delta = (90.0 - lat)
            elif lat + step < -90.0:
                delta = (-90.0 - lat)
            else:
                delta = step

            self._delta_df.loc[(nem,'lat')] += delta

            # now also check if another nem is tracking this one
            # and move it too
            for trackingnem,row in self.current.iterrows():
                delta2 = delta

                if row.tracking == nem:
                    lat2 = row.lat

                    if lat2 + step > 180.0:
                        delta2 = (180.0 - lat2)
                    elif lat2 + step < -180.0:
                        delta2 = (-180.0 - lat2)
                    else:
                        delta2 = step

                    self._delta_df.loc[(trackingnem,'lat')] += delta2

        self.update()


    def move_alt(self, nemlist, step):
        for nem in nemlist:
            alt = self.current.loc[nem].alt

            delta = 0.0

            # restrict current alt > 0.0
            if alt + step < 0.0:
                delta = (0.0 - alt)
            else:
                delta = step

            self._delta_df.loc[(nem,'alt')] += delta

            # now also check if another nem is tracking this one
            # and move it too
            for trackingnem,row in self.current.iterrows():
                delta2 = delta

                if row.tracking == nem:
                    alt2 = row.alt

                    if alt2 + step < 0.0:
                        delta2 = (0.0 - alt)
                    else:
                        delta2 = step

                    self._delta_df.loc[(trackingnem,'alt')] += delta2

        self.update()


    def moveto(self, srcnems, dstnem):
        # for more than one srcnem, the first is the anchor nem.
        # all nodes are "fixed" translated by the same vector
        # as the anchornem do the dstnem
        anchornem = srcnems[0]

        delta = self.current.loc[dstnem] - self.current.loc[anchornem]

        for nem in srcnems:
            self._delta_df.loc[nem] += delta

            for trackingnem,row in self.current.iterrows():
                if row.tracking == nem:
                    self._delta_df.loc[trackingnem] += delta

        self.update()


    def movewith(self, srcnems, dstnem):
        # movewith sets srcnems "tracking" column to the
        # dstnem id. when the dstnem moves, the source nems
        # move in the same fixed translation. This is
        # placed in the delta_df so that is is "sticky"
        # across time steps
        for srcnem in srcnems:
            if srcnem == dstnem:
                print('Ignoring same source, destination "%d"' % srcnem)
                continue

            self._delta_df.loc[(srcnem,'tracking')] = dstnem


    def orient_elevation(self, nemlist, step):
        for nem in nemlist:
            self._delta_df.loc[(nem,'el')] += step

            for trackingnem,row in self.current.iterrows():
                if row.tracking == nem:
                    self._delta_df.loc[(trackingnem,'el')] += step

        self.update()


    def orient_azimuth(self, nemlist, step):
        for nem in nemlist:
            self._delta_df.loc[(nem,'az')] += step

            for trackingnem,row in self.current.iterrows():
                if row.tracking == nem:
                    self._delta_df.loc[(trackingnem,'az')] += step

        self.update()


    def pitch(self, nemlist, step):
        for nem in nemlist:
            self._delta_df.loc[(nem, 'pitch')] += step

            for trackingnem,row in self.current.iterrows():
                if row.tracking == nem:
                    self._delta_df.loc[(trackingnem,'pitch')] += step

        self.update()

    def roll(self, nemlist, step):
        for nem in nemlist:
            self._delta_df.loc[(nem, 'roll')] += step

            for trackingnem,row in self.current.iterrows():
                if row.tracking == nem:
                    self._delta_df.loc[(trackingnem,'roll')] += step

        self.update()

    def yaw(self, nemlist, step):
        for nem in nemlist:
            self._delta_df.loc[(nem, 'yaw')] += step

            for trackingnem,row in self.current.iterrows():
                if row.tracking == nem:
                    self._delta_df.loc[(trackingnem,'yaw')] += step

        self.update()


    @property
    def current(self):
        return self._delta_df + self._state_df


    @property
    def current_time(self):
        return self._state_time


    def step(self, steps):
        new_state_index = None

        if steps > 0:
            max_state_index = len(self._states) - 1

            new_state_index = min(self._stateidx + steps, max_state_index)
        else:
            new_state_index = max(self._stateidx + steps, 0)

        # we don't reset on a step, preserve any translations
        # that have been done, and preserve tracking
        self._stateidx = new_state_index
        self._state_time, self._state_df = self._states[self._stateidx]

        self.update()
