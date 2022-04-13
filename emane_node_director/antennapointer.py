#!/usr/bin/env python
#
# Copyright (c) 2020-2022 - Adjacent Link LLC, Bridgewater, New Jersey
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
import os

from pandas import DataFrame
from emane_node_director.positionorientationvelocity import calculateDirection


class AntennaPointer(object):
    def __init__(self, eelfile, tracker):
        self._eelfile = eelfile

        self._tracker = tracker

        self._states = self._eelfile_to_dataframe_states(eelfile)

        self.reset()

        # add this object to our node tracker so we can update antenna
        # positions whenever a tracked node moves
        self._tracker.add_observer(self)

        self._observers = set([])


    def add_observer(self, observer):
        self._observers.update([observer])

        observer.update()


    def update(self):
        current_pos = self._tracker.current

        for tracking_nem,row in self._current_df.iterrows():
            if not row.tracking:
                continue

            tracked_nem = row.tracking

            remote = current_pos.loc[tracked_nem]

            local = current_pos.loc[tracking_nem]

            az, el = calculateDirection(local, remote)

            self._current_df.loc[(tracking_nem,'az')] = az
            self._current_df.loc[(tracking_nem,'el')] = el

        for observer in self._observers:
            observer.update()


    def _eelfile_to_dataframe_states(self, eelfile):
        states = []
        rows = {}

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

            # -Inf nem:601 antennaprofile 3,251.29,0.031
            eventtime = float(toks[0])
            moduleid = toks[1]
            eventtype = toks[2]
            eventargs = ','.join(toks[3:])
            eventargs = eventargs.split(',')

            # ignore other events
            if not eventtype == 'antennaprofile':
                continue

            if not eventtime == last_eventtime:
                if last_eventtime is not None:
                    state_df = DataFrame(list(rows.values()),
                                         columns=['nem','ant_num','az','el','tracking'])
                    try:
                        # this seems necessary for python3
                        state_df = state_df.astype({'ant_num':int,'tracking':int})
                    except:
                        pass
                    state_df.set_index('nem', inplace=True)
                    state_df.sort_index(inplace=True)
                    states.append(state_df)
                last_eventtime = eventtime

            nem = int(moduleid.split(':')[1])
            ant_num = int(eventargs[0])
            az = float(eventargs[1])
            el = float(eventargs[2])

            rows[nem] = (nem,ant_num,az,el,0)

        state_df = DataFrame(list(rows.values()),
                             columns=['nem','ant_num','az','el','tracking'])
        try:
            # this seems necessary for python3
            state_df = state_df.astype({'ant_num':int,'tracking':int})
        except:
            pass
        state_df.set_index('nem', inplace=True)
        state_df.sort_index(inplace=True)
        states.append(state_df)

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
        self._stateidx = index
        self._state_df = self._states[self._stateidx]

        # reset current ot initial
        self._current_df = self._state_df.copy()


    def point_elevation(self, nemlist, step):
        for nem in nemlist:
            self._current_df.loc[(nem,'el')] += step


    def point_azimuth(self, nemlist, step):
        for nem in nemlist:
            self._current_df.loc[(nem,'az')] += step


    def select(self, nemlist, ant_num):
        for nem in nemlist:
            self._current_df.loc[(nem,'ant_num')] = ant_num


    def point_at(self, src_nemlist, dstnem, track):
        for srcnem in src_nemlist:
            if srcnem == dstnem:
                print('Ignoring same source, destination "%d"' % srcnem)
                continue

            local = self._tracker.current.loc[srcnem]
            remote = self._tracker.current.loc[dstnem]
            az,el = calculateDirection(local, remote)
            self._current_df.loc[(srcnem,'az')] = az
            self._current_df.loc[(srcnem,'el')] = el

            if track:
                self._current_df.loc[(srcnem,'tracking')] = dstnem


    @property
    def current(self):
        return self._current_df.copy()


    def step(self, steps):
        new_state_index = None

        if steps > 0:
            max_state_index = len(self._states) - 1

            new_state_index = min(self._stateidx + steps, max_state_index)
        else:
            new_state_index = max(self._stateidx + steps, 0)

        self.reset(new_state_index)
