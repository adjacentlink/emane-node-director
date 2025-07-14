#!/usr/bin/env python
#
# Copyright (c) 2020-2023 - Adjacent Link LLC, Bridgewater, New Jersey
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
from pandas import DataFrame
from emane_node_director.eelparser import EELParser
from emane_node_director.positionorientationvelocity import calculateDirection


class AntennaPointer(object):
    def __init__(self, states, tracker):
        self._states = states

        self._tracker = tracker

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

        for tracking_node,row in self._current.iterrows():
            if not row.tracking:
                continue

            tracked_node = row.tracking

            remote = current_pos.get_row(tracked_node)

            local = current_pos.get_row(tracking_node)

            az, el = calculateDirection(local, remote)

            self._current.set_cell(tracking_node,'az',az)
            self._current.set_cell(tracking_node,'el',el)

        for observer in self._observers:
            observer.update()


    def nodeidstr_to_nodeidlist(self, nodeidstr):
        nodeids=[]

        if not nodeidstr:
            return nodeids

        if len(nodeidstr.strip()) == 0:
            return nodeids

        nodeidranges = nodeidstr.split(',')

        for nodeidrange in nodeidranges:
            endpoints = nodeidrange.split('-')

            startendpoint = int(endpoints[0])

            stopendpoint = int(endpoints[-1])

            newnodeids = []

            if startendpoint > stopendpoint:
                newnodeids = [ i for i in range(startendpoint,stopendpoint-1,-1) ]
            else:
                newnodeids = [ i for i in range(startendpoint,stopendpoint+1) ]

            for i in newnodeids:
                if not i in nodeids:
                    nodeids.append(i)

        known_nodeids = self._state.nodeids()

        found_nodeids = [ nodeid for nodeid in nodeids if nodeid in known_nodeids ]

        return found_nodeids


    def reset(self, index=0):
        self._stateidx = index
        _,self._state = self._states[self._stateidx]

        # reset current ot initial
        self._current = self._state.copy()


    def point_elevation(self, nodeidlist, step):
        for nodeid in nodeidlist:
            self._current.add_cell(nodeid,'el',step)


    def point_azimuth(self, nodeidlist, step):
        for nodeid in nodeidlist:
            self._current.add_cell(nodeid,'az',step)


    def select(self, nodeidlist, ant_num):
        for nodeid in nodeidlist:
            self._current.set_cell(nodeid,'ant_num',ant_num)


    def point_at(self, src_nodeidlist, dstnodeid, track):
        for srcnodeid in src_nodeidlist:
            if srcnodeid == dstnodeid:
                print('Ignoring same source, destination "%d"' % srcnodeid)
                continue

            local = self._tracker.current.get_row(srcnodeid)
            remote = self._tracker.current.get_row(dstnodeid)
            az,el = calculateDirection(local, remote)
            self._current.set_cell(srcnodeid,'az',az)
            self._current.set_cell(srcnodeid,'el',el)

            if track:
                self._current.set_cell(srcnodeid,'tracking',dstnodeid)


    @property
    def current(self):
        return self._current.copy()


    def step(self, steps):
        new_state_index = None

        if steps > 0:
            max_state_index = len(self._states) - 1

            new_state_index = min(self._stateidx + steps, max_state_index)
        else:
            new_state_index = max(self._stateidx + steps, 0)

        self.reset(new_state_index)
