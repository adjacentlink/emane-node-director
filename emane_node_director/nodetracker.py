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

import os


class NodeTracker(object):
    def __init__(self, states):
        self._observers = set([])

        self._states = states

        self.reset()


    def add_observer(self, observer):
        self._observers.update([observer])


    def update(self):
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


    def reset(self):
        # start back at first state
        self._stateidx = 0
        self._state_time,self._state = self._states[self._stateidx]

        # delta positions are 0 to start. just copy the state
        # positions and subtract it off to get all 0s
        self._delta_state = self._state.zero()
        self.update()


    def move_lon(self, nodeidlist, step):
        for nodeid in nodeidlist:
            lon = self.current.get_cell(nodeid, 'lon')
            delta = 0.0
            # restrict current lon to range [-180.0, 180.0]
            # TODO, need to deal with wrap around at 180 to -180
            if lon + step > 180.0:
                delta = (180.0 - lon)
            elif lon + step < -180.0:
                delta = (-180.0 - lon)
            else:
                delta = step

            self._delta_state.add_cell(nodeid,'lon',delta)

            # now also check if another nodeid is tracking this one
            # and move it too
            for trackingnodeid,row in self.current.iterrows():
                delta2 = delta

                if row.tracking == nodeid:
                    lon2 = row.lon

                    if lon2 + step > 180.0:
                        delta2 = (180.0 - lon2)
                    elif lon2 + step < -180.0:
                        delta2 = (-180.0 - lon2)
                    else:
                        delta2 = step

                    self._delta_state.add_cell(trackingnodeid,'lon',delta2)

        self.update()


    def move_lat(self, nodeidlist, step):
        for nodeid in nodeidlist:
            lat = self.current.get_cell(nodeid, 'lat')

            delta = 0.0

            # restrict current lat to range [-90.0, 90.0]
            if lat + step > 90.0:
                delta = (90.0 - lat)
            elif lat + step < -90.0:
                delta = (-90.0 - lat)
            else:
                delta = step

            self._delta_state.add_cell(nodeid,'lat',delta)

            # now also check if another nodeid is tracking this one
            # and move it too
            for trackingnodeid,row in self.current.iterrows():
                delta2 = delta

                if row.tracking == nodeid:
                    lat2 = row.lat

                    if lat2 + step > 180.0:
                        delta2 = (180.0 - lat2)
                    elif lat2 + step < -180.0:
                        delta2 = (-180.0 - lat2)
                    else:
                        delta2 = step

                    self._delta_state.add_cell(trackingnodeid,'lat',delta2)

        self.update()


    def move_alt(self, nodeidlist, step):
        for nodeid in nodeidlist:
            alt = self.current.get_cell(nodeid, 'alt')

            delta = 0.0

            # restrict current alt > 0.0
            if alt + step < 0.0:
                delta = (0.0 - alt)
            else:
                delta = step

            self._delta_state.add_cell(nodeid,'alt',delta)

            # now also check if another nodeid is tracking this one
            # and move it too
            for trackingnodeid,row in self.current.iterrows():
                delta2 = delta

                if row.tracking == nodeid:
                    alt2 = row.alt

                    if alt2 + step < 0.0:
                        delta2 = (0.0 - alt)
                    else:
                        delta2 = step

                    self._delta_state.add_cell(trackingnodeid,'alt',delta2)

        self.update()


    def moveto(self, srcnodeids, dstnodeid):
        # for more than one srcnodeid, the first is the anchor nodeid.
        # all nodes are "fixed" translated by the same vector
        # as the anchornodeid do the dstnodeid
        anchornodeid = srcnodeids[0]

        delta = self.current.get_row(dstnodeid) - self.current.get_row(anchornodeid)

        for nodeid in srcnodeids:
            self._delta_state.add_row(nodeid, delta)

            for trackingnodeid,row in self.current.iterrows():
                if row.tracking == nodeid:
                    self._delta_state.add_row(trackingnodeid, delta)

        self.update()


    def movewith(self, srcnodeids, dstnodeid):
        # movewith sets srcnodeids "tracking" column to the
        # dstnodeid id. when the dstnodeid moves, the source nodeids
        # move in the same fixed translation. This is
        # placed in the delta so that is is "sticky"
        # across time steps
        for srcnodeid in srcnodeids:
            if srcnodeid == dstnodeid:
                print('Ignoring same source, destination "%d"' % srcnodeid)
                continue

            self._delta_state.set_cell(srcnodeid,'tracking',dstnodeid)


    def orient_elevation(self, nodeidlist, step):
        for nodeid in nodeidlist:
            self._delta_state.add_cell(nodeid, 'el', step)

            for trackingnodeid,row in self.current.iterrows():
                if row.tracking == nodeid:
                    self._delta_state.add_cell(trackingnodeid, 'el', step)

        self.update()


    def orient_azimuth(self, nodeidlist, step):
        for nodeid in nodeidlist:
            self._delta_state.add_cell(nodeid,'az', step)

            for trackingnodeid,row in self.current.iterrows():
                if row.tracking == nodeid:
                    self._delta_state.add_cell(trackingnodeid,'az', step)

        self.update()


    def pitch(self, nodeidlist, step):
        for nodeid in nodeidlist:
            self._delta_state.add_cell(nodeid, 'pitch', step)

            for trackingnodeid,row in self.current.iterrows():
                if row.tracking == nodeid:
                    self._delta_state.add_cell(trackingnodeid, 'pitch', step)

        self.update()

    def roll(self, nodeidlist, step):
        for nodeid in nodeidlist:
            self._delta_state.add_cell(nodeid, 'roll', step)

            for trackingnodeid,row in self.current.iterrows():
                if row.tracking == nodeid:
                    self._delta_state.add_cell(trackingnodeid,'roll',step)

        self.update()

    def yaw(self, nodeidlist, step):
        for nodeid in nodeidlist:
            self._delta_state.add_cell(nodeid, 'yaw', step)

            for trackingnodeid,row in self.current.iterrows():
                if row.tracking == nodeid:
                    self._delta_state.add_cell(trackingnodeid, 'yaw', step)

        self.update()


    @property
    def current(self):
        return self._delta_state + self._state


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


        self._stateidx = new_state_index
        self._state_time, self._state = self._states[self._stateidx]

        for observer in self._observers:
            observer.update_step(self._state_time)


    def id_to_node(self, id):
        _,state = self._states[self._stateidx]
        return state.id_to_node(id)
