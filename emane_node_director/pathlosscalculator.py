#
# Copyright (c) 2021-2023 - Adjacent Link LLC, Bridgewater, New Jersey
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

from emane_node_director.propagationmodelalgorithm import PropagationModelAlgorithm


class PathlossCalculator(object):
    def __init__(self, args, tracker, pointer):
        self._tracker = tracker

        self._pointer = pointer

        self._algorithm = PropagationModelAlgorithm(args.pathloss, args.frequency)

        self._columns = ['node1id', 'node1', 'node2id', 'node2', 'pathloss', 'distance']

        self._current_df = DataFrame([], columns=self._columns)

        self._current_df.set_index(['node1id', 'node2id'], inplace=True)

        # add this object to the antenna pointer so pathloss
        # events can be generated whenever positions and
        # orientations change.
        self._pointer.add_observer(self)


    def update(self):
        rows = []
        for id1,id2,pathloss,distance in  self._algorithm.compute(self._tracker.current):
            rows.append(
                (id1, self._tracker.id_to_node(id1), id2, self._tracker.id_to_node(id2), pathloss, distance))

        self._current_df = DataFrame(rows, columns=self._columns)

        self._current_df.set_index(['node1id', 'node2id'], inplace=True)


    @property
    def current(self):
        return self

    @property
    def empty(self):
        return self._current_df.empty

    def iterrows(self):
        return self._current_df.iterrows()

    def get_rows(self, idlist):
        if self._current_df.empty:
            return DataFrame(columns=self._columns)
        else:
            return self._current_df.loc[idlist]

    def pathloss_table(self):
        return self._current_df.pivot_table('pathloss', index='node1id', columns='node2id')

    def distance_table(self):
        return self._current_df.pivot_table('meters', index='node1id', columns='node2id')

    def __str__(self):
        return str(self._current_df)

