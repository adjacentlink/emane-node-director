#
# Copyright (c) 2023 - Adjacent Link LLC, Bridgewater, New Jersey
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

class DataFrameWrapper:
    """
    Helper class to wrap the state DataFrame to hide the gyrations
    needed to handle access to the node state values regardless if the
    index is just a nodeid or is both a tuple (nodeid, nodename).
    """
    def __init__(self, df, id_to_index_map):
        self._df = df
        self._id_to_index_map = id_to_index_map
        self._index_to_id_map = {val:key for key,val in self._id_to_index_map.items()}

    def __add__(self, other):
        return DataFrameWrapper(self._df + other._df, self._id_to_index_map)

    def zero(self):
        return DataFrameWrapper(self._df.copy() - self._df, self._id_to_index_map)

    def copy(self):
        return DataFrameWrapper(self._df.copy(),  self._id_to_index_map)

    def nodeids(self):
        return self._id_to_index_map.keys()

    def get_cell(self, id, col):
        return self._df.loc[(self._id_to_index_map[id], col)]

    def set_cell(self, id, col, val):
        self._df.loc[(self._id_to_index_map[id],col)] = val

    def add_cell(self, id, col, addend):
        self._df.loc[(self._id_to_index_map[id],col)] += addend

    def get_row(self, id):
        return self._df.loc[self._id_to_index_map[id]]
        
    def add_row(self, id, addend_row):
        self._df.loc[self._id_to_index_map[id]] += addend_row

    def iterrows(self):
        # unwrap index to nodeid
        return [
            (self._index_to_id_map[index], row)
            for index,row in self._df.iterrows()
        ]
        
    def __str__(self):
        return str(self._df)
