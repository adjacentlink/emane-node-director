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
from collections import defaultdict
import re
import sys

from pandas import DataFrame

from emane_node_director.dataframewrapper import DataFrameWrapper


class EELParser:
    def __init__(self, args):
        self._args = args
        self._all_nemids,self._nemid_to_node = self._parse_comment_lines(args.eelfile)


    def _parse_comment_lines(self, eelfile):
        """
        iterate through the header comments to look for lines
        # nem:NEMID node:NODENAME

        also get all nemids that appear after the timestamp - we don't care
        about nemids in pathloss. node-directory requires locations for every nem
        so an eelfile that sets pathloss but not location for any nemid is not
        usable.
        """
        nemid_to_node = {}
        all_nemids = set([])

        for line in open(eelfile):
            line = line.strip()

            # skip blank lines
            if len(line) == 0:
                continue

            if line[0] == '#':
                found_first_comment = True

                m = re.search(r'#\s*nem:\s*(?P<nemid>\d+)\s+node:\s*(?P<node>[\w\-]+)\s*', line)

                if m:
                    nemid_to_node[int(m.group('nemid'))] = m.group('node')

            else:
                toks = line.split()
                if len(line) < 2:
                    continue

                all_nemids.add(int(toks[1].split(':')[1]))

        if nemid_to_node:
            mapped_nemids = set(nemid_to_node.keys())

            if not set(mapped_nemids.issubset(all_nemids)):
                missing_nems = mapped_nemids.difference(all_nemids)
                print(f'Found NEMID to NODENAME comments but missing nems '
                      f'{",".join(sorted(list(missing_nems)))}',
                      file=sys.stderr)
                exit(1)

        return all_nemids,nemid_to_node


    def parse(self):
        return (
            self.parse_pov(self._args.eelfile),
            self.parse_antenna_pointings(self._args.eelfile),
            self.parse_pathloss(self._args.eelfile, self._args.pathloss)
        )


    def parse_pov(self, eelfile):
        states = []

        column_names=['nodeid','lat','lon','alt','az','el','speed','pitch','roll','yaw','tracking']

        rows = defaultdict(lambda: [0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0])

        last_eventtime = None

        # process eel lines
        lineno = 0
        for line in open(eelfile, 'r'):
            lineno += 1

            line = line.strip()

            # skip blank lines
            if len(line) == 0:
                continue

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
                    state_df = DataFrame(list(rows.values()), columns=column_names)
                    state_df.set_index('nodeid', inplace=True)
                    states.append((last_eventtime, state_df))

                last_eventtime = eventtime

            # -Inf   nem:45 location gps 40.025495,-74.315441,3.0
            # <time> nem:<Id> velocity <azimuth>,<elevation>,<magnitude>
            # <time> nem:<Id> orientation <pitch>,<roll>,<yaw>
            event_nodeid = int(moduleid.split(':')[1])

            if eventtype == 'location':
                lat, lon, alt = list(map(float, eventargs[1:]))
                row = rows[event_nodeid]
                row[0] = event_nodeid
                row[1] = lat
                row[2] = lon
                row[3] = alt
            elif eventtype == 'velocity':
                az, el, speed = list(map(float, eventargs))
                row[0] = event_nodeid
                row[4] = az
                row[5] = el
                row[6] = speed
            elif eventtype == 'orientation':
                pitch, roll, yaw = list(map(float, eventargs))
                row[0] = event_nodeid
                row[7] = pitch
                row[8] = roll
                row[9] = yaw

        state_df = DataFrame(list(rows.values()), columns=column_names)
        state_df.set_index('nodeid', inplace=True)
        states.append((last_eventtime, state_df))

        return self.format_index(states)


    def format_index(self, states):
        """
        Check to see if nem/nodes comment is embedded and if it
        covers all of the nems in the file then change the DataFrame
        index to be (nodeid,node).
        """
        nemid_to_index_map = {id:id for id in self._all_nemids}

        if self._nemid_to_node:
            nemid_to_index_map = {id:(id,self._nemid_to_node[id]) for id in self._all_nemids}

            # add node column according to mapping, nodeid is now index
            # so remove from cols and add node at front
            for _,state_df in states:
                state_df.reset_index(inplace=True)
                state_df['node'] = state_df.nodeid.apply(lambda x: self._nemid_to_node[x])
                state_df.set_index(['nodeid','node'], inplace=True)

        output_states = []
        for tstamp,state_df in states:
            state_df.sort_index(inplace=True)
            output_states.append(
                (tstamp, DataFrameWrapper(state_df, nemid_to_index_map)))

        return output_states


    def parse_antenna_pointings(self, eelfile):
        states = []
        rows = {}

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
                                         columns=['nodeid','ant_num','az','el','tracking'])
                    try:
                        # this seems necessary for python3
                        state_df = state_df.astype({'ant_num':int,'tracking':int})
                    except:
                        pass

                    if not state_df.empty:
                        state_df.set_index('nodeid', inplace=True)
                        state_df.sort_index(inplace=True)
                        states.append((last_eventtime, state_df))
                last_eventtime = eventtime

            nodeid = int(moduleid.split(':')[1])
            ant_num = int(eventargs[0])
            az = float(eventargs[1])
            el = float(eventargs[2])

            rows[nodeid] = (nodeid,ant_num,az,el,0)

        state_df = DataFrame(list(rows.values()),
                             columns=['nodeid','ant_num','az','el','tracking'])
        try:
            # this seems necessary for python3
            state_df = state_df.astype({'ant_num':int,'tracking':int})
        except:
            pass

        if not state_df.empty:
            state_df.set_index('nodeid', inplace=True)
            state_df.sort_index(inplace=True)
            states.append((last_eventtime,state_df))

        if not states:
            nullentry = DataFrame([], columns=['nodeid','ant_num','az','el','tracking'])
            states.append((0.0, nullentry))

        return self.format_index(states)


    def parse_pathloss(self, eelfile, pathloss_type):
        if not pathloss_type == 'precomputed':
            return []

        states = []
        rows = {}

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

            # 0.00 nem:1 pathloss nem:2,45.243 nem:3,45.243 nem:4,45.243 nem:5,45.243
            eventtime = float(toks[0])
            moduleid = toks[1]
            eventtype = toks[2]
            eventargs = [t.split(',') for t in toks[3:]]

            # ignore other events
            if not eventtype == 'pathloss':
                continue

            if not eventtime == last_eventtime:
                if last_eventtime is not None:
                    state_df = DataFrame(list(rows.values()),
                                         columns=['node1id', 'node2id', 'pathloss', 'distance'])

                    if not state_df.empty:
                        states.append((last_eventtime, state_df))
                last_eventtime = eventtime

            nodeid1 = int(moduleid.split(':')[1])

            for nemstr,pathlossstr in eventargs:
                nodeid2 = int(nemstr.split(':')[1])
                pathloss = float(pathlossstr)
                # Enter pathloss bidirectionally - not handling asymmetric links
                # for now.
                rows[(nodeid1,nodeid2)] = (nodeid1,nodeid2,pathloss,float('nan'))
                rows[(nodeid2,nodeid1)] = (nodeid1,nodeid2,pathloss,float('nan'))

        state_df = DataFrame(list(rows.values()),
                             columns=['node1id', 'node2id', 'pathloss','distance'])

        if not state_df.empty:
            states.append((last_eventtime,state_df))

        for eventtime,state_df in states:
            if self._nemid_to_node:
                state_df['node1'] = state_df.nodeid1.apply(lambda x: self._nemid_to_node[x])
                state_df['node2'] = state_df.nodeid2.apply(lambda x: self._nemid_to_node[x])

            state_df.set_index(['node1id','node2id'], inplace=True)
            state_df.sort_index(inplace=True)

        return states

