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


class EELParser:
    def parse_pov(self, eelfile):
        states = []

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
                                         columns=['nodeid','lat','lon','alt','az','el','speed','pitch','roll','yaw','tracking'])
                    state_df.set_index('nodeid', inplace=True)
                    state_df.sort_index(inplace=True)
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

        return states,rows,last_eventtime


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
                    state_df.set_index('nodeid', inplace=True)
                    state_df.sort_index(inplace=True)
                    states.append(state_df)
                last_eventtime = eventtime

            nodeid = int(moduleid.split(':')[1])
            ant_num = int(eventargs[0])
            az = float(eventargs[1])
            el = float(eventargs[2])

            rows[nodeid] = (nodeid,ant_num,az,el,0)

        return states,rows,last_eventtime
