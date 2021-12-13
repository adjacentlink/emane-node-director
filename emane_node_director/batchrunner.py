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

import datetime
import math
import time


class BatchRunner(object):
    def __init__(self, shell, batchfile):
        self._shell = shell

        self._batchfile = batchfile


    def run(self, starttimestr):
        # starttime is HH:MM:SS
        # make it a datetime
        now = datetime.datetime.now()

        if starttimestr:
            hour,minute,second = map(int, starttimestr.split(':'))

            starttime = datetime.datetime(year=now.year,
                                          month=now.month,
                                          day=now.day,
                                          hour=hour,
                                          minute=minute,
                                          second=second)
        else:
            starttime = now

        self._wait(0.0, starttime)

        with open(self._batchfile) as bfd:
            for line in bfd:

                toks = line.strip().split()

                # empty
                if not toks:
                    continue

                # comment
                if toks[0] == '#':
                    continue

                eventtime = float(toks[0])

                self._wait(eventtime, starttime)

                cmd = ' '.join(toks[1:])

                print(cmd)

                self._shell.onecmd(cmd)


    def _wait(self, eventtime, starttime):
        if math.isinf(eventtime) and eventtime < 0:
            return

        nowtime = datetime.datetime.now()

        eventabstime = starttime + datetime.timedelta(seconds=eventtime)

        sleeptime = (eventabstime - nowtime).total_seconds()

        if sleeptime <= 0:
            return

        time.sleep(sleeptime)
