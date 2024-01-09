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

from itertools import product
import math
import sys
from emane_node_director.positionorientationvelocity import distance_to

MIN_DISTANCE_METERS=0.001
MIN_ALTITUDE_METERS=MIN_DISTANCE_METERS
MIN_PATHLOSS=0.0


class NullAlgorithm(object):
    def compute(self, pairs):
        return []
    def update_step(self, state_time, current_df):
        pass


class TwoRayAlgorithm(object):
    def __init__(self, calc):
        self._calc = calc

    def compute(self, pairs):
        """
        Replicating emane two ray pathloss calculation for each pair

        dPathloss =
        (40.0 * log10(dDistance)) -
        (20.0 *
         (log10(locationPairInfo.getLocalPOV().getPosition().getAltitudeMeters()) +
          log10(locationPairInfo.getRemotePOV().getPosition().getAltitudeMeters())));
        """
        pathlosses = []

        for (node1id, loc1), (node2id, loc2) in pairs:
            if node1id == node2id:
                continue

            distance_meters = distance_to(loc1, loc2)[0]

            dist = max(distance_meters, MIN_DISTANCE_METERS)

            alt1 = max(loc1.alt, MIN_ALTITUDE_METERS)

            alt2 = max(loc2.alt, MIN_ALTITUDE_METERS)

            pathloss = (40.0 * math.log10(dist)) - (20.0 * (math.log10(alt1) + math.log10(alt2)))

            pathlosses.append((node1id, node2id, pathloss, dist))

        return pathlosses

    def update_step(self, state_time, current_df):
        self._calc.update()


class FreespaceAlgorithm(object):
    FSPL_CONST = 41.916900439033640

    def __init__(self, calc, frequency):
        self._calc
        self._frequency = float(frequency)


    def compute(self, pairs):
        """
        Replicating emane freespace pathloss calculation for each pair using the single
        frequency passed in.

        auto val =
        20.0 * log10(FSPL_CONST * (segment.getFrequencyHz() / 1000000.0) * (dDistance / 1000.0));

        pathloss[i++] = val < 0 ? 0 : val;
        """
        pathlosses = []

        for (node1id, loc1), (node2id, loc2) in pairs:
            if node1id == node2id:
                continue

            distance_meters = distance_to(loc1, loc2)[0]

            dist = max(distance_meters, MIN_DISTANCE_METERS)

            pathloss = 20.0 * math.log10(FreespaceAlgorithm.FSPL_CONST * \
                                         (self._frequency / 1000000.0) * (dist/1000.0))

            pathlosses.append((node1id, node2id, pathloss, dist))

        return pathlosses

    def update_step(self, state_time, current_df):
        self._calc.update()



class PrecomputedAlgorithm(object):
    def __init__(self, calc, pathloss_states):
        self._calc = calc
        self._states = pathloss_states
        self._calc.set_current_df(pathloss_states[0][1])

    def compute(self, locations):
        # pathloss updates only occur via update_step on
        # which pathloss values are defined from the eel file
        return []

    def update_step(self, state_time, current_df):
        if not self._states:
            return

        current = self._states[0]

        upto_states = list(filter(lambda s: s[0]<=state_time, self._states))

        if upto_states:
            current = self._states[len(upto_states)-1]

        self._calc.set_current_df(current[1])


class PropagationModelAlgorithm(object):
    def __init__(self, calc, algorithm, frequency, pathloss_states):
        if algorithm == 'none':
            self._alg = NullAlgorithm()
        elif algorithm == 'freespace':
            self._alg = FreespaceAlgorithm(calc, frequency)
        elif algorithm == '2ray':
            self._alg = TwoRayAlgorithm(calc)
        elif algorithm == 'precomputed':
            if pathloss_states:
                self._alg = PrecomputedAlgorithm(calc, pathloss_states)
            else:
                print('Warning - no pathlosses found when using pathloss=precomputed',
                      file=sys.stderr)
                self._alg = NullAlgorithm()
        else:
            raise ValueError('Unknown pathloss algorithm "%s"' % algorithm)

    def compute(self, locations):
        return self._alg.compute(product(locations.iterrows(), locations.iterrows()))

    def update_step(self, state_time, current_df):
        self._alg.update_step(state_time, current_df)
