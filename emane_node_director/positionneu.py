# Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

from emane_node_director.conversionutils import DEGREES_TO_RADIANS
from math import sin,cos


class PositionNEU(object):
    def __init__(self):
        self.dNorthMeters_= 0.0
        self.dEastMeters_= 0.0
        self.dUpMeters_ = 0.0


    def set(self, dNorthMeters, dEastMeters, dUpMeters):
        self.dNorthMeters_ = dNorthMeters
        self.dEastMeters_ = dEastMeters
        self.dUpMeters_ = dUpMeters


    @property
    def dNorthMeters(self):
        return self.dNorthMeters_


    @property
    def dEastMeters(self):
        return self.dEastMeters_


    @property
    def dUpMeters(self):
        return self.dUpMeters_


    def rotate1(self, orientation):
        self.rotate2(orientation.dYawRadians,
                     orientation.dPitchRadians,
                     orientation.dRollRadians)


    def rotate2(self, dYawRadians, dPitchRadians, dRollRadians):
        # check if rotation needed
        if (dYawRadians != 0.0) | (dPitchRadians != 0.0) | (dRollRadians != 0.0):
            # order of rotion applied here is yaw, pitch, roll
            dRotatedNorthMeters = \
                dNorthMeters_ * cos(dYawRadians) * cos(dPitchRadians) + \
                dEastMeters_  * sin(dYawRadians) * cos(dPitchRadians) + \
                dUpMeters_    * sin(dPitchRadians)

            dRotatedEastMeters = \
                dNorthMeters_ * (cos(dYawRadians)   * sin(dPitchRadians) * sin(dRollRadians) - \
                                  sin(dYawRadians)      * cos(dRollRadians))  + \
                dEastMeters_  * (cos(dYawRadians)   * cos(dRollRadians)  + \
                                 sin(dYawRadians)      * sin(dPitchRadians)  * sin(dRollRadians)) - \
                dUpMeters_    * (cos(dPitchRadians) * sin(dRollRadians))

            dRotatedUpMeters = \
                -dNorthMeters_ * (cos(dYawRadians)  * sin(dPitchRadians) * cos(dRollRadians) + \
                                  sin(dYawRadians)   * sin(dRollRadians)) - \
                dEastMeters_  * (sin(dYawRadians)   * sin(dPitchRadians) * cos(dRollRadians) - \
                                 cos(dYawRadians)   * sin(dRollRadians)) + \
                dUpMeters_    * (cos(dPitchRadians) * cos(dRollRadians))

            self.dNorthMeters_ = dRotatedNorthMeters
            self.dEastMeters_ = dRotatedEastMeters
            self.dUpMeters_ = dRotatedUpMeters


    def adjust(dNorthMeters, dEastMeters, dUpMeters):
        self.dNorthMeters_ +=  dNorthMeters
        self.dEastMeters_ += dEastMeters
        self.dUpMeters_ += dUpMeters
