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


from math import sin,cos,sqrt,pow

SEMI_MAJOR = 6378137.0
SEMI_MINOR = 6356752.3142
SEMI_MAJOR_2 = SEMI_MAJOR * SEMI_MAJOR
SEMI_MINOR_2 = SEMI_MINOR * SEMI_MINOR
ECC2 = (SEMI_MAJOR_2 - SEMI_MINOR_2) / SEMI_MAJOR_2


class PositionECEF(object):
    def __init__(self):
        self.dX_ = 0.0
        self.dY_ = 0.0
        self.dZ_ = 0.0
        self.bValid_ = False


    def set(self, position):
        self.bValid_= True

        dLatitudeRadians = position.dLatitudeRadians

        dLongitudeRadians = position.dLongitudeRadians

        dAltitudeMeters = position.dAltitudeMeters

        R = SEMI_MAJOR / sqrt(1.0 - (ECC2 * (pow(sin(dLatitudeRadians), 2.0))))

        self.dX_ = (R + dAltitudeMeters)  * cos(dLatitudeRadians) * cos(dLongitudeRadians)

        self.dY_ = (R + dAltitudeMeters)  * cos(dLatitudeRadians) * sin(dLongitudeRadians)

        self.dZ_ = ((1.0 - ECC2) * R + dAltitudeMeters) * sin(dLatitudeRadians)


    @property
    def dX(self):
        return self.dX_


    @property
    def dY(self):
        return self.dY_


    @property
    def dZ(self):
        return self.dZ_


    def __neg__(self):
        return self.bValid_ == False
