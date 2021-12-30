#
#  Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
#  * Neither the name of Adjacent Link LLC nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#

from emane_node_director.conversionutils import DEGREES_TO_RADIANS


class Velocity(object):
    def __init__(self):
        self.dAzimuthDegrees_ = 0.0
        self.dElevationDegrees_ = 0.0
        self.dMagnitudeMetersPerSecond_ = 0.0
        self.dAzimuthRadians_ = 0.0
        self.dElevationRadians_ = 0.0

    def set(self,
            dAzimuthDegrees,
            dElevationDegrees,
            dMagnitudeMetersPerSecond):
        self.dAzimuthDegrees_ = dAzimuthDegrees
        self.dElevationDegrees_ = dElevationDegrees
        self.dMagnitudeMetersPerSecond_ = dMagnitudeMetersPerSecond
        self.dAzimuthRadians_ = DEGREES_TO_RADIANS(dAzimuthDegrees)
        self.dElevationRadians_ = DEGREES_TO_RADIANS(dElevationDegrees)

    @property
    def dAzimuthDegrees(self):
        return self.dAzimuthDegrees_

    @property
    def dElevationDegrees(self):
        return self.dElevationDegrees_

    @property
    def dMagnitudeMetersPerSecond(self):
        return self.dMagnitudeMetersPerSecond_

    @property
    def dAzimuthRadians(self):
        return self.dAzimuthRadians_

    @property
    def dElevationRadians(self):
        return self.dElevationRadians_

    def __eq__(self, rhs):
        return self.dAzimuthDegrees_ == rhs.dAzimuthDegrees & \
            self.dElevationDegrees_ == rhs.dElevationDegrees & \
            self.dMagnitudeMetersPerSecond_ == rhs.dMagnitudeMetersPerSecond
