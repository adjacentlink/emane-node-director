#
#  Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

class Position(object):
    def __init__(self):
        self.dLatitudeDegrees_ = 0.0
        self.dLongitudeDegrees_ = 0.0
        self.dAltitudeMeters_ = 0.0
        self.dLatitudeRadians_ = 0.0
        self.dLongitudeRadians_ = 0.0

    def set(self,
            dLatitudeDegrees,
            dLongitudeDegrees,
            dAltitudeMeters):
        self.dLatitudeDegrees_ = dLatitudeDegrees
        self.dLongitudeDegrees_ = dLongitudeDegrees
        self.dAltitudeMeters_ = dAltitudeMeters
        self.dLatitudeRadians_ = DEGREES_TO_RADIANS(dLatitudeDegrees)
        self.dLongitudeRadians_ = DEGREES_TO_RADIANS(dLongitudeDegrees)

    @property
    def dLatitudeDegrees(self):
        return self.dLatitudeDegrees_

    @property
    def dLongitudeDegrees(self):
        return self.dLongitudeDegrees_

    @property
    def dAltitudeMeters(self):
        return self.dAltitudeMeters_

    @property
    def dLatitudeRadians(self):
        return self.dLatitudeRadians_

    @property
    def dLongitudeRadians(self):
        return self.dLongitudeRadians_

    def __eq__(self, rhs):
        return self.dLatitudeDegrees_ == rhs.dLatitudeDegrees & \
            self.dLongitudeDegrees_ == rhs.dLongitudeDegrees & \
            self.dAltitudeMeters_ == rhs.dAltitudeMeters
