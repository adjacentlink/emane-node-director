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

class Orientation(object):
    def __init__(self):
        self.dRollDegrees_ = 0.0
        self.dPitchDegrees_ = 0.0
        self.dYawDegrees_ = 0.0
        self.dRollRadians_ = 0.0
        self.dPitchRadians_ = 0.0
        self.dYawRadians_ = 0.0

    def set(self,
            dRollDegrees,
            dPitchDegrees,
            dYawDegrees):
        self.dRollDegrees_ = dRollDegrees
        self.dPitchDegrees_ = dPitchDegrees
        self.dYawDegrees_ = dYawDegrees
        self.dRollRadians_ = DEGREES_TO_RADIANS(dRollDegrees)
        self.dPitchRadians_ = DEGREES_TO_RADIANS(dPitchDegrees)
        self.dYawRadians_ = DEGREES_TO_RADIANS(dYawDegrees)

    @property
    def dRollDegrees(self):
        return self.dRollDegrees_

    @property
    def dPitchDegrees(self):
        return self.dPitchDegrees_

    @property
    def dYawDegrees(self):
        return self.dYawDegrees_

    @property
    def dRollRadians(self):
        return self.dRollRadians_

    @property
    def dPitchRadians(self):
        return self.dPitchRadians_;

    @property
    def dYawRadians(self):
        return self.dYawRadians_

    def __eq__(self, rhs):
        return self.dRollDegrees_ == rhs.dRollDegrees & \
            self.dPitchDegrees_ == rhs.dPitchDegrees & \
            self.dYawDegrees_ == rhs.dYawDegrees
