#
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
#


from emane_node_director.conversionutils import AI_TO_BE_DEGREES,NEG_90_TO_POS_90_DEGREES,NORMALIZE_VECTOR
from emane_node_director.position import Position
from emane_node_director.velocity import Velocity
from emane_node_director.orientation import Orientation
from emane_node_director.positionecef import PositionECEF
from emane_node_director.positionneu import PositionNEU
from math import sin,cos,asin,atan,pi


def distance_to(local, remote):
    localPOV = PositionOrientationVelocity()

    localPosition = Position()
    localPosition.set(local.lat, local.lon, local.alt)

    localPOV.set(localPosition,
                 (Orientation(), False),
                 (Velocity(), False))

    remotePOV = PositionOrientationVelocity()

    remotePosition = Position()
    remotePosition.set(remote.lat, remote.lon, remote.alt)

    remotePOV.set(remotePosition,
                  (Orientation(), False),
                  (Velocity(), False))

    # NEU for the remote node, includes rotation for orientation
    remoteNEU = localPOV.getPositionNEU(remotePOV)

    # get distance
    dDistanceMeters = NORMALIZE_VECTOR(remoteNEU.dNorthMeters,
                                       remoteNEU.dEastMeters,
                                       remoteNEU.dUpMeters)

    return dDistanceMeters, remoteNEU


def calculateDirection(local, remote):
    dDistanceMeters, remoteNEU = distance_to(local, remote)

    dTargetAzimuthDegrees = 0.0

    # get elevation
    dTargetElevationDegrees = asin(remoteNEU.dUpMeters / dDistanceMeters) * (180.0 / pi)

    if remoteNEU.dNorthMeters == 0.0:
        if remoteNEU.dEastMeters > 0.0:
            dTargetAzimuthDegrees = 90.0
        else:
            dTargetAzimuthDegrees = 270.0
    else:
        if remoteNEU.dEastMeters == 0.0:
            if remoteNEU.dNorthMeters > 0.0:
                dTargetAzimuthDegrees = 0.0
            else:
                dTargetAzimuthDegrees = 180.0
        else:
            dTargetAzimuthDegrees = \
              atan(remoteNEU.dEastMeters / remoteNEU.dNorthMeters) * (180.0 / pi)

        if remoteNEU.dNorthMeters < 0.0:
            dTargetAzimuthDegrees += 180.0

        if (remoteNEU.dNorthMeters > 0.0) & (remoteNEU.dEastMeters < 0.0):
            dTargetAzimuthDegrees += 360.0

    return (dTargetAzimuthDegrees,dTargetElevationDegrees)


def adjustOrientation(orientation, velocity):
    dYaw = velocity.dAzimuthDegrees + orientation.dYawDegrees

    # set yaw to [0 to 360)
    AI_TO_BE_DEGREES(dYaw, 0.0, 360.0);

    dPitch = velocity.dElevationDegrees + orientation.dPitchDegrees

    # set dpitch to [0 to 360)
    dPitch = AI_TO_BE_DEGREES(dPitch, 0.0, 360.0)

    # set pitch to [-90 to 90]
    dPitch = NEG_90_TO_POS_90_DEGREES(dPitch)

    return Orientation().set(orientation.dRollDegrees,dPitch,dYaw)


class PositionOrientationVelocity(object):
    def __init__(self):
        self.position_ = Position()
        self.orientation_ = Orientation()
        self.velocity_ = Velocity()
        self.bValid_ = False
        self.bHasOrientation_ = False
        self.bHasVelocity_ = False


    def set(self,
            position,
            orientation_pair,
            velocity_pair):
        self.position_ = position
        self.orientation_ = orientation_pair[0]
        self.velocity_ = velocity_pair[0]
        self.bValid_ = True
        self.bHasOrientation_ = orientation_pair[1]
        self.bHasVelocity_ = velocity_pair[1]
        self.positionECEF_ = PositionECEF()
        self.positionECEF_.set(position)
        self.adjustedOrientation_ = adjustOrientation(self.orientation_, self.velocity_)


    def set_position(self, lat, lon, alt):
        """
        Helper function for building POV when only position
        is known
        """
        position = Position()
        position.set(lat, lon, alt)
        self.set(position,
                 (Orientation(), False),
                 (Velocity(), False))


    def update(self,
               position,
               orientation_pair,
               velocity_pair):
        if (position == self.position_) & \
           (not orientation_pair[1] | (orientation_pair[0] == self.orientation_)) & \
           (not velocity_pair[1] | (velocity_pair[0] == self.velocity_)):
            return False
        else:
            bValid_ = True

            if position != position_:
                self.position_ = position
                self.positionECEF_ = PositionECEF(position_)

            bCalculateAdjustedOrientation = False

            if orientation[1]:
                orientation_ = orientation[0]
                bHasOrientation_ = True
                bCalculateAdjustedOrientation = True

            if velocity[1]:
                self.velocity_ = velocity[0]
                self.bHasVelocity_ = True
                bCalculateAdjustedOrientation = True

            if bCalculateAdjustedOrientation:
                self.adjustedOrientation_ = adjustOrientation(orientation_,velocity_)

            return True


    def getPosition(self):
        return self.position_


    def getOrientation(self):
        return (self.orientation_, self.bHasOrientation_)


    def getAdjustedOrientation(self):
        return (self.adjustedOrientation_, self.bHasOrientation_ | self.bHasVelocity_)


    def getVelocity(self):
        return (self.velocity_, self.bHasVelocity_)


    def getPositionECEF(self):
        return self.positionECEF_


    def getPositionNEU(self, other):
        selfECEF = self.getPositionECEF()
        otherECEF = other.getPositionECEF()

        dX = otherECEF.dX - selfECEF.dX
        dY = otherECEF.dY - selfECEF.dY
        dZ = otherECEF.dZ - selfECEF.dZ

        dLatitudeRadians = self.position_.dLatitudeRadians
        dLongitudeRadians = self.position_.dLongitudeRadians

        dNorthMeters = -dX * sin(dLatitudeRadians) * cos(dLongitudeRadians) - \
            dY * sin(dLatitudeRadians) * sin(dLongitudeRadians) + \
            dZ * cos(dLatitudeRadians)

        dEastMeters = -dX * sin(dLongitudeRadians) + dY * cos(dLongitudeRadians)

        dUpMeters = dX * cos(dLatitudeRadians) * cos(dLongitudeRadians) + \
            dY * cos(dLatitudeRadians) * sin(dLongitudeRadians) + \
            dZ * sin(dLatitudeRadians)

        otherNEU = PositionNEU()
        otherNEU.set(dNorthMeters,dEastMeters,dUpMeters)

        if(self.bHasOrientation_ | self.bHasVelocity_):
            otherNEU.rotate1(self.adjustedOrientation_)

        return otherNEU


    def __neg__(self):
        return self.bValid_ == False

