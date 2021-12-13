FROM fedora:35

RUN mkdir -p /opt/built

RUN dnf -y update

# Copy and install packages from the latest upsteam dependencies
WORKDIR /opt
# RUN mkdir -p dependencies
# RUN dnf -y install dependencies/*\.rpm

# Build tools
RUN dnf -y install git make python3-devel python3-setuptools autoconf automake libtool rpm-build

RUN git clone https://github.com/adjacentlink/emane-node-director -b develop

WORKDIR emane-node-director
RUN ./autogen.sh
RUN ./configure
RUN make rpm
RUN cp $(find .rpmbuild/RPMS -name "*\.rpm") /opt/built
RUN dnf -y install /opt/built/*

RUN dnf clean all

RUN echo 'complete'

