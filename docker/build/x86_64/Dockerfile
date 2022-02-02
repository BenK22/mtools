FROM python:3-alpine

ENV BTMON_VERSION="4.0.0"

# For a list of pre-defined annotation keys and value types see:
# https://github.com/opencontainers/image-spec/blob/master/annotations.md
# Note: Additional labels are added by the build workflow.
LABEL org.opencontainers.image.authors="jdahlke@gmail.com"
LABEL org.opencontainers.image.title="BTMON for Python3 and InfluxDB2"
LABEL org.opencontainers.image.description="Data collector/processor for Brultech monitoring devices. Python 3 and InfluxDB2 support."
LABEL org.opencontainers.image.source="https://github.com/jordandahlke/mtools/tree/influxdb"
LABEL org.opencontainers.image.version=${BTMON_VERSION}

#Connections to InfluxDB 2.x and InfluxDB 1.8+ require the InfluxDB-Python client.
RUN pip install --no-cache-dir influxdb-client

WORKDIR /usr/src/btmon

COPY bin/btmon3.py /usr/src/btmon

CMD [ "python", "btmon3.py", "-c", "/etc/btmon/btmon.cfg" ]