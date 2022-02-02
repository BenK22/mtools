# Introduction

This dockerfile is used to build a btmon3 docker container using the current source code.

# Building the docker container

From the top-level directory (should contain `bin/btmon3.py` file):

```
docker build -t jdahlke/btmon3 -f ./docker/build/x86_64/Dockerfile .

```

# Running the docker container

## Using `docker run`

After building the container:

```
docker run \
 -v <path to btmon.cfg file>:/etc/btmon/btmon.cfg \
 jdahlke/btmon3:latest 
```