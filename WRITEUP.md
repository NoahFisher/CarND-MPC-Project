## Dependencies
===============

To build the docker image with all the required dependencies:

```sh
./build.sh
```

To run the docker image:

```sh
./run.sh
```


## Background on Dependendies
==============

To properly get the depencies set up, I created a docker container.

The docker container was originally built with the contents of the Dockerfile using the following
command:

```
// runs in the same directory as the Dockerfile
docker build .
```

After the docker image is built, it can be run with the following command:

```
docker run -it -v $(pwd):/home/ubuntu -p 4567:4567 nfisher89/carnd-mpc bash
```
To explain...
  - `docker run` run the docker container
  - `-it` run interactively, and without a timeout
  - `-v` share the volume between the host and the docker image (allows for code editing with local
      tools like vim)
  - `-w` working directory
  - `-p` port forwarding
  - `nfisher89/carnd-mpc` the name of the container
  - `bash` after building, leave me in bash mode
