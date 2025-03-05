# Development inside Docker

The Space Robotics Bench supports a Docker setup, which in itself provides an isolated development environment. By default, the [`.docker/run.bash`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/run.bash) script already mounts the source code into the container (can be disabled with `WITH_DEV_VOLUME=false`). In itself, this already makes the standalone Docker setup quite convenient for development.

## Joint a Running Container

Once the Docker container is running, you can join the running Docker container with the [`.docker/join.bash`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/join.bash) script:

```bash
.docker/join.bash
```
