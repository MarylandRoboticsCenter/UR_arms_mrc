Docker image for working with UR arms. The image is based on Ubuntu 22.04.

* Build ROS2 humble docker image (run the command from the `docker` folder):
    ```
    userid=$(id -u) groupid=$(id -g) docker compose -f humble-ur_mrc-compose.yml build
    ```
* Start the container:
    ```
    docker compose -f humble-ur_mrc-compose.yml run --rm ur_mrc-docker
    ```
* UR ROS drivers and UR Gazebo simulation package are installed in the `ros_ur_driver` workspace inside the docker container. That workspace should not be used when running the container since all changes will be temporary. To make permanent changes, edit the dockerfile and rebuild the image.
* `dev_ws` workspace is the main worksapce to work with when running the docker container.
* Launch `tmux` inside the docker container and create several panes:
  * `tmux`
  * `Ctrl+A b`
  * `Ctrl+A v`

Used resources:
1. https://github.com/2b-t/docker-for-robotics