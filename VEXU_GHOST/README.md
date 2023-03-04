# Setting Up LIDAR container

## Required Software
* Docker
* VNC Viewer (this installation was tested with [real vnc](https://www.realvnc.com/en/connect/download/viewer/), but any should do)

## Helpful Commands
* `docker build . -t <image-name>` : create image
* `docker run -it -p 22:22 --name <container-name> <image-name>` : create container
* `docker container start <container-name>` : start container
* `docker exec -it <container-name> bash` : access container's terminal
* `ssh -L 5900:localhost:5900 <username>@$(ipconfig getifaddr en0) "x11vnc -create -nopw -listen 127.0.0.1 -localhost"` : start vnc server
* `startxfce4` : start GUI
* `bash ~/VEXU_GHOST/scripts/launch_sim.sh` : launch simulation

  --mount type=bind,source="$(pwd)"/VEXU_GHOST,target=/root/VEXU_GHOST

  docker run -it -p 22:22 --mount type=bind,source="$(pwd)",target=/root/VEXU_GHOST --name LIDAR2 ros2-foxy2

## Setup Steps
1. Build the Dockerfile by navigating to this directory and running the command `docker build . -t ros2-foxy`.
2. Create a container from the image using `docker run -it -p 22:22 --name LIDAR ros2-foxy`. You're now running bash in the container. Type `exit` to stop. Note that `-p 22:22` maps the container's port 22 to your local port 22; only one container at a time can be bound to your local port 22.
    * The docker file will create a user for you that defaults to the username `lidar` with password `password`. To change these defaults, add `--build-arg USERNAME=<your-username>` and/or `--build-arg PASSWORD=<your-password>` to the `docker build` line.
3. Start the container via the docker GUI or by running `docker container start LIDAR` and then _on your local machine_ run the command `ssh -L 5900:localhost:5900 lidar@$(ipconfig getifaddr en0) "x11vnc -create -nopw -listen 127.0.0.1 -localhost"` to establish a vnc server in the container and pipe it to localhost:5900. Unless you changed it during the previous step, the password is "password".
    * If you changed your username in step 2, change "lidar" to your username.
    * It is annoying to copy and paste this command every time so it's helpful to alias it by adding `alias lidarvnc="ssh -L 5900:localhost:5900 lidar@$(ipconfig getifaddr en0) \"x11vnc -create -nopw -listen 127.0.0.1 -localhost\""` to your shell initialization script.
4. Use your VNC Viewer to connect with `localhost:5900` and type `startxfce4` in the terminal that appears; a GUI should form.
5. Open a new terminal and run `sudo -i` to launch a root session and navigate to `/root/VEXU_GHOST/scripts`. Run `bash launch_sim.sh` and the rviz simulation should start up!
    * You may need to change from world frame to base link in the dropdown in the upper left-hand corner for the simulation to work properly. Ideally, the world frame will be created in the future.

## Development Environment
It's a pain in the ass to write code on the container and move it to back onto your local machine – especially without an IDE on the container. I never want to subjugate you to that, so we'll use a very handy VScode (sorry IntelliJ people) extension called [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers). Add the extension, then click the green "><" in the lower left corner of VScode and select "Attach to Running Container..." then select the container your using. Simple as that. Now on your machine, you can edit code in the container and it will update in real-time! Basically, we just use the container for running the simulation and nothing else – write code locally.

## Troubleshooting
* Running `xeyes` (and having some eyes follow your mouse around the scene) is a good way to check basic x11 forwarding is working.
* If get "WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED!" when trying to ssh into the container, you may need to remove the known_host files in your local .ssh directory. Note that this should only happen if you change up your container and you should take caution when doing this especially if you ssh into other devices. Do some research first.
* If you get the error "fatal: mot a git repository", navigate to the VEXU_GHOST folder and run "git init" and then "git submodule add https://github.com/MaxxWilson/amrl_shared_lib.git ghost_estimation/src/shared". This error happens when git initializes in a funny manner.

## Interesting Links & Explanations
* Here is an explanation of our VNC setup that may help you work through issues <https://jasonmurray.org/posts/2021/x11vnc/>
* When we VNC into a user account, we only give that user x11 forwarding authorization and have to essentially copy the credentials from the user to root so we can do the simulation from root. This is all done programmatically now, but this link covers how it is done manually <https://unix.stackexchange.com/questions/476290/x11-forwarding-does-not-work-if-su-to-another-user>