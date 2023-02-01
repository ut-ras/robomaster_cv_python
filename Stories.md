# Current

### Lidar Research
#### Aaditya, Leo, Samik, Tanay, Will, Hasif
Objective: We have a lot to learn
See Lidar Max Debrief.md for a chatGPT summary from our chat with Maxx. Pay attention to the links at the botton from Maxx. Understand it and be able to explain it

### Depth Learning Project 
Objective: Learn how ROS subscribers and publishers work by making one for depth calculations
There is a LIDAR publisher out there that spits out angles and distances; this is a learning project to figure out how ros works.
 * Subscribe to the published and publish the distance to whats right in front of you.
 * Show what you did to team (most imporant)
Here is how to run a demo subscriber and publisher that are already installed via the docker file or can be installed pretty easily:
https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html#try-some-examples

### Run the Sim
Objective: Be able to use the simulation
Learn the ins and outs of the simulation.
 * How do I tell robot where to go. Whats it doing on the backend. As much of that jazz as you can.
 * Show what you did to team (most imporant)

### Make Robotics Field
Objective: Have our field in the simulation
We need to add our robomasters field into the simulation
An example of how the simulation wants the world can be found at `VEXU_GHOST/ghost_sim/urdf/spin_up.world` (it is in meters)

### Mecanum in Sim
Objective: Get _roughly_ our robot in the simulation with mecanum wheels
This is top priority and whill be necessary for all our tuning. Maxx put some stuff about this in discord so read over that and look into the plugin he found. Implement it or find a better solution.
Link to discord message:
https://discord.com/channels/785324407606083625/1053837858970017843/1069326413007691816
Link to plugin:
https://github.com/qaz9517532846/gazebo_mecanum_plugins/tree/ros2-foxy

# Old