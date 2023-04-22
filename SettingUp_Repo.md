Written by Paul J. Han (Discord bornhater#7417)
Last modified: 2/11/2023 
All of these commands were done in Powershell, but theoretically should work on any other shell (i.e Git Bash, etc.)
If you have any questions or suggestions, feel free to message Hasif or Wenyu on Discord (Has02#1313 and Wenyu#3334), the CV leads for the UT Robomaster team.
Otherwise, feel free to contact me on Discord as well.

1. Go to the folder that you wish the repo to exist in using commands such as **cd** 

2. Once you are in the desired folder, type in **git clone https://github.com/ut-ras/robomaster_CV.git**
This step will allow you to clone the repo and all of its contents

3. Once you have cloned the repo, enter the folder of the repo in the terminal using the command **cd robomaster_CV**

4. Once you have entered the repo, we want to enter the branch of the subteam that you want to work on. To do this, we will enter in this command.
**git checkout -t origin/feature/subteam-name**. The **subteam-name** will be replaced by the actual name of the subteam that you wish to work on. For example,
this command would be ***git checkout -t origin/feature/depth_calculation*** if you wished to checkout the branch of the depth calculation subteam. 

What this command does is it creates a local branch with the same name as the remote branch and assigns the local branch to track the contents of the remote branch, allowing you to push and pull from the remote server and retrieve updates easily. 

For example, if I did ***git checkout -t origin/feature/subteam-name***, it would create a local branch called **feature/subteam-name** and then set the remote upstream to be **remotes/origin/feature/subteam-name**.