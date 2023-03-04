cd ~/VEXU_GHOST

# apt install -y dos2unix
# find /root/VEXU_GHOST -type f -exec dos2unix {} +
# chmod -R +x /root/VEXU_GHOST

mkdir /root/VEXU_GHOST/ghost_sim/rviz
bash /root/VEXU_GHOST/scripts/install_dependencies.sh

bash /root/VEXU_GHOST/scripts/setup_paths.sh

source /ros_entrypoint.sh
bash /root/VEXU_GHOST/scripts/build.sh

echo "source /ros_entrypoint.sh" >> ~/.bashrc
echo "source /root/VEXU_GHOST/install/setup.bash" >> ~/.bashrc

cd ~/VEXU_GHOST
rm -r .git