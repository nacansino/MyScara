# How I made this dir

## Building the ROS directory using `catkin_make`
```bash
cd ~
mkdir -p myscara/src # -p option also creates the parent directories
cd myscara
catkin_make
echo "source ~/.myscara/devel/setup.bash" >> ~/.bashrc # Adds workspace to search path
```
