# How I made this dir

## 1 .Building the ROS directory using `catkin_make`
```bash
cd ~
mkdir -p myscara/src # -p option also creates the parent directories
cd myscara
catkin_make
echo "source ~/.myscara/devel/setup.bash" >> ~/.bashrc # Adds workspace to search path
```

## 2. Create ROS packages using catkin_create_pkg
```bash
cd ~/myscara/src
catkin_create_pkg myscara_gazebo
```


