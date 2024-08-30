# Smart Factory Package

A **Dockerfile** and **devcontainer.json** file were developed to development and deploy of the project inside a ROS Humble container. Dependencies and configurations from the workspace are already handled by docker.

### Customization of enviroment

If you want to customize your Development Enviroment, modify the *"customizations"* in **devcontainer.json**, and add *"devcontainer.json"* to **.gitignore** to avoid conflicts.

### Build the package
After opening the container in DevContainer, follow these steps to build the ROS Humble package.
```
cd smartfactory_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --executor sequential
