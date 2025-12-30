# Smart Factoring Simulation Scene

This folder contains the 3D simulation environment for the Smart Factory cell, used with **NVIDIA Isaac Sim** and integrated with **ROS 2 Jazzy**.

---

## 📁 Folder Structure

- `assets/`: Contains custom meshes, materials, and static components (e.g. walls, table, conveyor)
- `scenes/`: Main `.usd` files used for the simulation in Isaac Sim
- `scripts/`: Python scripts used to generate or configure the scene

---


## 🖼️ Scene Preview

![Scene Preview](./smartscene.png)

> *This image shows the core elements of the simulation: robot, workspace, and factory layout.*

---

## How to Open in Isaac Sim

### 1. Run Isaac Sim:

```bash
cd isaacsim/_build/linux-x86_64/release
./isaac-sim.sh
```
### 2. In the Isaac Sim menu, go to File → Open
### 3. Open the scene:
```bash
smartfactoring_isaac/scenes/smartfactoring_world.usda
```

## Notes

- The scene must be opened manually in Isaac Sim via the UI or command line
- No Action Graphs are included — all control is handled via ROS 2

## 🤖 Isaac Sim Installation

Isaac Sim is not included in this repository due to its large size and NVIDIA licensing policies.


### 1. Install Dependencies 



```bash
sudo apt update
sudo apt install -y git git-lfs build-essential gcc-11 g++-11

# Set GCC 11 as default
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 200
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 200

```
### 2. Clone the Isaac Sim Repository
To ensure compatibility with this project, clone and checkout the exact version used during development:

```bash
cd smartfactoring_jazzy
git clone https://github.com/isaac-sim/IsaacSim.git isaacsim
cd isaacsim
git lfs install
git lfs pull 
```
### 3. Build

Confirm that GCC/G++ 11 is in use:

```bash
gcc --version
g++ --version
```
```bash
./build.sh
```
### 4. Run Isaac Sim
⚠️ Startup Time The first time loading Isaac Sim may take up to several minutes as Extensions and Shader are loaded and cached. The subsequent startup time should be in the ranges of 10-30 seconds depending on hardware configuration.

Navigate to the corresponding binary directory for your platform and run the executable.

```bash
cd _build/linux-x86_64/release
./isaac-sim.sh
```
For more detailed documentation and advanced usage, visit the official Isaac Sim repository:

🔗 [https://github.com/isaac-sim/IsaacSim](https://github.com/isaac-sim/IsaacSim)
