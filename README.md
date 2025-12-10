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
