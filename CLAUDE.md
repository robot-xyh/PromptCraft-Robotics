# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

PromptCraft-Robotics is a Microsoft research project for testing and sharing LLM prompts in the robotics domain. It includes a ChatGPT-AirSim integration that allows natural language control of drones in a simulated environment.

## Setup Commands

```bash
# Create and activate conda environment
conda env create -f chatgpt_airsim/environment.yml
conda activate chatgpt

# Install AirSim client (required after environment setup)
pip install airsim
```

## Running the Simulator

1. Download AirSim binary from GitHub Releases and unzip
2. Copy `chatgpt_airsim/settings.json` to `C:\Users\<username>\Documents\AirSim\`
3. Run AirSim simulation: `.\run.bat` (from sim folder)
4. Run the interface: `python chatgpt_airsim/chatgpt_airsim.py`

Custom prompts: `python chatgpt_airsim/chatgpt_airsim.py --prompt <path> --sysprompt <path>`

Alternative interface with configurable API endpoint: `python chatgpt_airsim/claude-airsim.py`

## Architecture

### Core Components

**chatgpt_airsim/chatgpt_airsim.py** - Main OpenAI interface that:
- Maintains chat history with the LLM
- Extracts Python code blocks from LLM responses using regex
- Executes extracted code via `exec()` in the AirSim context
- The `aw` variable (AirSimWrapper instance) is available in execution scope

**chatgpt_airsim/claude-airsim.py** - Alternative interface supporting configurable API endpoints via `BASE_URL` in config.json. Supports both `OPENAI_API_KEY` and `ANTHROPIC_API_KEY`.

**chatgpt_airsim/airsim_wrapper.py** - High-level drone control API:
- Wraps low-level AirSim MultirotorClient calls
- Navigation: `takeoff()`, `land()`, `fly_to([x,y,z])`, `fly_path(points)`, `set_yaw()`, `get_yaw()`, `get_drone_position()`, `get_position(object_name)`
- Camera: `get_image()`, `get_depth_image()`, `save_image()`, `set_camera_orientation()`, `get_camera_orientation()`
- Contains `objects_dict` mapping friendly names to Unreal Engine actor names

### Available Scene Objects

`turbine1`, `turbine2`, `solarpanels`, `car`, `crowd`, `tower1`, `tower2`, `tower3`

### Camera Names

`front_center`, `front_right`, `front_left`, `bottom_center`, `back_center` (or "0" to "4")

### Prompt Structure

- **system_prompts/** - System prompts defining LLM behavior constraints
- **prompts/** - User prompts containing available function documentation and scene object names

The LLM is constrained to only use wrapper functions defined in prompts, not raw AirSim API.

### Coordinate System

- Forward = +X axis
- Right = +Y axis
- Up = +Z axis (wrapper handles AirSim's inverted Z internally)

## Configuration

`chatgpt_airsim/config.json` stores API credentials and settings:
- `OPENAI_API_KEY` - OpenAI API key
- `ANTHROPIC_API_KEY` - Alternative key field (used by claude-airsim.py)
- `BASE_URL` - Custom API endpoint (optional, for claude-airsim.py)

## Examples Directory

The `examples/` folder contains markdown documentation of prompt examples across domains:
- aerial_robotics/ - Drone inspection and navigation
- manipulation/ - Robot arm tasks
- embodied_agents/ - Visual navigation
- spatial_temporal_reasoning/ - Visual servoing
