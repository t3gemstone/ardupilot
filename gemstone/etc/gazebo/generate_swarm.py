import os
import urllib.request
import re
import sys

BASE_URL = "https://raw.githubusercontent.com/ArduPilot/SITL_Models/master/Gazebo"
MODEL_NAME = "skywalker_x8_quad"
MODELS_DIR = "./gemstone/etc/gazebo/models"
WORLDS_DIR = "./gemstone/etc/gazebo/worlds"

os.makedirs(MODELS_DIR, exist_ok=True)
os.makedirs(WORLDS_DIR, exist_ok=True)

def download_file(url, dest):
    print(f"Downloading {url} to {dest}")
    urllib.request.urlretrieve(url, dest)

print("Fetching base model data...")
base_model_sdf_url = f"{BASE_URL}/models/{MODEL_NAME}/model.sdf"
base_model_config_url = f"{BASE_URL}/models/{MODEL_NAME}/model.config"

temp_sdf = os.path.join(MODELS_DIR, "temp_model.sdf")
temp_config = os.path.join(MODELS_DIR, "temp_model.config")

download_file(base_model_sdf_url, temp_sdf)
download_file(base_model_config_url, temp_config)

with open(temp_sdf, "r") as f:
    orig_sdf = f.read()
with open(temp_config, "r") as f:
    orig_config = f.read()

for i in range(4):
    new_model_name = f"{MODEL_NAME}_{i}"
    new_model_dir = os.path.join(MODELS_DIR, new_model_name)
    os.makedirs(new_model_dir, exist_ok=True)
    
    # Update config
    new_config = orig_config.replace(f"<name>{MODEL_NAME}</name>", f"<name>{new_model_name}</name>")
    with open(os.path.join(new_model_dir, "model.config"), "w") as f:
        f.write(new_config)
    
    # Update sdf
    new_sdf = orig_sdf.replace(f'<model name="{MODEL_NAME}">', f'<model name="{new_model_name}">')
    
    # Change ports
    port_in = 9002 + i * 10
    port_out = 9003 + i * 10
    
    new_sdf = re.sub(r'<fdm_port_in>\d+</fdm_port_in>', f'<fdm_port_in>{port_in}</fdm_port_in>', new_sdf)
    new_sdf = re.sub(r'<fdm_port_out>\d+</fdm_port_out>', f'<fdm_port_out>{port_out}</fdm_port_out>', new_sdf)
    
    with open(os.path.join(new_model_dir, "model.sdf"), "w") as f:
        f.write(new_sdf)
    
    print(f"Created model {new_model_name} with ports in:{port_in} out:{port_out}")

# Create world file
world_sdf_url = f"{BASE_URL}/worlds/skywalker_x8_quad_runway.sdf"
temp_world = os.path.join(WORLDS_DIR, "temp_world.sdf")
download_file(world_sdf_url, temp_world)

with open(temp_world, "r") as f:
    orig_world = f.read()

# Remove the original single model include
include_block_regex = r'<!-- SkyWalker X8 model -->\s*<include>\s*<pose degrees="true">.*?</pose>\s*<uri>model://skywalker_x8_quad</uri>\s*</include>'
new_world = re.sub(include_block_regex, '', orig_world, flags=re.DOTALL)

# Insert 4 models slightly separated
includes = "\n    <!-- Swarm Models -->\n"
for i in range(4):
    # offset y axis
    includes += f"""
    <include>
      <name>{MODEL_NAME}_{i}</name>
      <pose degrees="true">0 {i * 5} 0.2 0 0 90</pose>
      <uri>model://{MODEL_NAME}_{i}</uri>
    </include>
"""

new_world = new_world.replace("</world>", f"{includes}\n  </world>")

with open(os.path.join(WORLDS_DIR, f"{MODEL_NAME}_swarm.sdf"), "w") as f:
    f.write(new_world)
    
print("Created world file.")

# cleanup temp
os.remove(temp_sdf)
os.remove(temp_config)
os.remove(temp_world)
