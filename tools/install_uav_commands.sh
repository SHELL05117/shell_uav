#!/usr/bin/env bash
# Run from this repository's tools directory on RK; preserve local configuration.
set -eo pipefail
uav_tools="$(cd -- "$(dirname -- "$0")" && pwd)"
uav_dest="$HOME/.local/share/uav"
mkdir -p "$uav_dest" "$HOME/.local/bin" "$HOME/.config/uav"
install -m 755 "$uav_tools/uav" "$uav_tools/uav_start.py" "$uav_tools/uav_log_lite.py" "$uav_tools/uav_arm.py" "$uav_dest/"
install -m 644 "$uav_tools/uav_env.sh" "$uav_dest/"
ln -sfn "$uav_dest/uav" "$HOME/.local/bin/uav"
python3 - <<'PY'
import os, re, shutil, time
path=os.path.expanduser('~/.bashrc')
with open(path) as f: text=f.read()
line='source "$HOME/.local/share/uav/uav_env.sh"'
updated=re.sub(r'^export ROS_(?:IP|MASTER_URI)=.*\n','',text,flags=re.M)
# Remove only the known original fixed setup lines; preserve other customization.
for original in ('source /opt/ros/melodic/setup.bash',
                 'source /home/jcfh/catkin_ws/devel/setup.bash'):
    updated=updated.replace(original+'\n','')
if line not in updated: updated+='\n# SHELL UAV unified ROS environment\n'+line+'\n'
if updated!=text:
    backup=path+'.uav-'+time.strftime('%Y%m%d-%H%M%S')
    shutil.copy2(path,backup)
    with open(path,'w') as f: f.write(updated)
    print('Previous .bashrc saved: '+backup)
PY
echo 'Installed uav. New SSH terminals load the unified environment automatically.'
