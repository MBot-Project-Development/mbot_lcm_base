# MBot LCM Base

This repo contains some essential subpackages needed by all other MBot applications running off LCM:
* [mbot_lcm_serial](mbot_lcm_serial): LCM-to-serial communication with the MBot control board.
* [mbot_msgs](mbot_msgs): Message type definitions for LCM.

## Build instructions

```bash
mkdir build && cd build
cmake ..
make
```

You will likely want to install the MBot message types so they can be used across the system for other packages. To do this, do `sudo make install`.

## Uninstalling

You can uninstall with:
```bash
cd build
sudo xargs rm < install_manifest.txt
```
