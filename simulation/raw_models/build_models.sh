#!/bin/bash

# python3 ./aruco_generator.py

# resulting model name should NOT have '-', replace '-' with '_'
./convert.sh ./objects/meshes/box_1185x785x1010-Body.dae clean_box 250
./convert.sh ./objects/meshes/eur-pallet.dae clean_pallet 20

# NOTE: the collision for this box is defined manually and hardcoded. Be careful not to overwrite it.
# ONLY uncomment this if you want to rebuild the visual .dae mesh

#./convert.sh ./objects/meshes/box_1185x785x1010-Body.dae box_1185x785x1010 250
#./convert.sh ./objects/meshes/shelf.dae shelf 40
#./convert.sh ./objects/meshes/eur-pallet.dae eur_pallet 20