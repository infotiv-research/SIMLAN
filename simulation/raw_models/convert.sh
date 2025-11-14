#!/bin/bash

# Takes the output from the blender script and reformats it into the structure Gazebo expects
# Uses 2 arguments: [dae] [model name] [mass]
# Author: Anders Bäckelie
mkdir -p ./phobos_out/unnamed/
blender -b --python convert_helper.py -- $1 $3

outdir=../simlan_gazebo_environment/models/$2
rm -rf $outdir
mkdir -p $outdir
mkdir -p $outdir/meshes

cp phobos_out/unnamed/meshes/dae/*.dae $outdir/meshes/
cp phobos_out/unnamed/sdf/*.sdf $outdir/model.sdf

sed -i -e 's|../meshes/dae/|meshes/|g' $outdir/model.sdf
sed -i -e "s|<model name=\"unnamed\"|<model name=\"$2\"|g" $outdir/model.sdf
sed -i -e 's|version="1.9"|version="1.7"|g' $outdir/model.sdf
sed -i -e 's|<contact/>|<contact><ode>  <max_vel>0.001</max_vel><min_depth>0.01</min_depth><kd>1e15</kd>  </ode></contact> |g' $outdir/model.sdf

echo "<?xml version=\"1.0\" ?>
<model>
    <name>$2</name>
    <version>1.0</version>
    <sdf version=\"1.7\">model.sdf</sdf>
    <author>
        <name>SIMLAN project</name>
    </author>
    <description>Auto-generated sdf model file for $2.</description>
</model>" > $outdir/model.config

rm -rf ./phobos_out/unnamed/


