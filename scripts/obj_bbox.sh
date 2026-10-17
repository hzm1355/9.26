#!/bin/bash
awk '/^v / {x=$2;y=$3;z=$4;
  if(minx==""||x<minx)minx=x; if(x>maxx)maxx=x;
  if(miny==""||y<miny)miny=y; if(y>maxy)maxy=y;
  if(minz==""||z<minz)minz=z; if(z>maxz)maxz=z}
END {printf "X: %.2f .. %.2f (span %.2f)\nY: %.2f .. %.2f (span %.2f)\nZ: %.2f .. %.2f (span %.2f)\n",
  minx,maxx,maxx-minx, miny,maxy,maxy-miny, minz,maxz,maxz-minz}' \
  ~/v2x_models/room_33/meshes/room_33.obj
