#!/bin/bash

# Abilita l'accesso al server grafico locale
xhost +local:root

# Percorso assoluto della cartella workspace
WORKSPACE_DIR=$(cd ../workspace && pwd)

echo "Avvio container con workspace montato in: $WORKSPACE_DIR"
echo "Modalità: CPU (Software Rendering) - Override OpenGL 3.3"

docker run -it --rm \
    --name px4_container \
    --net=host \
    --privileged \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="LIBGL_ALWAYS_SOFTWARE=1" \
    --env="MESA_GL_VERSION_OVERRIDE=3.3" \
    --env="MESA_GLSL_VERSION_OVERRIDE=330" \
    --env="GZ_SIM_RENDER_ENGINE=ogre" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$WORKSPACE_DIR:/root/workspace:rw" \
    --workdir="/root/workspace" \
    px4_ros2_project \
    bash
