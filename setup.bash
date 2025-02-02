cd ~
cd .gazebo
if [ -d "models" ]; then
    echo "ok"
else
    mkdir models
    echo "A pasta 'models' foi criada"
fi
cd ~/catkin_ws/src/eletroquad
cd models && cp -r * ~/.gazebo/models/

