sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y build-essential cmake cmake-curses-gui pkg-config
sudo apt-get install -y libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev libavcodec-dev libavformat-dev libswscale-dev libeigen3-dev libxvidcore-dev libx264-dev libgtk2.0-dev

sudo apt-get install -y libv4l-dev v4l-utils
sudo modprobe bcm2835-v4l2

wget -O opencv.zip "https://github.com/opencv/opencv/archive/3.4.1.zip"
wget -O opencv_contrib.zip "https://github.com/opencv/opencv_contrib/archive/3.4.1.zip"
unzip opencv.zip
unzip opencv_contrib.zip
cd opencv
mkdir build
cd build
cmake ..
make
sudo make install

cd ~
git clone https://github.com/cedricve/raspicam
cd raspicam
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
