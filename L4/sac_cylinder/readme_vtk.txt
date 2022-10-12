Ubuntu 20.04 + ROS + PCL 1.8.1 + VTK 7.1.1

//https://icode.best/i/80586946520052

//====================//
//  VTK Installation
//====================//


//=============//
//  QT4
//=============//
//https://askubuntu.com/questions/1274134/cannot-install-qt-4-on-ubuntu-20-04-quite-universal-circuit-simulator-qucs

The Qt4 framework has been removed from Ubuntu 20.04 main repository.
You can still get Qt4 libraries, adding the PPA rock-core/qt4
Run in a terminal:

sudo add-apt-repository ppa:rock-core/qt4

sudo apt update

And install the required Qt4 libraries by running command:

sudo apt install qt4-dev-tools libqt4-dev libqt4-core libqt4-gui

or

sudo apt install qt4-dev-tools libqt4-dev libqtcore4 libqtgui4


//=============//
//  CMakeLists
//=============//
//https://vtk.org/Wiki/VTK/Tutorials/CMakeListsFile