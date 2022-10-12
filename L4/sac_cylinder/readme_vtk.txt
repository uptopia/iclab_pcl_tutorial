// VTK CMakeLists
https://vtk.org/Wiki/VTK/Tutorials/CMakeListsFile

//VTK相機控制
https://blog.51cto.com/u_15346339/3670692

//VTK畫圓柱體 
1. 圓柱體點雲投影到軸心上
2. 起始點、終點、軸心點

https://blog.csdn.net/qq_44575789/article/details/126225441
https://blog.csdn.net/stanshi/article/details/124773320?spm=1001.2101.3001.6650.1&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-124773320-blog-126225441.pc_relevant_3mothn_strategy_and_data_recovery&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-124773320-blog-126225441.pc_relevant_3mothn_strategy_and_data_recovery&utm_relevant_index=2
=> PCA


//求三维空间中一点在直线上的投影点
https://www.csdn.net/tags/NtjaYgxsNjY4NTYtYmxvZwO0O0OO0O0O.html
已知：
待投影点的坐标 P=（xp,yp,zp）
直线L的方程（点法式）：(x-a)/d=(y-b)/e=(z-c)/f
直线L上一点O的坐标：（a,b,c） 法向量：（d,e,f）

待求：
待投影点P1在直线L上的投影点M的坐标（xm,ym,zm）

计算：
1）O和P构成法向量OP（xp-a,yp-b,zp-c）
2) 求参数t=（d(xp-a)+e(yp-b)+f(zp-c)）/( dd+ee+ff )
3）直线L的参数方程为 (x-a)/d=(y-b)/e=(z-c)/f=t
那么a b c d e f都是已知的，t也算出来了，就可以得到投影点M的坐标（xm,ym,zm）
xm=dt+a
ym=et+b
zm=ft+c

//VTK render方式改成如同PCL Viewer
https://www.twblogs.net/a/5b8aa38a2b71775d1ce834ed

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