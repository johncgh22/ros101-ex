/* Realizar Operaciones Matriciales con Eigen Library */

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(int argc, char* argv[]){
    ros::init(argc,argv,"matrix_node");   // Inicializacion del Nodo
    Vector2d X;                           // Definicion Vector 2x1
    X << -5, 3;                           // Valores del Vector "X"
    Vector2d Q(2,8);
    cout << "X=\n" << X << "\n\n";
    cout << "Q=\n" << Q << "\n\n\n\n";

    Matrix2d R;                           // Definicion Matriz 2x2
    MatrixXf T(3,2);                      // Definicion Matriz 3x2
    R << 2,3,                             // Valores de la Matriz "R"
         0,7;
    R(0,0) = 1;
    cout << "R=\n" << R << "\n\n";
    T.row(0) << 1,0;
    T.row(1) << 0,33;
    T.row(2) << 0,33;
    T.col(1) << 0,1,9;
}
