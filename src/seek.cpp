#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    // Matriz de homografía
    Mat H = (Mat_<double>(3, 3) << 1.26143636, -0.17429638, -154.63456613,
                                   0.11861404, 1.12960584, -80.40128631,
                                   0.000234, 0.0004209, 1);
    
    // Matrices de puntos de referencia en 3D y 2D
    Mat points3D = (Mat_<double>(4, 3) << 0, 0, 0,
                                          1, 0, 0,
                                          0, 1, 0,
                                          0, 0, 1);
    Mat points2D = (Mat_<double>(4, 2) << 500, 200,
                                          600, 300,
                                          700, 200,
                                          600, 100);

    // Filtrar las soluciones de la descomposición de la matriz de homografía
    vector<Mat> Rs, Ts, normals;
    filterHomographyDecompByVisibleRefpoints(H, points3D, points2D, Rs, Ts, normals);
    
    // Imprimir las soluciones
    for (int i = 0; i < Rs.size(); i++) {
        cout << "Solución " << i + 1 << ":" << endl;
        cout << "Rotación:" << endl << Rs[i] << endl;
        cout << "Traslación:" << endl << Ts[i] << endl;
        cout << "Normal:" << endl << normals[i] << endl;
    }

    return 0;
}
