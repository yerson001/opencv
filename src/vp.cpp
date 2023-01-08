#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<iostream>
#include <opencv2/features2d/features2d.hpp>
//colores
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
//************************
//          imagen
//    -------------------> +x
//   |-|-|-|-|-|-|-|-|-|-|
//   |-|-|-|-|-|-|-|-|-|-|
//   |-|-|-|-|-|-|-|-|-|-|
//   |-|-|-|-|-|-|-|-|-|-|
//   | +y
//*************************

//using namespace cv;

#define debug(a) std::cout<<#a<<" = "<<a<<std::endl;

int sigma = 1;
int tam = 5 * sigma;

cv::Mat vec_mascara_der;
cv::Mat vec_mascara;
cv::Mat gx, gy, mag, ori, contornos;
cv::Mat votacion, votacionRecta, orientacion;
cv::Mat Rho;
cv::Mat Theeta;
int rectasRegistradas(0);
float umbral = 30;

using namespace cv;


cv::Mat ResizeImage(cv::Mat &image, float scale)
{
    cv::Mat resized;
    resize(image, resized, cv::Size(image.cols*scale, image.rows*scale), cv::INTER_LINEAR);
    return resized;
}

/**
 * Funcion que calcula el valor de gaussiana correspodiente al valor x
 */
float gaussiana(int x) {
    return (1 / (sigma * sqrt(2 * M_PI))) * (exp(-0.5 * (pow((x) / sigma, 2)))); //* (0.39 / 0.1621);
}

/**
 * Funcion que calcula la derivada de la gaussiana correspondiente al valor x
 */
float derivGaussiana(int x) {
    return ((-x / pow(sigma, 2)) * (exp(-pow(x, 2) / 2 * pow(sigma, 2))));
}

/**
 * Metodo que calcula los vectores máscara que se utilizan para calcular el gradiente
 */
void mascara() {
    int aux = (tam / 2);
    int j = 0;
    //Se calcula la gaussiana y su derivada para un vector del tamaño elegido
    for (int i = aux; i >= -aux; i--) {
        float gaus = gaussiana(i);
        float der = derivGaussiana(i);
        vec_mascara_der.at<float>(0, j) = der;
        vec_mascara.at<float>(j, 0) = gaus;
        j++;
    }
}


/*
 * Se registra una recta
 */
void registrar_recta(float theta, float rho) {
    Rho.at<float>(rectasRegistradas, 0) = rho;
    Theeta.at<float>(rectasRegistradas, 0) = theta;
    rectasRegistradas++;
}

// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2, Point2f &r) {
    Point2f x = o2 - o1;
    Point2f d1 = p1 - o1;
    Point2f d2 = p2 - o2;

    float cross = d1.x * d2.y - d1.y * d2.x;
    if (abs(cross) < /*EPS*/1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x) / cross;
    r = o1 + d1 * t1;
    return true;
}

/*
 * Se gerena una interseccion entre 2 rectas aletatorias si su orientacion no es similar y
 * se guarda la interseccion como un voto si entra en la imagen
 */
void generar_votar_interseccion(Mat image, Mat img) {
    int recta1 = rand() % rectasRegistradas;
    int recta2 = rand() % rectasRegistradas;
    if (recta1 != recta2) {
        float rho1 = Rho.at<float>(recta1, 0);
        float theta1 = Theeta.at<float>(recta1, 0);
        float rho2 = Rho.at<float>(recta2, 0);
        float theta2 = Theeta.at<float>(recta2, 0);
        //miras si las oreientaciones no se parecen mucho
        float a = theta1;
        float b = theta2;
        if (b < 0)
            b += CV_PI;
        if (a < 0)
            b += CV_PI;
        if (abs(a - b) > 0.35) {
            //se hace la interseccion si su orientacion no se parece
            int y1 = 0;
            int x1 = (rho1 - (y1 * sin(theta1))) / cos(theta1);
            int y2 = 512;
            int x2 = (rho1 - (y2 * sin(theta1))) / cos(theta1);

            int y3 = 0;
            int x3 = (rho2 - (y3 * sin(theta2))) / cos(theta2);

            int y4 = 512;
            int x4 = (rho2 - (y4 * sin(theta2))) / cos(theta2);

            Point2f inters;
            if (intersection(Point2f(x1, y1), Point2f(x2, y2), Point2f(x3, y3),
                    Point2f(x4, y4), inters)) {

                float i = inters.x + image.cols / 2;
                float j = (image.rows / 2) - inters.y;
                if (i >= 0 && i < image.cols && j >= 0 && j < image.rows) {

                    votacionRecta.at<short>(i, j) += 1;
                }
            }

        }

    }

}

int main()
{
    cv::Mat frame,image;
    //Read video with frames
    cv::VideoCapture cap;

    cap.open("/home/yrsn/Videos/video_.mp4");

    if(!cap.isOpened())
    {
        std::cout<<"No video stream detected"<<std::endl;
        system("pause");
        return -1;
    }


    if(cap.isOpened())
    {
        while(1)
        {
            cap >> frame;



            int lowH = 0; int highH = 26;
            int lowS = 98; int highS = 118;
            int lowV = 71;  int highV = 183;

            //canny
            Mat oriImageGray;
            //imshow("recibido",oriImage);
            cvtColor(frame, oriImageGray, COLOR_RGB2GRAY);
            //Canny(oriImageGray, edgeImage, 100, 150, 3);
            Mat imgHSV;

            cvtColor(frame, imgHSV, COLOR_BGR2HSV);
            inRange(imgHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), image);
            cv::imshow("image",ResizeImage(image,0.5));



            cvtColor(frame, image, cv::COLOR_BGR2GRAY);

            vec_mascara_der = cv::Mat::zeros(1, tam, CV_32F);
            //vec_mascara_der = [0, 0, 0, 0, 0]
            //debug(vec_mascara_der);








            vec_mascara = cv::Mat::zeros(tam, 1, CV_32F);
            //            vec_mascara = [0;
            //             0;
            //             0;
            //             0;
            //             0]
            //debug(vec_mascara);

            // imagen filas = 720  columnas = 1280, canal = 1
            gx = cv::Mat::zeros(image.rows, image.cols, image.type());
            gy = cv::Mat::zeros(image.rows, image.cols, image.type());
            mag = cv::Mat::zeros(image.rows, image.cols, CV_32F);
            ori = cv::Mat::zeros(image.rows, image.cols, CV_32F);
            //            debug(gx.channels());
            //            debug(mag.channels());
            mascara();

            sepFilter2D(image, gx, CV_16S, vec_mascara_der, vec_mascara);
            sepFilter2D(image, gy, CV_16S, vec_mascara.t(),-vec_mascara_der.t());

            //            cv::imshow("gx",gx);

            //            cv::imshow("gy",gy);

            for (int i = 0; i < gx.rows; i++) {
                for (int j = 0; j < gx.cols; j++) {

                    mag.at<float>(i, j) = sqrt(pow(gx.at<short>(i, j), 2)+ pow(gy.at<short>(i, j), 2));

                    ori.at<float>(i, j) = (atan2(gy.at<short>(i, j),gx.at<short>(i, j)));
                }
            }
            //            cv::imshow("mag",mag);
            //            cv::imshow("ori",ori);

            votacionRecta = cv::Mat::zeros(image.cols, image.rows, CV_16S);
            Rho = cv::Mat::zeros(image.rows*image.cols, 1, CV_32F);// * image.cols
            Theeta = cv::Mat::zeros(image.rows * image.cols, 1, CV_32F);//* image.cols

            //            cv::imshow("Rho",Rho);
            //            cv::imshow("Theeta",Theeta);
            rectasRegistradas = 0;

            int x;
            int y;
            float theta;
            float rho;
            int ncols = image.cols;
            int nrows = image.rows;


            for (int i = 0; i < image.rows; i++) {
                for (int j = 0; j < image.cols; j++) {
                    if (mag.at<float>(i, j) >= umbral) {
                        x = j - (ncols / 2);
                        y = (nrows / 2) - i;
                        theta = ori.at<float>(i, j);
                        rho = (x * cos(theta)) + (y * sin(theta));

                        if (!(theta <= CV_PI + 0.20 && theta >= CV_PI - 0.20)
                                && !(theta <= 0 + 0.20 && theta >= 0 - 0.20)
                                && !(theta <= -CV_PI + 0.20
                                     && theta >= -CV_PI - 0.20)
                                && !(theta <= CV_PI / 2 + 0.20
                                     && theta >= CV_PI / 2 - 0.20)
                                && !(theta <= 3 * CV_PI / 2 + 0.20
                                     && theta >= 3 * CV_PI / 2 - 0.20)) {

                            //dibujar recta

                            registrar_recta(theta, rho);
                        }
                    }
                }
            }

            //cv::imshow("Rho",ResizeImage(Rho,0.3));
            //cv::imshow("Theeta",ResizeImage(Theeta,0.5));

            double min1, max1;
            Point minLugar, maxLugar;

            for (int i = 0; i < rectasRegistradas * log(rectasRegistradas);
                 i++) {
                generar_votar_interseccion(image, frame);
            }


            /*
             * Se obtiene el punto mas intersectado
             */


            minMaxLoc(votacionRecta, &min1, &max1, &minLugar, &maxLugar);
            line(frame, Point(maxLugar.y - 50, maxLugar.x - 50),Point(maxLugar.y + 50, maxLugar.x + 50),Scalar(255, 0, 255), 4);
            line(frame, Point(maxLugar.y - 50, maxLugar.x + 50),Point(maxLugar.y + 50, maxLugar.x - 50),Scalar(255, 0, 255), 4);



            cv::imshow("frame",ResizeImage(image,0.5));
             cv::imshow("fram",ResizeImage(frame,0.5));
            if(cv::waitKey(1)==27)
            {
                break;
            }
        }
    }




    std::cout<<"hello"<<std::endl;
    return 0;
}
