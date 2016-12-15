//include de la parte de predicción donde se definen las clases necesarias 
//para su implementación


#include <string>
#include <vector>
#include <ostream>
#include <iostream>
#include <string>
#include <math.h>
//#include "variables.h"
#include <eigen3/Eigen/Dense>
using namespace std;
using Eigen::MatrixXd;

class predictor
{
public:

	//Variables por las que se define
	double estado_predictor[10];
	double varianza_predictor[10][10];

	//Funciones/////////////////////////////////////////////////////

	//Función del predictor
	void predecir(double media_anterior[10], double covarianza_anterior_m[10][10], double u[4])
	{

		int i, j, z;

		const double t_EKF = 0.01; //tiempo fijado de iteración
		//const double t_EKF = 1; //tiempo fijado de iteración
		
		//Constantes que modelan el vuelo del drone
		double c1=10;
		double c2=3;
		double c3=1.25;
		double c4=1;
		double c5=8.3;
		double c6=0.5;
		double c7=2.5;
		double c8=0.5;
		
		//Eigen::Matrix4d Mt;
		Eigen::MatrixXf Mt(4,4);
		Mt << 	0.4, 0, 0, 0,
				0, 0.4, 0, 0,
				0, 0, 0.4, 0,
				0, 0, 0, 0.4;
		
		Eigen::MatrixXf covarianza_anterior(10,10);
		
		for(i=0;i<10;i++)
		{
			for(j=0;j<10;j++)
			{
				covarianza_anterior(i,j)=covarianza_anterior_m[i][j];
			}
		}
		
		Eigen::MatrixXf varianza_predictor_eigen(10,10);

		//Defino los factores que voy a utilizar en adelante
		double roll=media_anterior[6];
		double pitch=media_anterior[7];
		double yaw=media_anterior[8];

		//Matrices jacobianas
		Eigen::MatrixXf G(10,10);
		G << 	1, 0, 0, t_EKF, 0, 0, 0, 0, 0, 0,
				0, 1, 0, 0, t_EKF, 0, 0, 0, 0, 0,
				0, 0, 1, 0, 0, t_EKF, 0, 0, 0, 0,
				0, 0, 0, (1-(t_EKF*c1)), 0, 0, (t_EKF*c1*c2*((-sin(roll))*sin(pitch)*cos(yaw) + cos(roll)*sin(yaw))), (t_EKF*c1*c2*cos(roll)*cos(pitch)*cos(yaw)), (t_EKF*c1*c2*(cos(roll)*sin(pitch)*(-sin(yaw)) + sin(roll)*cos(yaw))), 0,
				0, 0, 0, 0, (1-(t_EKF*c1)), 0, (t_EKF*c1*c2*((-sin(roll))*sin(pitch)*sin(yaw) - cos(roll)*cos(yaw))), (t_EKF*c1*c2*cos(roll)*cos(pitch)*sin(yaw)), (t_EKF*c1*c2*(cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw))), 0,
				0, 0, 0, 0, 0, (1-(t_EKF*c7)), 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, (1-(t_EKF*c3)), 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, (1-(t_EKF*c3)), 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 1, t_EKF,
				0, 0, 0, 0, 0, 0, 0, 0, 0, (1-(t_EKF*c5));
		
		Eigen::MatrixXf G_t(10,10);
		G_t=G.transpose();
		
		Eigen::MatrixXf V(10,4);
		V <<	0, 0, 0, 0,
				0, 0, 0, 0,
				0, 0, 0, 0,
				0, 0, 0, 0,
				0, 0, 0, 0,
				0, 0, 0, c7*c8*t_EKF,
				c3*c4*t_EKF, 0, 0, 0,
				0, c3*c4*t_EKF, 0, 0,
				0, 0, t_EKF, 0,
				0, 0, c5*c6*t_EKF, 0;

		Eigen::MatrixXf V_t(4,10);
		V_t=V.transpose();

		//Cálculo del nuevo estado_predictor
		//estado_predictor: x, y, z, vx, vy, vz, roll, pitch, yaw, vyaw
		//u: vx, vy, vyaw, vz
		estado_predictor[0]=media_anterior[0]+(t_EKF*media_anterior[3]);
		estado_predictor[1]=media_anterior[1]+(t_EKF*media_anterior[4]);
		estado_predictor[2]=media_anterior[2]+(t_EKF*media_anterior[5]);

		estado_predictor[3]=media_anterior[3] + t_EKF*c1*(c2*(cos(yaw)*cos(roll)*sin(pitch) + sin(yaw)*sin(roll))-media_anterior[3]);
		estado_predictor[4]=media_anterior[4] + t_EKF*c1*(c2*(sin(yaw)*cos(roll)*sin(pitch) - cos(yaw)*sin(roll))-media_anterior[4]);
		estado_predictor[5]=media_anterior[5]+t_EKF*c7*(c8*u[3]-media_anterior[5]);

		estado_predictor[6]=media_anterior[6]-t_EKF*c3*(c4*u[1]+media_anterior[6]);
		estado_predictor[7]=media_anterior[7]+t_EKF*c3*(c4*u[0]-media_anterior[7]);
		estado_predictor[8]=media_anterior[8]+(t_EKF*media_anterior[9]);

		estado_predictor[9]=media_anterior[9]+t_EKF*c5*(c6*u[2]-media_anterior[9]);

		//Cálculo de la nueva covarianza
		varianza_predictor_eigen=G*covarianza_anterior*G_t+V*Mt*V_t;		
		for(i=0;i<10;i++)
		{
			for(j=0;j<10;j++)   varianza_predictor[i][j]=varianza_predictor_eigen(i,j);
		}

	}

};
