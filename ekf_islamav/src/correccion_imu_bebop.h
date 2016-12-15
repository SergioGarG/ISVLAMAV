//include de la parte de corrección de imu bebop

#include <string>
#include <vector>
#include <ostream>
#include <iostream>
#include <string>
#include <math.h>
#include <eigen3/Eigen/Dense>
using namespace std;
using Eigen::MatrixXd;

class corrector_imu_bebop
{
public:

	//Variables por las que se define
	double estado_correccion_imu[10];
	double varianza_correccion_imu[10][10];

	void corregir_imu_bebop(double media_anterior[10], double covarianza_anterior[10][10], double z[6])
	{
		int i, j;
		
		Eigen::VectorXf media_ant(10);
		Eigen::MatrixXf covarianza_ant(10,10);
		for(i=0;i<10;i++)
		{
			media_ant(i)=media_anterior[i];
			for(j=0;j<10;j++)	covarianza_ant(i,j)=covarianza_anterior[i][j];
		}
		Eigen::VectorXf z_imu_eigen(6);
		for(j=0;j<6;j++)	z_imu_eigen(j)=z[j];
		
		Eigen::VectorXf estado_correccion_imu_eigen(10);
		Eigen::MatrixXf varianza_correccion_imu_eigen(10,10);
		
		//Defino los factores que voy a utilizar en adelante
		double roll=media_anterior[6];
		double pitch=media_anterior[7];
		double yaw=media_anterior[8];
		double vx=media_anterior[3];
		double vy=media_anterior[4];

		//Los valores de esta matriz deberían ser revisados
		Eigen::MatrixXf Qt_imu(4,4);
		Qt_imu << 	0.2, 0, 0, 0, 0, 0,
						0, 0.2, 0, 0, 0, 0,
						0, 0, 0.2, 0, 0, 0,
						0, 0, 0, 0.2, 0, 0,
						0, 0, 0, 0, 0.2, 0,
						0, 0, 0, 0, 0, 0.2;

		//Para seleccionar vx, vy, vz, roll, pitch y yaw
		Eigen::MatrixXf C(6,10);
		C << 	0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, cos(yaw), sin(yaw), 0, 0, 0, -vx*sin(yaw)+vy*cos(yaw), 0,
				0, 0, 0, -sin(yaw), cos(yaw), 0, 0, 0, -vx*cos(yaw)-vy*sin(yaw), 0,
				0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
		Eigen::MatrixXf C_t(10,6);
		C_t=C.transpose();

		//Matriz identidad para el cálculo de la nueva covarianza
		Eigen::MatrixXf I(10,10);
		I << 	1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

		Eigen::VectorXf hu(6);
		hu << vx*cos(yaw)-vy*sin(yaw), vx*sin(yaw)+vy*cos(yaw), media_anterior[2], roll, pitch, yaw;

		//Cálculo de la ganancia de Kalman
		//K=covarianza_ant*H'*inv(H*covarianza_ant*H'+Qt_imu);
		
		Eigen::MatrixXf K(10,6);
		Eigen::MatrixXf Inversa(6,6);
		
		Inversa=C*covarianza_ant*C_t + Qt_imu;
		K=covarianza_ant*C_t*(Inversa.inverse());

		//Cálculo del nuevo estado
		//hu=[media_ant(2) roll pitch yaw]';
		//media_act=media_ant+K*(z-hu);
		
		estado_correccion_imu_eigen=media_ant + K*(z_imu_eigen - hu);
		
		//Cálculo de la nueva covarianza
		//aux=K*H;
		//covarianza_act=(eye(size(aux))-aux)*covarianza_ant;

		varianza_correccion_imu_eigen=(I - K*C)*covarianza_ant;
		
		for(i=0;i<10;i++)
		{
			estado_correccion_imu[i]=estado_correccion_imu_eigen(i);
			for(j=0;j<10;j++)	varianza_correccion_imu[i][j]=varianza_correccion_imu_eigen(i,j);
		}
	}


};
