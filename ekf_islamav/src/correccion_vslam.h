//include de la parte de corrección de vslam

#include <string>
#include <vector>
#include <ostream>
#include <iostream>
#include <string>
#include <math.h>
#include <eigen3/Eigen/Dense>
using namespace std;
using Eigen::MatrixXd;

class corrector_vslam
{
public:

	//Variables por las que se define
	double estado_correccion_vslam[10];
	double varianza_correccion_vslam[10][10];

	//Funciones///////////////////////////////////////////
	
	//Función del corrector
	void corregir_vslam(double media_anterior[10], double covarianza_anterior[10][10], double z_vslam[6])
	{
		int i, j;
		
		Eigen::VectorXf media_ant(10);
		Eigen::MatrixXf covarianza_ant(10,10);
		for(i=0;i<10;i++)
		{
			media_ant(i)=media_anterior[i];
			for(j=0;j<10;j++)	covarianza_ant(i,j)=covarianza_anterior[i][j];
		}
		Eigen::VectorXf z_vslam_eigen(6);
		for(j=0;j<6;j++)	z_vslam_eigen(j)=z_vslam[j];
		
		Eigen::VectorXf estado_correccion_vslam_eigen(10);
		Eigen::MatrixXf varianza_correccion_vslam_eigen(10,10);

		//Los valores de esta matriz deberían ser revisados
		Eigen::MatrixXf Qt_vslam(6,6);
		Qt_vslam << 	0.15, 0, 0, 0, 0, 0,
						0, 0.15, 0, 0, 0, 0,
						0, 0, 0.15, 0, 0, 0,
						0, 0, 0, 0.15, 0, 0,
						0, 0, 0, 0, 0.15, 0,
						0, 0, 0, 0, 0, 0.15;

		//Para seleccionar x, y, z, roll, pitch y yaw
		Eigen::MatrixXf C(6,10);
		C << 	1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
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

		//Cálculo de la Ganancia de Kalman
		//K=covarianza_ant*C'*inv(C*covarianza_ant*C'+Qt_vslam);
		Eigen::MatrixXf K(10,6);
		Eigen::MatrixXf Inversa(6,6);

		Inversa=C*covarianza_ant*C_t + Qt_vslam;
		K=covarianza_ant*C_t*(Inversa.inverse());

		//Cálculo del nuevo estado
		//media_act=media_ant+K*(z-C*media_ant);
		
		estado_correccion_vslam_eigen=media_ant+K*(z_vslam_eigen - C*media_ant);

		//Cálculo de la nueva covarianza
		//aux=K*C;
		//covarianza_act=(eye(size(aux))-aux)*covarianza_ant;
		varianza_correccion_vslam_eigen=(I - K*C)*covarianza_ant;
		
		for(i=0;i<10;i++)
		{
			estado_correccion_vslam[i]=estado_correccion_vslam_eigen(i);
			for(j=0;j<10;j++)	varianza_correccion_vslam[i][j]=varianza_correccion_vslam_eigen(i,j);
		}
		
	}
};
