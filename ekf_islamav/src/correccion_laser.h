//include de la parte de corrección de laser

#include <string>
#include <vector>
#include <ostream>
#include <iostream>
#include <string>
#include <math.h>
#include <eigen3/Eigen/Dense>
using namespace std;
using Eigen::MatrixXd;

class corrector_laser
{
public:

	//Variables por las que se define
	double estado_correccion_laser[10];
	double varianza_correccion_laser[10][10];

	//Funciones///////////////////////////////////////////
	
	//Función del corrector
	void corregir_laser(double media_anterior[10], double covarianza_anterior[10][10], double z_laser[3])
	{
		int i, j;
		
		Eigen::VectorXf media_ant(10);
		Eigen::MatrixXf covarianza_ant(10,10);
		for(i=0;i<10;i++)
		{
			media_ant(i)=media_anterior[i];
			for(j=0;j<10;j++)	covarianza_ant(i,j)=covarianza_anterior[i][j];
		}
		Eigen::VectorXf z_laser_eigen(3);
		for(j=0;j<3;j++)	z_laser_eigen(j)=z_laser[j];
		
		Eigen::VectorXf estado_correccion_laser_eigen(10);
		Eigen::MatrixXf varianza_correccion_laser_eigen(10,10);

		//Los valores de esta matriz deberían ser revisados
		Eigen::MatrixXf Qt_laser(3,3);
		Qt_laser << 	0.4, 0, 0,
						0, 0.4, 0,
						0, 0, 0.4;

		//Para seleccionar x, y, z, roll, pitch y yaw
		Eigen::MatrixXf C(3,10);
		C << 	1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 1, 0;	
		
		Eigen::MatrixXf C_t(10,3);
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
		//K=covarianza_ant*C'*inv(C*covarianza_ant*C'+Qt_laser);
		Eigen::MatrixXf K(10,3);
		Eigen::MatrixXf Inversa(3,3);

		Inversa=C*covarianza_ant*C_t + Qt_laser;
		K=covarianza_ant*C_t*(Inversa.inverse());

		//Cálculo del nuevo estado
		//media_act=media_ant+K*(z-C*media_ant);
		
		estado_correccion_laser_eigen=media_ant+K*(z_laser_eigen - C*media_ant);

		//Cálculo de la nueva covarianza
		//aux=K*C;
		//covarianza_act=(eye(size(aux))-aux)*covarianza_ant;
		varianza_correccion_laser_eigen=(I - K*C)*covarianza_ant;
		
		for(i=0;i<10;i++)
		{
			estado_correccion_laser[i]=estado_correccion_laser_eigen(i);
			for(j=0;j<10;j++)	varianza_correccion_laser[i][j]=varianza_correccion_laser_eigen(i,j);
		}
	}
};
