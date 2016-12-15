#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <vector>
#include <ostream>
#include <stdlib.h>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "tf2_msgs/TFMessage.h"
#include "tf/tfMessage.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Transform.h"
#include <iostream>
#include <string>
#include <math.h>
#include "tf/transform_datatypes.h"
#include "ekf/mensaje_kalman.h"
#include "ekf/mensaje_kalman_array.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "pid_islamav/pid_errores.h"
using namespace std;

class pid_objeto
{
public:

	//Variables por las que se define
	double comando[4];
	double error_anterior[3];
	double xe, ye, ze, yawe;
	double distancia_euclidea;
	int cerca;

	//Función de cálculo de escala
	void Calcula_comando(double posicion[4], double consigna[4], double e_anterior[3], double K[3])
	{

		//Constantes del control PID
		const double Kpyaw=0.1;
		const double Kpz=0.5;
		const double Kiz=0;
		const double Kdz=0.05;
        
		const double pi=M_PI;

		//Variables del control PID
		double Kp=0.5;
		double Ki=0.2;
		double Kd=0.2;

		//Constantes de velocidad
		const int MAX_LINEAR=1;
		const int MAX_ANGULAR=1;

		//Distancia aproximada
		const double dist=0.2;

		//Variables
		double zeacumulado=0;

		int i;

		cerca=0;

		//Cálculo de errores
		xe=consigna[0]-posicion[0];
		ye=consigna[1]-posicion[1];
		ze=consigna[2]-posicion[2];
		yawe=consigna[3]-posicion[3];

		if(abs(yawe)>pi)
		{
			yawe=(abs(yawe)-2*pi);
			if(consigna[3]<0) yawe=-yawe;
		}

		Kp=K[0];
		Ki=K[1];
		Kd=K[2];

		//Cálculo de las señales de control
		if(zeacumulado<1.5) zeacumulado=zeacumulado+ze;
		comando[0]=(Kp*xe+Kd*(xe-e_anterior[0]))*cos(posicion[3]) + (Kp*ye+Kd*(ye-e_anterior[1]))*sin(posicion[3]); //Vx
		comando[1]=(-(Kp*xe+Kd*(xe-e_anterior[0]))*sin(posicion[3])) + (Kp*ye+Kd*(ye-e_anterior[1]))*cos(posicion[3]); //Vy
		comando[2]=Kpz*ze + Kdz*(ze-e_anterior[2]) + Kiz*zeacumulado; //Vz
		comando[3]=Kpyaw*yawe; //Vyaw


		//Se evita que las señales de control excedan la velocidad normalizada
		for(i=0;i<4;i++)
		{
			if(abs(comando[i])>MAX_LINEAR)
			{
				if(comando[i]<0) comando[i]=-MAX_LINEAR;
				else comando[i]=MAX_LINEAR;
			}
		}

		//Se calcula distancia euclídea y se deja en hoovering si dicha distancia es menor a 20 cm
		distancia_euclidea=sqrt(pow(xe,2)+pow(ye,2)+pow(ze,2));
		cout<<"Distancia euclídea: "<<distancia_euclidea<<endl;
		if(distancia_euclidea<=dist)
		{
			//Se ponen todos los valores a 0 para poder hacer hovering
			comando[0]=0;
			comando[1]=0;
			comando[2]=0;
			comando[3]=0;
			cout<<"Distancia euclídea menor de "<<dist<<endl;
			cerca=1;
		}

		//Actualizo variables
		error_anterior[0]=xe;
		error_anterior[1]=ye;
		error_anterior[2]=ze;

		cout<<"Xe: "<<xe<<" Comando vx: "<<comando[0]<<endl;
		cout<<"Ye: "<<ye<<" Comando vy: "<<comando[1]<<endl;
		cout<<"Ze: "<<ze<<" Comando vz: "<<comando[3]<<endl;
		cout<<"Yawe: "<<yawe<<" Comando vyaw: "<<comando[2]<<endl;
		cout<<"Kp: "<<K[0]<<" Ki: "<<K[1]<<" Kd: "<<K[2]<<endl;


	}
};
