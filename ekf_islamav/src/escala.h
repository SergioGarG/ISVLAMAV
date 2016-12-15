#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include "tf2_msgs/TFMessage.h"
#include "tf/tfMessage.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Transform.h"
#include <iostream>
#include <math.h>
#include "tf/LinearMath/Transform.h"

class escala_obj
{
public:

  //Variables por las que se define
  double posicion_real[3];
  double orientacion_real[3];


  //Función de cálculo de escala
  void Calcula_escala(double posicion[3], double orientacion[4], double altitud, double posicion_anterior[3])
  {
	int i;
	double escala;

	escala=altitud/posicion[2];
	if(escala<0) escala=-escala;
	else if(escala==0) escala=1; //En posición de inicio (como altd=0->escala=0) suponer que la escala es unidad
	else if (escala > 20) escala=1;

	//Se hace la conversión quaternion-RPY
	tf::Quaternion q(orientacion[0], orientacion[1], orientacion[2], orientacion[3]);
	tf::Matrix3x3(q).getRPY(orientacion_real[0], orientacion_real[1], orientacion_real[2]);

	for(i=0;i<3;i++)
	{
		//Se calcula la posición real
		posicion_real[i]=posicion[i]*escala;
		//Se evitan saltos o errores de cálculo frente a irregularidades del terreno mediante este ajuste
		//if(abs(abs(posicion_real[i])-abs(posicion_anterior[i]))>1) posicion_real[i]=posicion_anterior[i];
		//Se evitan problemas en el algoritmo eliminando medidas no deseadas
		if(isnan(posicion_real[i])) posicion_real[i]=0;
		if(isnan(orientacion_real[i])) orientacion_real[i]=0;
		//posicion_anterior[i]=posicion_real[i];
	}

//		cout<<"-------------------------------"<<endl;
//		cout<<"Altura ultrasonidos: "<<altitud<<endl;
//		cout<<"Altura VSLAM: "<<posicion[2]<<endl;
//		cout<<"Escala=hultras/hvslam= "<<escala<<endl;
//		cout<<"Xreal=Xvslam*escala= "<<posicion[0]*escala<<endl;
//		cout<<"Yreal=Yvslam*escala= "<<posicion[1]*escala<<endl;
//		cout<<"Zreal=Zvslam*escala= "<<posicion[2]*escala<<endl;
//		cout<<"-------------------------------"<<endl;

	orientacion_real[1]=-orientacion_real[1];
	orientacion_real[2]=-orientacion_real[2];


  }

};
