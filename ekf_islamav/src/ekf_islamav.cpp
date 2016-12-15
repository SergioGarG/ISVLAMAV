// filtro de kalman sin tener en cuenta retardos con láser

#include "predictor.h"
#include "correccion_vslam.h"
#include "correccion_laser.h"
#include "correccion_imu_erle.h"
#include "correccion_imu_bebop.h"
#include "escala.h"
#include "variables.h"
using namespace std;

/* Se normalizan todos los ejes a:
   x : desplazamiento de profundidad (adelante y atrás)
   y : desplazamiento lateral (izquierda y derecha)
   z : desplazamiento vertical (arriba y abajo)

   Negativo |   | Positivo
   ---------|---|----------
    Atrás   | X | Adelante
    Derecha | Y | Izquierda
    Abajo   | Z | Arriba
 */

/////////////////////////////////////MAIN//////////////////////////////////////

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ekf");

	ros::NodeHandle n;

	//Suscriptores
	ros::Subscriber sub_poseLSD = n.subscribe("lsd_slam/pose", 1000, CallbackPoseLSD); //El nodo al que se subscribe en caso de ser LSD

	ros::Subscriber sub_altitude_bebop = n.subscribe("bebop/states/ARDrone3/PilotingState/AltitudeChanged", 1000, CallbackAltitudeBebop); //Nodo bebop altura
	ros::Subscriber sub_attitude_bebop = n.subscribe("bebop/states/ARDrone3/PilotingState/AttitudeChanged", 1000, CallbackAttitudeBebop); //Nodo bebop ángulos
	ros::Subscriber sub_speed_bebop = n.subscribe("bebop/states/ARDrone3/PilotingState/SpeedChanged", 1000, CallbackVelocidadBebop); //Nodo bebop velocidades

	ros::Subscriber sub_cmd_bebop = n.subscribe("bebop/cmd_vel", 1000, CallbackCMDBebop); //El nodo al que se subscribe para conseguir los comandos enviados al bebop

	ros::Subscriber sub_altitude_erle = n.subscribe("/ultrasonic_sensor_1", 1000, CallbackAltitudeErle); //Nodo erle altura
	ros::Subscriber sub_attitude_erle = n.subscribe("/mavros/imu/data", 1000, CallbackAttitudeErle); //Nodo erle altura
	ros::Subscriber sub_u_falsa = n.subscribe("/cmd_vel_stamped", 1000, CallbackUFalsa); //Nodo u falsa
	ros::Subscriber sub_laser = n.subscribe("/poseupdate", 1000, CallbackLaser); //Nodo láser

	tf::TransformListener listener;
	//tf::TransformListener listener_tf(ros::Duration(10)); //Descomentar para usar el cambio de frame

	//Objetos de las clases
	predictor prediccion;
	corrector_vslam correctorvslam;
	corrector_laser correctorlaser;
	corrector_imu_bebop correctorimu_bebop;
	corrector_imu_erle correctorimu_erle;
	escala_obj escala_cal;

	//Publicadores
	ros::Publisher chatter_pub = n.advertise<ekf_islamav::mensaje_kalman_array>("kalman_topic", 1000);
	ros::Publisher escala_pub = n.advertise<geometry_msgs::PoseStamped>("escala_topic", 1000);

	//Defino variables de tiempo
	ros::Time current_time;
	ros::Time last_time;


	ros::Rate loop_rate(100); //valor en Hz del rate

	while (ros::ok())
	{
		ros::spinOnce();
		current_time = ros::Time::now();

		ekf_islamav::mensaje_kalman data;
		ekf_islamav::mensaje_kalman_array msg_pub;
		geometry_msgs::PoseStamped msg;
		tf::StampedTransform transform;

		are_equal=false;
		are_equal_0=false;

		//Se lee la posición con 6DoF según ORB
		if(!tecnica_vslam)
		{
			try
			{
				listener.lookupTransform("/ORB_SLAM/World", "/ORB_SLAM/Camera",
						ros::Time(0), transform);
			}

			catch (tf::TransformException &ex)
			{
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
				continue;
			}

			pose_vslam[1]=-transform.getOrigin().x();
			pose_vslam[2]=-transform.getOrigin().y();
			pose_vslam[0]=transform.getOrigin().z();
			orientacion_vslam[0]=transform.getRotation().x();
			orientacion_vslam[1]=transform.getRotation().y();
			orientacion_vslam[2]=transform.getRotation().z();
			orientacion_vslam[3]=transform.getRotation().w();
		}

		//if(z_imu[2]<0) z_imu[2]=-z_imu[2];
		if(pose_vslam[2]<0) pose_vslam[2]=-pose_vslam[2];

		if(!drone_utilizado) escala_cal.Calcula_escala(pose_vslam, orientacion_vslam, z_imu_bebop[2], posicion_anterior);
		else escala_cal.Calcula_escala(pose_vslam, orientacion_vslam, z_imu_erle[0], posicion_anterior);

		for(i=0;i<3;i++)
		{
			z_vslam[i]=escala_cal.posicion_real[i];
			z_vslam[i+3]=escala_cal.orientacion_real[i];
		}
		
		cout<<"------------------------"<<endl;
		cout<<"Tf x: "<<pose_vslam[0]<<endl;
		cout<<"Tf y: "<<pose_vslam[1]<<endl;
		cout<<"Tf z: "<<pose_vslam[2]<<endl;
		cout<<"Tf x: "<<pose_vslam[0]<<endl;
		cout<<"------------------------"<<endl;
		cout<<"Escala x "<<z_vslam[0]<<endl;
		cout<<"Escala y "<<z_vslam[1]<<endl;
		cout<<"Escala z "<<z_vslam[2]<<endl;
		cout<<"------------------------"<<endl;
		cout<<"Láser x "<<z_laser[0]<<endl;
		cout<<"Láser y "<<z_laser[1]<<endl;


		//Aquí se comprueba si el feed de vídeo del drone se ha congelado viendo si las últimas 6 estimaciones de posición son iguales

		for(i=0;i<6;i++)
		{
			for(j=0;j<3;j++)
			{
				if(i==0)
				{
					if (pose_vslam[j] == posicion_anterior_vslam[i][j]) are_equal_0 = true;
				}
				else if(i!=0 && are_equal_0==true)
				{
					if(posicion_anterior_vslam[i][j]==posicion_anterior_vslam[i-1][j])
					{
						are_equal_0=true;
						if(i==5) are_equal=true;
					}
					else
					{
						are_equal_0=false;
						break;
					}
				}
			}
		}

		//cout<<"tiempo transcurrido"<<tiempo_transcurrido<<endl;

		//Aquí se aplican los 3 modelos del EKF. Si se ha detectado que el feed de vídeo se ha congelado, se deja de utilizar el modelo
		//de corrección de VSLAM.

		if(are_equal==true)
		{
			prediccion.predecir(media, covarianza, u);
			if(!drone_utilizado) 
			{
				correctorimu_bebop.corregir_imu_bebop(prediccion.estado_predictor,prediccion.varianza_predictor, z_imu_bebop);
				//correctorlaser.corregir_laser(correctorimu_bebop.estado_correccion_imu, correctorimu_bebop.varianza_correccion_imu, z_laser);
			}
			else 
			{
				correctorimu_erle.corregir_imu_erle(prediccion.estado_predictor,prediccion.varianza_predictor, z_imu_erle);
				//correctorlaser.corregir_laser(correctorimu_erle.estado_correccion_imu, correctorimu_erle.varianza_correccion_imu, z_laser);
			}
			cout<<"["<<current_time<<"]"<<" Video stream is frozen, estimation discarding VSLAM correction"<<endl;
		}
		else
		{
			prediccion.predecir(media, covarianza, u);
			correctorvslam.corregir_vslam(prediccion.estado_predictor, prediccion.varianza_predictor, z_vslam);
			if(!drone_utilizado)
			{
				correctorimu_bebop.corregir_imu_bebop(correctorvslam.estado_correccion_vslam, correctorvslam.varianza_correccion_vslam, z_imu_bebop);
				//correctorimu_bebop.corregir_imu_bebop(prediccion.estado_predictor, prediccion.varianza_predictor, z_imu_bebop);
				//correctorlaser.corregir_laser(correctorimu_bebop.estado_correccion_imu, correctorimu_bebop.varianza_correccion_imu, z_laser);
			}
			else
			{
				correctorimu_erle.corregir_imu_erle(correctorvslam.estado_correccion_vslam, correctorvslam.varianza_correccion_vslam, z_imu_erle);
				//correctorimu_erle.corregir_imu_erle(prediccion.estado_predictor, prediccion.varianza_predictor, z_imu_erle);
				//correctorlaser.corregir_laser(correctorimu_erle.estado_correccion_imu, correctorimu_erle.varianza_correccion_imu, z_laser);
			}
		}

		//Se recarga la posición anterior de vslam para la comparación en la siguiente iteración
		for(i=5;i>(-1);i--)
		{
			for(j=0;j<3;j++)
			{
				if(i==0) posicion_anterior_vslam[i][j]=pose_vslam[j];
				else posicion_anterior_vslam[i][j]=posicion_anterior_vslam[i-1][j];
			}
		}

		//Se crea este bucle para poner la salida deseada y poder hacer pruebas con todos los modelos por separado
		for(i=0;i<10;i++)
		{
			//modelo[i]=prediccion.estado_predictor[i];
			//modelo[i]=correctorvslam.estado_correccion_vslam[i];
			if(!drone_utilizado) modelo[i]=correctorimu_bebop.estado_correccion_imu[i];
			else modelo[i]=correctorimu_erle.estado_correccion_imu[i];
			//modelo[i]=correctorlaser.estado_correccion_laser[i];
			media[i]=modelo[i];
			for(j=0;j<10;j++)
			{
				//covarianza_modelo[i][j]=prediccion.varianza_predictor[i][j];
				//covarianza_modelo[i][j]=correctorvslam.varianza_correccion_vslam[i][j];
				if(!drone_utilizado) covarianza_modelo[i][j]=correctorimu_bebop.varianza_correccion_imu[i][j];
				else covarianza_modelo[i][j]=correctorimu_erle.varianza_correccion_imu[i][j];
				//covarianza_modelo[i][j]=correctorlaser.varianza_correccion_laser[i][j];
				covarianza[i][j]=covarianza_modelo[i][j];
			}
		}

		for(i=0;i<3;i++)
		{
			//Se evitan saltos o errores de cálculo frente a irregularidades del terreno mediante este ajuste
			//if(abs(abs(media[i])-abs(posicion_anterior[i]))>0.1) media[i]=posicion_anterior[i];
			//Se evitan problemas en el algoritmo eliminando medidas no deseadas
			if(isnan(media[i])) media[i]=0;
			if(isnan(media[i+3])) media[i+3]=0;
			posicion_anterior[i]=media[i];
			orientacion_anterior[i]=media[6+i];
		}


		//Prueba, se toman como correctas las medidas de la IMU para rotaciones y velocidades

		if(!drone_utilizado)
		{
			media[2]=z_imu_bebop[2];
			media[3]=z_imu_bebop[0];
			media[4]=z_imu_bebop[1];
			media[6]=z_imu_bebop[3];
			media[7]=z_imu_bebop[4];
			media[8]=z_imu_bebop[5];
		}
		else
		{
			media[2]=z_imu_erle[0];
			media[6]=z_imu_erle[1];
			media[7]=z_imu_erle[2];
			media[8]=z_imu_erle[3];
		}

		//Se elimina el offset inicial de yaw para que inicie siempre en 0
		//Calculo_orientacion(z_imu[5]);
		if((z_imu_bebop[2]>=1) || (z_imu_erle[0]>=1)) //&& (primera_vez==1))
		{
			if(!drone_utilizado) Calculo_orientacion(z_imu_bebop[5]);
			else Calculo_orientacion(z_imu_erle[3]);
			primera_vez2=1;
			//cout<<"Yaw ajustada a: "<<media[8]<<endl;
		}

		//Se realiza la conversión entre la posición de la cámara y el centro del drone, descomentar para usar la función
		//ConversorCamara_listener(listener_tf);

		/////////////Se envían los datos
		
		msg.pose.position.x=escala_cal.posicion_real[0];
		msg.pose.position.y=escala_cal.posicion_real[1];
		msg.pose.position.z=escala_cal.posicion_real[2];

		for(i=0;i<10;i++)
		{
			data.estado=media[i];
			msg_pub.estado_vector.push_back(data);
		}

		escala_pub.publish(msg);
		chatter_pub.publish(msg_pub);

		loop_rate.sleep();

	}

	//ros::spin();

	return 0;
}



