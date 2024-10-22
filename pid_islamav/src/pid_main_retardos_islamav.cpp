// PID controller main loop. Set the state machine in order to change the path followed by the MAV

#include "pid_include.h"

//using namespace std;

double pos_kalman[4]={0};
double consigna[4]={0};
double K[3]={0.05, 0, 0};
int m=0;

void CallbackLecturaKalman(const ekf::mensaje_kalman_array::ConstPtr& msg)
{
	pos_kalman[0]=msg->estado_vector[0].estado;
	pos_kalman[1]=msg->estado_vector[1].estado;
	pos_kalman[2]=msg->estado_vector[2].estado;
	pos_kalman[3]=msg->estado_vector[3].estado;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pid_main_retardos_islamav");
	ros::NodeHandle n;

	pid_objeto pid_obj;

	//Suscriptores
	ros::Subscriber sub_poseEKF = n.subscribe("kalman_topic_retardos", 1000, CallbackLecturaKalman); //El nodo al que se suscribe para leer la salida del f de kalman

	//Publicadores
	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1000);
	ros::Publisher pub_errores = n.advertise<pid_islamav::pid_errores>("pid_errores", 1000);

	ros::Rate loop_rate(25); //valor en Hz del rate
	while (ros::ok())
	{
		ros::spinOnce();

		geometry_msgs::Twist msg_pub;
		pid_islamav::pid_errores msg_pid;

		switch(m)
		{
		case 0:
			pid_obj.error_anterior[0]=0;
			pid_obj.error_anterior[1]=0;
			pid_obj.error_anterior[2]=0;
			consigna[0]=0;
			consigna[1]=0;
			consigna[2]=1.5;
			consigna[3]=0;
			pid_obj.Calcula_comando(pos_kalman, consigna, pid_obj.error_anterior, K);
			if(pid_obj.cerca) m=1;
			break;
		case 1:
			consigna[0]=0.3;
			consigna[1]=0;
			consigna[2]=1.5;
			consigna[3]=0;
			pid_obj.Calcula_comando(pos_kalman, consigna, pid_obj.error_anterior, K);
			if(pid_obj.cerca) m=2;
			break;
		case 2:
			consigna[0]=0.3;
			consigna[1]=0.3;
			consigna[2]=1.5;
			consigna[3]=0;
			pid_obj.Calcula_comando(pos_kalman, consigna, pid_obj.error_anterior, K);
			if(pid_obj.cerca) m=3;
			break;
		case 3:
			consigna[0]=0.3;
			consigna[1]=0.6;
			consigna[2]=1.5;
			consigna[3]=0;
			pid_obj.Calcula_comando(pos_kalman, consigna, pid_obj.error_anterior, K);
			if(pid_obj.cerca) m=4;
			break;
		case 4:
			consigna[0]=0.3;
			consigna[1]=0.9;
			consigna[2]=1.5;
			consigna[3]=0;
			pid_obj.Calcula_comando(pos_kalman, consigna, pid_obj.error_anterior, K);
			if(pid_obj.cerca) m=5;
			break;
		case 5:
			consigna[0]=0.3;
			consigna[1]=1.2;
			consigna[2]=1.5;
			consigna[3]=0;
			pid_obj.Calcula_comando(pos_kalman, consigna, pid_obj.error_anterior, K);
			if(pid_obj.cerca) m=6;
			break;
		case 6:
			consigna[0]=0;
			consigna[1]=1.2;
			consigna[2]=1.5;
			consigna[3]=0;
			pid_obj.Calcula_comando(pos_kalman, consigna, pid_obj.error_anterior, K);
			if(pid_obj.cerca) m=7;
			break;
		case 7:
			consigna[0]=-0.3;
			consigna[1]=1.2;
			consigna[2]=1.5;
			consigna[3]=0;
			pid_obj.Calcula_comando(pos_kalman, consigna, pid_obj.error_anterior, K);
			if(pid_obj.cerca) m=8;
			break;
		case 8:
			consigna[0]=-0.3;
			consigna[1]=0.9;
			consigna[2]=1.5;
			consigna[3]=0;
			pid_obj.Calcula_comando(pos_kalman, consigna, pid_obj.error_anterior, K);
			if(pid_obj.cerca) m=9;
			break;
		case 9:
			consigna[0]=-0.3;
			consigna[1]=0.6;
			consigna[2]=1.5;
			consigna[3]=0;
			pid_obj.Calcula_comando(pos_kalman, consigna, pid_obj.error_anterior, K);
			if(pid_obj.cerca) m=10;
			break;
		case 10:
			consigna[0]=-0.3;
			consigna[1]=0.3;
			consigna[2]=1.5;
			consigna[3]=0;
			pid_obj.Calcula_comando(pos_kalman, consigna, pid_obj.error_anterior, K);
			if(pid_obj.cerca) m=11;
			break;
		case 11:
			consigna[0]=-0.3;
			consigna[1]=0;
			consigna[2]=1.5;
			consigna[3]=0;
			pid_obj.Calcula_comando(pos_kalman, consigna, pid_obj.error_anterior, K);
			if(pid_obj.cerca) m=12;
			break;
		case 12:
			consigna[0]=0;
			consigna[1]=0;
			consigna[2]=1.5;
			consigna[3]=0;
			pid_obj.Calcula_comando(pos_kalman, consigna, pid_obj.error_anterior, K);
			if(pid_obj.cerca) m=13;
			break;
		case 13:
			consigna[0]=0;
			consigna[1]=0;
			consigna[2]=1;
			consigna[3]=0;
			pid_obj.Calcula_comando(pos_kalman, consigna, pid_obj.error_anterior, K);
			break;
		default:
			m=0;
		}
		cout<<"Estado: "<<m<<endl;

		if(pid_obj.distancia_euclidea > 0.5)
		{
			K[0]=0.15;
			K[1]=0;
			K[2]=0.2;
		}
		else
		{
			K[0]=0.1;
			K[1]=0;
			K[2]=0.15;
		}



		//Envío las señales de control al drone
		msg_pub.linear.x=pid_obj.comando[0];
		msg_pub.linear.y=pid_obj.comando[1];
		msg_pub.linear.z=pid_obj.comando[2];
		msg_pub.angular.z=-pid_obj.comando[3];
		//Señales que solo servirán para poder hacer hovering en el momento que se desee
		msg_pub.angular.x=0;
		msg_pub.angular.y=0;
		//Envío las señales de error del drone
		msg_pid.xe=pid_obj.xe;
		msg_pid.ye=pid_obj.ye;
		msg_pid.ze=pid_obj.ze;
		msg_pid.yawe=pid_obj.yawe;
		msg_pid.m=m;

		chatter_pub.publish(msg_pub);
		pub_errores.publish(msg_pid);

		loop_rate.sleep();
	}

	//ros::spin();
	return 0;
}

