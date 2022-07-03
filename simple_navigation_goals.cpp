#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
using namespace std;

//Atributos globales
ros::Publisher pub;
std_msgs::UInt8 goalID;
tf2::Quaternion orientation;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


void moveBaseCallback(const geometry_msgs::Pose2D &msg) {
    
    //Creo al cliente mediante la librería actionlib
	MoveBaseClient action("move_base", true);

	//Creo al servidor y espero a que empiece a recibir mensajes
	while (!action.waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Waiting for the move_base action server to come up...");
	}
	
	//Asigno las coordenadas y orientación recibidas del punto
	float m_xPos = msg.x;
	float m_yPos = msg.y;
	float m_aPos = msg.theta;
	
	//Creo un goal y le asigno sus parámetros iniciales
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	
	//Asigno al goal las coordenadas X e Y del punto
	goal.target_pose.pose.position.x = m_xPos;
	goal.target_pose.pose.position.y = m_yPos;

	//Asigno al goal la orientación del punto
	orientation.setRPY(0,0,m_aPos);
	goal.target_pose.pose.orientation.x = ((double) orientation.x());
	goal.target_pose.pose.orientation.y = ((double) orientation.y());
	goal.target_pose.pose.orientation.z = ((double) orientation.z());
	goal.target_pose.pose.orientation.w = ((double) orientation.w());
    
	//Envío la información al servidor y espero la respuesta
	ROS_INFO("Sending goal...");
	action.sendGoal(goal);
	action.waitForResult();

	//Si se ha alcanzado el punto, publico su ID
	if (action.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("The robot has arrived at the goal location");
		pub.publish(goalID);
	}
	else {
		ROS_INFO("The robot failed to reach the goal location for some reason");
	}
	
	//Preparo la ID para el siguiente punto
	goalID.data += 1;
	
}


int main(int argc, char** argv) {

	//Inicio del main
	ros::init(argc, argv, "simple_navigation_goals");
  	cout << "\nWaiting for a message from the topic..." << endl;

	//Creo el nodo
	ros::NodeHandle node;
	
	//Creo un subscriber al topic de entrada de datos para recibir las coordenadas del punto en la función moveBaseCallback()
	ros::Subscriber sub = node.subscribe("/waypoint", 1000, moveBaseCallback);
	
	//Inicializo el publisher al segundo topic
	pub = node.advertise<std_msgs::UInt8>("/reached_waypoint", 100);
	
	//Se detiene la ejecución del main mientras se ejecuta la llamada a la función
	ros::spin();

	return 0;
	
}
