#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <exception>
#include <string>

using namespace std;


class CONTROLLER {
	public:
		CONTROLLER();
		void odom_cb( nav_msgs::Odometry );
		void control_loop ();
		void load_from_file(string, std::vector<double> &);
		void load_to_file(string, std::vector<double> &);
		void run();
		void vel_ctrl(double, double);
		double radians (double);

	private:
		ros::NodeHandle _nh;
		ros::Publisher  _vel_pub;
		ros::Subscriber _odom_sub;

		double x_real;
		double y_real;
		double theta_real;

		double x_g = 9.0;
		double y_g = 9.0;
		double theta_g = 0.0;

		string file_path;

		double v_max = 0.26; //0.26
		double w_max = 1.7; //1.82

		std::vector<double> x_path;
		std::vector<double> y_path;
        std::vector<double> theta_path;
		std::vector<double> lin_vel;
		std::vector<double> ang_vel;
		std::vector<double> pos_error;
		std::vector<double> ang_error;
		std::vector<double> lin_vel_real;
		std::vector<double> ang_vel_real;

};


CONTROLLER::CONTROLLER() {

	file_path = "/home/marco/Scrivania/ros_ws/src/fsr_exam/src/";

	_vel_pub = _nh.advertise< geometry_msgs::Twist >("/cmd_vel", 0);
	_odom_sub = _nh.subscribe("/odom",0,&CONTROLLER::odom_cb, this);

	load_from_file("x_path.txt", x_path);
	load_from_file("y_path.txt", y_path);
	load_from_file("theta_path.txt", theta_path);
	load_from_file("lin_vel.txt", lin_vel);
	load_from_file("ang_vel.txt", ang_vel);

}

void CONTROLLER::odom_cb( nav_msgs::Odometry msg){
	double roll, pitch;

	x_real = msg.pose.pose.position.x;
	y_real = msg.pose.pose.position.y;

	tf::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);

    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, theta_real);

	

}


void CONTROLLER::vel_ctrl(double v, double w) {
	
	geometry_msgs::Twist cmd_vel;

	if (v > v_max) 	{ v = v_max; }
	if (v < -v_max) { v = -v_max; }
	if (w > w_max) 	{ w = w_max; }
	if (w < -w_max) { w = -w_max; }

	lin_vel_real.push_back(v);
	ang_vel_real.push_back(w);

	cmd_vel.linear.x = v;
	cmd_vel.angular.z = w;
	_vel_pub.publish( cmd_vel );

}

double CONTROLLER::radians (double degree){
    return degree*M_PI/180;
}

void CONTROLLER::load_from_file(string name, std::vector<double> &vector){
	
	string line;
	fstream myfile (file_path+name);

	if (myfile.is_open())
	{
		while ( getline (myfile,line) )
		{
			vector.push_back(boost::lexical_cast<double>(line));
		}
		myfile.close();
	}
		else ROS_INFO_STREAM("FILE " << name <<" NOT OPENED");

}

void CONTROLLER::load_to_file(string name, std::vector<double> &vector){
	
	string line;
	ofstream myfile (file_path+name);

	if (myfile.is_open())
	{
		for (int i=0; i<vector.size(); i++){
			myfile<<vector.at(i)<<"\n";
		}
		myfile.close();
	}
		else ROS_INFO_STREAM("FILE " << name <<" NOT OPENED");

}

void CONTROLLER::control_loop(){
	ROS_INFO_STREAM("Start control loop");
	ros::Rate rate(80);

	int i = 0;
	double b = 0;
	double y1, y2;
	double y1d_dot, y2d_dot;
	double u1, u2;
	double v, w;
	double k1 = 2.5; 	//2.5
	double k2 = 1;		//1
	double k3 = 1;		//1	
	double pos_error_temp;
	double ang_error_temp;
	double rho, gamma, delta;
	double x_diff, y_diff;
	

	//INPUT/OUTPUT LINEARIZATION
	while (i<(x_path.size()-1)){
		x_diff = x_path.at(i+1) - x_path.at(i);
		y_diff = y_path.at(i+1) - y_path.at(i);

		b = sqrt(pow(x_diff,2) + pow(y_diff,2));
		
		y1 = x_real + b*cos(theta_real);
		y2 = y_real + b*sin(theta_real);

		y1d_dot = lin_vel.at(i)*cos(theta_path.at(i)) - ang_vel.at(i)*b*sin(theta_path.at(i));
		y2d_dot = lin_vel.at(i)*sin(theta_path.at(i)) + ang_vel.at(i)*b*cos(theta_path.at(i));

		u1 = y1d_dot + k1*(x_path.at(i+1) - y1);
		u2 = y2d_dot + k2*(y_path.at(i+1) - y2);

		//u1 = y1d_dot + k1*(x_path.at(i) - y1);
		//u2 = y2d_dot + k2*(y_path.at(i) - y2);

		v = u1*cos(theta_real) + u2*sin(theta_real);
		w = -u1*sin(theta_real)/b + u2*cos(theta_real)/b; 


		pos_error_temp = sqrt(pow(x_real - x_path.at(i),2) + pow(y_real - y_path.at(i),2));

		
		//ROS_INFO_STREAM("y2d_dot="<< y2d_dot);
		ROS_INFO_STREAM("error="<< pos_error_temp);
		
		pos_error.push_back(pos_error_temp);

		vel_ctrl(v, w);
		i++;
		rate.sleep();
	}

	vel_ctrl(0, 0);
	ang_error_temp = theta_real - theta_g;
	pos_error_temp = sqrt(pow(x_g - x_real,2) + pow(y_g - y_real,2));

	k1 = 2;
	k2 = 1;
	k3 = 3;

	//POSTURE REGULATION
	while( (abs(pos_error_temp)>0.05) || ((abs(ang_error_temp)>0.15))){
		x_diff = x_real - x_g;
		y_diff = y_real - y_g;

		rho = sqrt(pow(x_diff,2) + pow(y_diff,2));
		//gamma = atan2(y_diff, x_diff) - theta_g + M_PI;
		//delta = gamma + theta_g;

		gamma = atan2(y_diff, x_diff) - theta_real + M_PI;
		delta = gamma + theta_real;

		v = k1*rho*cos(gamma);
		w = (k2*gamma)+k1*((sin(gamma)*cos(gamma))/gamma)*(gamma + k3*delta);

		pos_error_temp = rho;
		ang_error_temp = theta_g - theta_real;
		pos_error.push_back(pos_error_temp);
		ang_error.push_back(ang_error_temp);
		//ROS_INFO_STREAM("pos_error="<< pos_error_temp);
		ROS_INFO_STREAM("ang_error="<< ang_error_temp);

		vel_ctrl(v, w);
		rate.sleep();
	}

	vel_ctrl(0, 0);
	load_to_file("pos_error.txt", pos_error);
	load_to_file("ang_error.txt", ang_error);
	load_to_file("lin_vel_real.txt", lin_vel_real);
	load_to_file("ang_vel_real.txt", ang_vel_real);

}


void CONTROLLER::run() {
	boost::thread control_loop_t( &CONTROLLER::control_loop, this );
	//boost::thread vel_ctrl_t( &CONTROLLER::vel_ctrl, this );
	ros::spin();
}


int main(int argc, char** argv ) {

	ros::init(argc, argv, "CONTROLLER");

	CONTROLLER io_ctr;
	io_ctr.run();

	return 0;
}
