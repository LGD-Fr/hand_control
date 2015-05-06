#include <ros/ros.h>
#include <ros/time.h>
#include <locale.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <hand_control/Plan.h>

#include <geometry_msgs/Twist.h>

#include <math.h>

class Run
{
	private:
	  float z_previous;
	  ros::Time t_previous;
	  ros::Time t_current;	  
	  ros::Rate loop_rate;
	  float dz;
	  float xx;
	  float yy;
	  float zz;
	  float x_speed;
	  float y_speed;
	  float z_speed_max;
	  
	  float precision;
  	  
	public:
	  Run() :
	      x_speed(0.2),
	      y_speed(0.2),
	      z_speed_max(0.5),
	      loop_rate(30)
	      {
	      //Sensibilité du drône	      
	      precision = 0.1;   
	      }
		

	void operator()(const hand_control::Plan::ConstPtr& msg)
	{
		t_current = msg->header.stamp;
		
		dz = (msg->normal.z - z_previous)/(t_current.sec - t_previous.sec);
		
		xx = msg->normal.x ; 
		yy = msg->normal.y ; 		
		//zz = msg->normal.z ; 
		
		t_previous = msg->header.stamp;
		z_previous = msg->normal.z;      	
        };

	void run(){
	//pour initialiser mvt (twist)
	geometry_msgs::Twist::Ptr mvt_init(new geometry_msgs::Twist());
	mvt_init->linear.x = mvt_init->linear.y = mvt_init->linear.z = 
	mvt_init->angular.x = mvt_init->angular.y = mvt_init->angular.z = 0.;
	
	
	  while(ros::ok()){
  
		  geometry_msgs::Twist::Ptr mvt(new geometry_msgs::Twist());
		  mvt = mvt_init;
		  
		  //mouvement selon z
		  if (dz < z_speed_max){  //pour éviter un soulèvement trop prompt du drône
		  mvt->linear.z = dz *0.1 ;
		  }
	  
		  	//mouvement selon x ou y	
			if (fabs(xx) > precision && fabs(yy) > precision ) 
			  {
				mvt->linear.x = x_speed ;
				mvt->linear.y = y_speed ;			
			  }
	  
		ros::spinOnce();
		loop_rate.sleep();
	}//end while
	}//end run
	
}; //class Callback

int main(int argc, char** argv)
{

  
  
  ros::init(argc, argv, "commande");
  
  Run run;
  run.run();

  
  return 0;
}
