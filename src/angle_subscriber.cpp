#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include"sensor_msgs/LaserScan.h"
#include<iostream>
#include<limits>

using namespace std;

double lower_angle=0;
double upper_angle=0;
ros::Publisher pub; 


void anglecallback(const std_msgs::Float64MultiArray::ConstPtr & msg)
{
    //cout<<"into anglecallback"<<endl;
  lower_angle=msg->data[1];
  upper_angle=msg->data[0];
  //cout<<"done"<<endl;
}

void lasercallback(const sensor_msgs::LaserScan::ConstPtr &input_scan)
{
        //cout<<"into lasercallback"<<endl;
        sensor_msgs::LaserScan filtered_scan;
        filtered_scan.ranges.resize(input_scan->ranges.size());
        filtered_scan.intensities.resize(input_scan->intensities.size());

        double start_angle = input_scan->angle_min;
        double current_angle = input_scan->angle_min;
        ros::Time start_time = input_scan->header.stamp;
        unsigned int count = 0;
        double temp_replacement_value = std::numeric_limits<double>::quiet_NaN();
        //loop through the scan and truncate the beginning and the end of the scan as necessary
        //cout << "before for loop"<<endl;
        for(unsigned int i = 0; i < input_scan->ranges.size(); ++i){
          //wait until we get to our desired starting angle
          if(start_angle < lower_angle){
            start_angle += input_scan->angle_increment;
            current_angle += input_scan->angle_increment;
            start_time += ros::Duration(input_scan->time_increment);
          }
          else{

            if (input_scan->ranges[i] <= 0.2)
            {
              filtered_scan.ranges[count] = temp_replacement_value;
            }
            else
            {
              filtered_scan.ranges[count] = input_scan->ranges[i];
            }
            //cout << "filtered_scan.ranges[count]" <<endl;
            //make sure  that we don't update intensity data if its not available
            if(input_scan->intensities.size() > i)
              filtered_scan.intensities[count] = input_scan->intensities[i];

            count++;

            //check if we need to break out of the loop, basically if the next increment will put us over the threshold
            if(current_angle + input_scan->angle_increment > upper_angle){
              break;

            }

            current_angle += input_scan->angle_increment;

          }
        }

        //make sure to set all the needed fields on the filtered scan
        //cout<< "out for loop" <<endl;
        filtered_scan.header.frame_id = input_scan->header.frame_id;
        filtered_scan.header.stamp = start_time;
        filtered_scan.angle_min = start_angle;
        filtered_scan.angle_max = current_angle;
        filtered_scan.angle_increment = input_scan->angle_increment;
        filtered_scan.time_increment = input_scan->time_increment;
        filtered_scan.scan_time = input_scan->scan_time;
        filtered_scan.range_min = input_scan->range_min;
        filtered_scan.range_max = input_scan->range_max;

        filtered_scan.ranges.resize(count);
        //cout << "almost done"<<endl;
        if(input_scan->intensities.size() >= count)
          filtered_scan.intensities.resize(count);
        //cout << "done"<<endl;

        
        pub.publish(filtered_scan);
        //cout<<"published"<<endl;
}

int main(int argc,char**argv)
{
ros::init(argc,argv,"laser_modifier");
ros::NodeHandle n;
pub=n.advertise<sensor_msgs::LaserScan>("filtered_scan",1);
ros::Subscriber sub_1=n.subscribe("tracker_angles",1,anglecallback);
ros::Subscriber sub_2=n.subscribe("scan",1,lasercallback);

ros::spin();

}

