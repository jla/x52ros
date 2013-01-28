#include <unistd.h>
#include <math.h>
#include <vector>
#include <fcntl.h>
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>
#include <diagnostic_updater/diagnostic_updater.h>
//#include "X52/Calibrate.h"

extern "C" {
    #include "x52interface.h"
}

#include "hidapi.h"

class X52
{
private:
	ros::NodeHandle n_;
	ros::Publisher  pub_;
	ros::Subscriber sub_;
	ros::ServiceServer serv_;

	diagnostic_updater::Updater diagnostic_;

	hid_device *hid_;
	double deadzone_;
	bool   calibrated_;
	double rate_min_;
	double rate_max_;

	std::vector<float> axesOffset;
	std::vector<float> axesDeadzone;


/*
	bool calibrateCallback(X52::Calibrate::Request  &req, X52::Calibrate::Response &res )
	{

		return true;
	}
*/
	void feedbackCallback(const sensor_msgs::JoyFeedback::ConstPtr& msg)
	{
		char buf[512];
		sprintf(buf,"I heard: [%d]", msg->id);
		ROS_INFO(buf);
		switch(msg->id)
		{
			case 0:
				if (msg->intensity <= 0) x52i_clr_led(x52i_led_launch);
				else x52i_set_led(x52i_led_launch);
				break;

			case 1:
				if (msg->intensity <= 0) {
					x52i_clr_led(x52i_led_A_amber);
				} else if (msg->intensity <= 1.0) {
					x52i_clr_led(x52i_led_A_red);
					x52i_set_led(x52i_led_A_green);
				} else if  (msg->intensity <= 2.0) {
					x52i_set_led(x52i_led_A_amber);
				} else {
					x52i_clr_led(x52i_led_A_green);
					x52i_set_led(x52i_led_A_red);
				}
				break;

			case 2:
				if (msg->intensity <= 0) {
					x52i_clr_led(x52i_led_B_amber);
				} else if (msg->intensity <= 1.0) {
					x52i_clr_led(x52i_led_B_red);
					x52i_set_led(x52i_led_B_green);
				} else if  (msg->intensity <= 2.0) {
					x52i_set_led(x52i_led_B_amber);
				} else {
					x52i_clr_led(x52i_led_B_green);
					x52i_set_led(x52i_led_B_red);
				}
				break;

			case 4:
				if (msg->intensity <= 0) {
					x52i_clr_led(x52i_led_D_amber);
				} else if (msg->intensity <= 1.0) {
					x52i_clr_led(x52i_led_D_red);
					x52i_set_led(x52i_led_D_green);
				} else if  (msg->intensity <= 2.0) {
					x52i_set_led(x52i_led_D_amber);
				} else {
					x52i_clr_led(x52i_led_D_green);
					x52i_set_led(x52i_led_D_red);
				}
				break;
		}

		x52i_commit();
	}

public:
	X52(): n_(), diagnostic_()
	{
		calibrated_ = false;
	}

	void open(void)
	{
		int res = x52i_open_device();
		//x52i_set_led(x52i_led_A_red);
		x52i_clr_led(x52i_led_all);
		x52i_set_text(x52i_text_line1,"ROS Running");
		x52i_commit();

		hid_ = hid_open(0x06a3,0x0762,NULL);
		if (!hid_) {
		        printf("unable to open device\n");
		//        return 1;
		}

		n_.param<double>("deadzone", deadzone_, 0.05);

		pub_ = n_.advertise<sensor_msgs::Joy>("joy", 1);
		sub_ = n_.subscribe("joy/set_feedback",1024,&X52::feedbackCallback,this);

	//	serv_ = n_.advertiseService("calibrate",&X52::calibrateCallback,this);


	}

	void close(void)
	{
		x52i_close_device();
		hid_close(hid_);
		/* Free static HIDAPI objects. */
		hid_exit();
	}

	int read(sensor_msgs::Joy* const msg)
	{
		unsigned char buf[256];
		int res = hid_read_timeout(hid_,buf,sizeof(buf),100);
		if (res == 0) {
			return 0; // TimeOut
		}
		if (res < 0) {
			return -1; // Error;
		}

		// x,y,z 10bit (0,1024)  Scaled to (-1,1)
		double aux;

		aux = (double)(((buf[1]&0x3)<<8)+buf[0]);
		if (aux <= 512) msg->axes[0] = (aux - 512) / 512;
		else msg->axes[0] = (aux - 511) / 512;

		aux = (double)(((buf[2]&0xF)<<8) + buf[1]>>2);
		if (aux <= 512) msg->axes[1] = (aux - 512) / 512;
		else msg->axes[1] = (aux - 511) / 512;

		aux = (double)(((buf[3])<<8) + buf[2]>>6);
		if (aux <= 512) msg->axes[2] = (aux - 512) / 512;
		else msg->axes[2] = (aux - 511) / 512;

		// (0,255) Scaled to (0,1)
		msg->axes[3] = (255 - (double)buf[4]) / 255.0;
		msg->axes[4] = (255 - (double)buf[5]) / 255.0; // xRot
		msg->axes[5] = (255 - (double)buf[6]) / 255.0; // yRot
		msg->axes[6] = (255 - (double)buf[7]) / 255.0; // Slider


		// | 0=Trigger L1 | 1=Fire | 2=A | 3=B | 4=C | 5=Pinkie | D=6 | E=7
		// | 8=T1 | 9=T2 | 10=T3 | 11=T4 | 12=T5 | 13=T6 | 14=Trigger L2 | 15=LeftMouse
		int i,j;
		for (i=8;i<10;i++) {
			for (j=0;j<8;j++) {
				msg->buttons[(i-8)*8 + j] = (bool)(buf[i] & (1 << j));
			}
		}

		// 16:Scroll
		if (buf[10] & (1 << 0)) msg->buttons[16] = -1;
		else if (buf[10] & (1 << 1)) msg->buttons[16] = 1;
		else msg->buttons[16] = 0;

		// 17:Right-Click
		msg->buttons[17] = (bool)(buf[16] & (1 << 2));

		// 18:POV3

		if (!calibrated_) {
			for (i=0;i<3;i++) {
				//ROS_INFO("Calibration Axe: %i Offset: %f",i,msg->axes[i]);
				axesOffset[i] = msg->axes[i];
			}
			calibrated_ = true;
		}


		// DeadZone
		for (i=0;i<3;i++) {
			msg->axes[i] = msg->axes[i] - axesOffset[i];
			if (fabs(msg->axes[i]) < deadzone_) msg->axes[i] = 0;
			else if (msg->axes[i] > 0) msg->axes[i] = (msg->axes[i] - deadzone_) * (1.0f / (1.0f - deadzone_ - axesOffset[i]));
			else msg->axes[i] = (msg->axes[i] + deadzone_) * (1.0f / (1.0f - deadzone_ + axesOffset[i]));
		}


		return res;
	}

	int spin(void)
	{
		open();

		sensor_msgs::Joy msg;
		msg.axes.resize(7);
		msg.buttons.resize(43);

		axesOffset.resize(7);

		while(ros::ok())
		{
			ros::spinOnce();

			msg.header.stamp = ros::Time().now();
			read(&msg);
			pub_.publish(msg);
		}
		close();
		return 0;
	}
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "X52");
    X52 x52;
    return x52.spin();
}
