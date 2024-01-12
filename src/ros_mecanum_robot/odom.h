const int odomRate = 10;

int odom_time = 0, _odom_time = 0;

double x_pos = 0, y_pos = 0, theta_pos = 0;
double _x_pos = 0, _y_pos = 0, _theta_pos = 0;
double x_vel = 0, y_vel = 0, theta_vel = 0;

extern float wheel_radius, wheel_separation_width, wheel_separation_length;

extern int LF_pos, RF_pos, LB_pos, RB_pos;

void loopOdom() {
	if (millis() - odom_time > 1000 / odomRate) {
		odom_time = millis();
		
		x_pos = (LF_pos + RF_pos + LB_pos + RB_pos) / 4 * wheel_radius;
		y_pos = (- LF_pos + RF_pos + LB_pos - RB_pos) / 4 * wheel_radius;
		theta_pos = (- LF_pos + RF_pos - LB_pos + RB_pos) / 4 * wheel_radius / (wheel_separation_width + wheel_separation_length);

		x_vel = (x_pos - _x_pos) / (odom_time - _odom_time);
		y_vel = (y_pos - _y_pos) / (odom_time - _odom_time);
		theta_vel = (theta_pos - _theta_pos) / (odom_time - _odom_time);

		_x_pos = x_pos;
		_y_pos = y_pos;
		_theta_pos = theta_pos;
		_odom_time = odom_time;
	}
}