const int odomRate = 10;

int odom_time = 0, _odom_time = 0;

double x_pos = 0, y_pos = 0, theta_pos = 0;
double _x_pos = 0, _y_pos = 0, _theta_pos = 0;
double x_vel = 0, y_vel = 0, theta_vel = 0;

extern float wheel_radius, wheel_separation_width, wheel_separation_length;

extern int LF_pos, RF_pos, LB_pos, RB_pos;

const double ticks_per_rotation = 40;
int LF_prev_pos, RF_prev_pos, LB_prev_pos, RB_prev_pos;

void loopOdom() {

    if (millis() - odom_time > 1000 / odomRate) {
        
        _odom_time = odom_time;
        odom_time = millis();
        double dt = (odom_time - _odom_time) / 1000.0; // Convert time difference to seconds
        
        // Calculate wheel displacements since the last update
        int LF_disp = LF_pos - LF_prev_pos;
        int RF_disp = RF_pos - RF_prev_pos;
        int LB_disp = LB_pos - LB_prev_pos;
        int RB_disp = RB_pos - RB_prev_pos;

        // Update previous positions
        LF_prev_pos = LF_pos;
        RF_prev_pos = RF_pos;
        LB_prev_pos = LB_pos;
        RB_prev_pos = RB_pos;

        // Calculate linear and angular velocities of the robot
        double LF_vel = (LF_disp / ticks_per_rotation) * 2 * PI * wheel_radius / dt;
        double RF_vel = (RF_disp / ticks_per_rotation) * 2 * PI * wheel_radius / dt;
        double LB_vel = (LB_disp / ticks_per_rotation) * 2 * PI * wheel_radius / dt;
        double RB_vel = (RB_disp / ticks_per_rotation) * 2 * PI * wheel_radius / dt;

        // Calculate resultant linear velocities
        x_vel = (LF_vel + RF_vel + LB_vel + RB_vel) / 4.0;
        y_vel = (-LF_vel + RF_vel + LB_vel - RB_vel) / 4.0;  // Adjusted sign
        theta_vel = (-LF_vel + RF_vel - LB_vel + RB_vel) * wheel_radius / (4.0 * (wheel_separation_width + wheel_separation_length));

        // Update positions using forward kinematics
        x_pos += cos(theta_pos) * x_vel * dt - sin(theta_pos) * y_vel * dt;
        y_pos += sin(theta_pos) * x_vel * dt + cos(theta_pos) * y_vel * dt;
        theta_pos += theta_vel * dt;

        // Normalize theta_pos to keep it within [0, 2 * PI)
        theta_pos = fmod(theta_pos, 2 * PI);
        if (theta_pos < 0) theta_pos += 2 * PI;

		
		#if DEBUG_ODOM
            Serial.printf("[DEBUG] (odom) x_pos: %f, y_pos: %f, theta_pos: %f\n", x_pos, y_pos, theta_pos);
        #endif
	}
	
}

// void loopOdom() {

//     if (millis() - odom_time > 1000 / odomRate) {

//         _odom_time = odom_time;
//         odom_time = millis();
//         double dt = (odom_time - _odom_time) / 1000.0; // Convert time difference to seconds

//         x_pos = (LF_pos + RF_pos + LB_pos + RB_pos) / 336.0;
//         y_pos = (-LF_pos + RF_pos + LB_pos - RB_pos) / 336.0;
//         theta_pos = (-LF_pos + RF_pos - LB_pos + RB_pos) * (wheel_radius / (4.0 * (wheel_separation_width + wheel_separation_length)));

//         x_vel = (x_pos - _x_pos) / dt;
//         y_vel = (y_pos - _y_pos) / dt;
//         theta_vel = (theta_pos - _theta_pos) / dt;

//         _x_pos = x_pos;
//         _y_pos = y_pos;
//         _theta_pos = theta_pos;

// 		#if DEBUG_ODOM
//             Serial.printf("[DEBUG] (odom) x_pos: %f, y_pos: %f, theta_pos: %f\n", x_pos, y_pos, theta_pos);
//         #endif
//     }

// // Calculate time elapsed since last update
// static unsigned long prevTime = 0;
// unsigned long currentTime = millis();
// double dt = (currentTime - prevTime) / 1000.0; // Convert milliseconds to seconds
// prevTime = currentTime;

// // Calculate velocities for each wheel
// double LF_delta_pos = LF_pos - LF_prev_pos;
// double RF_delta_pos = RF_pos - RF_prev_pos;
// double LB_delta_pos = LB_pos - LB_prev_pos;
// double RB_delta_pos = RB_pos - RB_prev_pos;

// double LF_vel = LF_delta_pos / (ticks_per_meter * dt);
// double RF_vel = RF_delta_pos / (ticks_per_meter * dt);
// double LB_vel = LB_delta_pos / (ticks_per_meter * dt);
// double RB_vel = RB_delta_pos / (ticks_per_meter * dt);

// // Update previous positions
// LF_prev_pos = LF_pos;
// RF_prev_pos = RF_pos;
// LB_prev_pos = LB_pos;
// RB_prev_pos = RB_pos;

// // Calculate distances traveled by each wheel
// double distance_LF = (2 * M_PI * wheel_radius * LF_delta_pos) / ticks_per_meter;
// double distance_RF = (2 * M_PI * wheel_radius * RF_delta_pos) / ticks_per_meter;
// double distance_LB = (2 * M_PI * wheel_radius * LB_delta_pos) / ticks_per_meter;
// double distance_RB = (2 * M_PI * wheel_radius * RB_delta_pos) / ticks_per_meter;

// // Calculate average distance and adjust signs
// double dx = (distance_LF + distance_RF + distance_LB + distance_RB) / 4.0;
// double dy = (-distance_LF + distance_RF + distance_LB - distance_RB) / 4.0;
// double dtheta = (-distance_LF + distance_RF - distance_LB + distance_RB) / (4.0 * (wheel_separation_width + wheel_separation_length));

// // Integrate to update positions and orientation
// x_pos += dx;
// y_pos += dy;
// theta_pos += dtheta;

// #if DEBUG_ODOM
//     Serial.printf("[DEBUG] (odom) x_pos: %f, y_pos: %f, theta_pos: %f\n", x_pos, y_pos, theta_pos);
// #endif
// }
