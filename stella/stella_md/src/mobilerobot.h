#define wheel_to_wheel_d            0.337       	// 바퀴와 바퀴 간 거리  [m]
#define distance_per_rev            0.5652        	// 한바퀴 회전시 이동 거리 [m / rev]
#define pulse_per_rev               54000        	// 한바퀴 회전시 엔코더 펄스 카운트(이 값은 부착된 엔코더와 감속기를 고려해 정해진다.) [pulse / rev]
#define pulse_per_distance          95541.4012739	// 1m 이동시 엔코더 펄스 카운트 [pulse / m]
#define gear_ratio                  27           	// 감속비
#define motor_scale_const           106.1032954  	// m/s to rpm



void calculate_wheel_vel(float linear, float angular, float *left_rpm,  float *right_rpm)
{
    float left_speed = 0.0, right_speed = 0.0;

    left_speed =  (linear + wheel_to_wheel_d / 2.0 * angular) * motor_scale_const;
    right_speed = (linear - wheel_to_wheel_d / 2.0 * angular) * motor_scale_const;

    *left_rpm  = left_speed * -1;
    *right_rpm = right_speed * -1;
}
