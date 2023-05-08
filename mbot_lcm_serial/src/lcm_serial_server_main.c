#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <lcm/lcm.h>

#include <mbot_lcm_serial/lcm_config.h>

#include <mbot_lcm_msgs_pose2D_t.h>
#include <mbot_lcm_msgs_mbot_imu_t.h>
#include <mbot_lcm_msgs_mbot_encoders_t.h>
#include <mbot_lcm_msgs_mbot_motor_vel_t.h>
#include <mbot_lcm_msgs_mbot_motor_pwm_t.h>
#include <mbot_lcm_msgs_twist2D_t.h>
#include <mbot_lcm_msgs_timestamp_t.h>

#include <mbot_lcm_serial/protocol.h>
#include <mbot_lcm_serial/topic_data.h>
#include <mbot_lcm_serial/mbot_lcm_msgs_serial.h>
#include <mbot_lcm_serial/comms_common.h>
#include <mbot_lcm_serial/listener.h>

struct termios options;

bool running = true;

lcm_t* lcmInstance;

static void mbot_vel_cmd_lcm_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                                   const mbot_lcm_msgs_twist2D_t* msg, void* _user)
{
    mbot_lcm_msgs_twist2D_t to_send;
    to_send.utime = msg->utime;
    to_send.vx = msg->vx;
    to_send.vy = msg->vy;
    to_send.wz = msg->wz;
    comms_set_topic_data(MBOT_VEL_CMD, &to_send, sizeof(serial_twist2D_t));
    comms_write_topic(MBOT_VEL_CMD, &to_send);
}

static void reset_encoders_lcm_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                                       const mbot_lcm_msgs_mbot_encoders_t* msg, void* _user)
{
    serial_mbot_encoders_t to_send = {0};
    to_send.utime = msg->utime;
    to_send.ticks[0] = msg->ticks[0];
    to_send.ticks[1] = msg->ticks[1];
    to_send.ticks[2] = msg->ticks[2];
    to_send.delta_ticks[0] = msg->delta_ticks[0];
    to_send.delta_ticks[1] = msg->delta_ticks[1];
    to_send.delta_ticks[2] = msg->delta_ticks[2];
    to_send.delta_time = msg->delta_time;
    comms_set_topic_data(MBOT_ENCODERS_RESET, &to_send, sizeof(serial_mbot_encoders_t));
    comms_write_topic(MBOT_ENCODERS_RESET, &to_send);
}

static void timestamp_lcm_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                                  const mbot_lcm_msgs_timestamp_t* msg, void* _user)
{
    //printf("got timestamp!\r\n");
    serial_timestamp_t to_send = {0};
    to_send.utime = msg->utime;
    comms_set_topic_data(MBOT_TIMESYNC, &to_send, sizeof(serial_timestamp_t));
    comms_write_topic(MBOT_TIMESYNC, &to_send);
    //printf("sent timestamp!\r\n");
}

static void reset_odom_lcm_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                                   const mbot_lcm_msgs_pose2D_t* msg, void* _user)
{
    serial_pose2D_t to_send = {0};
    to_send.theta = msg->theta;
    to_send.x = msg->x;
    to_send.y = msg->y;
    comms_set_topic_data(MBOT_ODOMETRY_RESET, &to_send, sizeof(serial_pose2D_t));
    comms_write_topic(MBOT_ENCODERS_RESET, &to_send);
}

static void mbot_motor_pwm_cmd_lcm_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                                   const mbot_lcm_msgs_mbot_motor_pwm_t* msg, void* _user)
{
    serial_mbot_motor_pwm_t to_send = {0};
    to_send.utime = msg->utime;
    to_send.pwm[0] = msg->pwm[0];
    to_send.pwm[1] = msg->pwm[1];
    to_send.pwm[2] = msg->pwm[2];
    comms_set_topic_data(MBOT_MOTOR_PWM_CMD, &to_send, sizeof(serial_mbot_motor_pwm_t));
    comms_write_topic(MBOT_MOTOR_PWM_CMD, &to_send);
}

static void mbot_motor_vel_cmd_lcm_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                                   const mbot_lcm_msgs_mbot_motor_vel_t* msg, void* _user)
{
    serial_mbot_motor_vel_t to_send = {0};
    to_send.utime = msg->utime;
    to_send.velocity[0] = msg->velocity[0];
    to_send.velocity[1] = msg->velocity[1];
    to_send.velocity[2] = msg->velocity[2];
    comms_set_topic_data(MBOT_MOTOR_PWM_CMD, &to_send, sizeof(serial_mbot_motor_vel_t));
    comms_write_topic(MBOT_MOTOR_PWM_CMD, &to_send);
}

void signal_callback_handler(int signum)
{
    printf("Caught exit signal - exiting!\r\n");
    running = false;
    listener_running = false;
}

void serial_encoders_cb(serial_mbot_encoders_t* msg)
{
    mbot_lcm_msgs_mbot_encoders_t to_send = {0};
    to_send.utime = msg->utime;
    to_send.ticks[0] = msg->ticks[0];
    to_send.ticks[1] = msg->ticks[1];
    to_send.ticks[2] = msg->ticks[2];
    to_send.delta_ticks[0] = msg->delta_ticks[0];
    to_send.delta_ticks[1] = msg->delta_ticks[1];
    to_send.delta_ticks[2] = msg->delta_ticks[2];
    to_send.delta_time = msg->delta_time;
    mbot_lcm_msgs_mbot_encoders_t_publish(lcmInstance, MBOT_ENCODERS_CHANNEL, &to_send);
}

void serial_motor_vel_cb(serial_mbot_motor_vel_t* data)
{
    mbot_lcm_msgs_mbot_motor_vel_t to_send = {0};
    to_send.utime = data->utime;
    to_send.velocity[0] = data->velocity[0];
    to_send.velocity[1] = data->velocity[1];
    to_send.velocity[2] = data->velocity[2];
    mbot_lcm_msgs_mbot_motor_vel_t_publish(lcmInstance, MBOT_MOTOR_VEL_CHANNEL, &to_send);
}

void serial_odometry_cb(serial_pose2D_t* data)
{
    mbot_lcm_msgs_pose2D_t to_send = {0};
    to_send.utime = data->utime;
    to_send.theta = data->theta;
    to_send.x = data->x;
    to_send.y = data->y;
    mbot_lcm_msgs_pose2D_t_publish(lcmInstance, MBOT_ODOMETRY_CHANNEL, &to_send);
}

void serial_imu_cb(serial_mbot_imu_t* data)
{
    mbot_lcm_msgs_mbot_imu_t to_send = {0};
    to_send.utime = data->utime;
    to_send.accel[0] = data->accel[0];
    to_send.accel[1] = data->accel[1];
    to_send.accel[2] = data->accel[2];
    to_send.gyro[0] = data->gyro[0];
    to_send.gyro[1] = data->gyro[1];
    to_send.gyro[2] = data->gyro[2];
    to_send.mag[0] = data->mag[0];
    to_send.mag[1] = data->mag[1];
    to_send.mag[2] = data->mag[2];
    to_send.angles_rpy[0] = data->angles_rpy[0];
    to_send.angles_rpy[1] = data->angles_rpy[1];
    to_send.angles_rpy[2] = data->angles_rpy[2];
    to_send.angles_quat[0] = data->angles_quat[0];
    to_send.angles_quat[1] = data->angles_quat[1];
    to_send.angles_quat[2] = data->angles_quat[2];
    to_send.angles_quat[3] = data->angles_quat[3];
    to_send.temp = data->temp;
    mbot_lcm_msgs_mbot_imu_t_publish(lcmInstance, MBOT_IMU_CHANNEL, &to_send);
}

/*
* Each topic that gets sent to the pico needs to be registered here
* It must know the LCM channel, size of the serial message, serialize & deserialize functions
* and if it originates on the pico, there must be a callback function to copy and publish the LCM message
*/
void register_topics()
{
    // timesync topic
    comms_register_topic(MBOT_TIMESYNC, sizeof(serial_timestamp_t), (Deserialize)&timestamp_t_deserialize, (Serialize)&timestamp_t_serialize, NULL);
    // odometry topic
    comms_register_topic(MBOT_ODOMETRY, sizeof(serial_pose2D_t), (Deserialize)&pose2D_t_deserialize, (Serialize)&pose2D_t_serialize, (MsgCb)serial_odometry_cb);
    // odometry topic
    comms_register_topic(MBOT_ODOMETRY_RESET,  sizeof(serial_pose2D_t), (Deserialize)&pose2D_t_deserialize, (Serialize)&pose2D_t_serialize, NULL);
    // IMU topic
    comms_register_topic(MBOT_IMU, sizeof(serial_mbot_imu_t), (Deserialize)&mbot_imu_t_deserialize, (Serialize)&mbot_imu_t_serialize, (MsgCb)serial_imu_cb);
    // encoders topic
    comms_register_topic(MBOT_ENCODERS, sizeof(serial_mbot_encoders_t), (Deserialize)&mbot_encoders_t_deserialize, (Serialize)&mbot_encoders_t_serialize, (MsgCb)serial_encoders_cb);
    // reset encoders topic
    comms_register_topic(MBOT_ENCODERS_RESET, sizeof(serial_mbot_encoders_t), (Deserialize)&mbot_encoders_t_deserialize, (Serialize)&mbot_encoders_t_serialize, NULL);
    // motor commands topic
    comms_register_topic(MBOT_MOTOR_PWM_CMD, sizeof(serial_mbot_motor_pwm_t), (Deserialize)&mbot_motor_pwm_t_deserialize, (Serialize)&mbot_motor_pwm_t_serialize, NULL);
    comms_register_topic(MBOT_MOTOR_VEL_CMD, sizeof(serial_mbot_motor_vel_t), (Deserialize)&mbot_motor_vel_t_deserialize, (Serialize)&mbot_motor_vel_t_serialize, NULL);
    comms_register_topic(MBOT_MOTOR_VEL, sizeof(serial_mbot_motor_vel_t), (Deserialize)&mbot_motor_vel_t_deserialize, (Serialize)&mbot_motor_vel_t_serialize, (MsgCb)serial_motor_vel_cb);
    comms_register_topic(MBOT_VEL_CMD, sizeof(serial_twist2D_t), (Deserialize)&twist2D_t_deserialize, (Serialize)&twist2D_t_serialize, NULL);
}

void* handle_lcm(void* data)
{
    lcm_t* lcmInstance = data;
    while(running)
    {
        lcm_handle_timeout(lcmInstance, 100);
    }
    return NULL;
}

void subscribe_lcm(lcm_t* lcm)
{
    mbot_lcm_msgs_timestamp_t_subscribe(lcm, MBOT_TIMESYNC_CHANNEL, &timestamp_lcm_handler, NULL);
    mbot_lcm_msgs_pose2D_t_subscribe(lcm, MBOT_ODOMETRY_RESET_CHANNEL, &reset_odom_lcm_handler, NULL);
    mbot_lcm_msgs_mbot_encoders_t_subscribe(lcm, MBOT_ENCODERS_RESET_CHANNEL, &reset_encoders_lcm_handler, NULL);
    mbot_lcm_msgs_mbot_motor_pwm_t_subscribe(lcm, MBOT_MOTOR_PWM_CMD_CHANNEL, &mbot_motor_pwm_cmd_lcm_handler, NULL);
    mbot_lcm_msgs_mbot_motor_vel_t_subscribe(lcm, MBOT_MOTOR_VEL_CMD_CHANNEL, &mbot_motor_vel_cmd_lcm_handler, NULL);
    mbot_lcm_msgs_twist2D_t_subscribe(lcm, MBOT_VEL_CMD_CHANNEL, &mbot_vel_cmd_lcm_handler, NULL);
}

int main(int argc, char** argv)
{
    printf("Starting the serial/lcm shim...\r\n");
    // Register signal and signal handler
    signal(SIGINT, signal_callback_handler);

    printf("Making the lcm instance...\r\n");
    lcmInstance = lcm_create(MULTICAST_URL);
    printf("Starting the lcm handle thread...\r\n");

    pthread_t lcmThread;
    pthread_create(&lcmThread, NULL, handle_lcm, lcmInstance);

    printf("Subscribing to lcm...\r\n");
    subscribe_lcm(lcmInstance);

    printf("Init Serial....\r\n");
    int ser_dev = open("/dev/ttyACM1", O_RDWR);
    tcgetattr(ser_dev, &options);
    cfsetspeed(&options, B115200);
    options.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    options.c_cflag |= CS8 | CREAD | CLOCAL;
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ISIG | ECHO | IEXTEN); /* Set non-canonical mode */
    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 0;
    cfmakeraw(&options);
    tcflush(ser_dev, TCIFLUSH);
    tcsetattr(ser_dev, TCSANOW, &options);
    if (ser_dev < 0)
    {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        running = false;
    }
    if(tcgetattr(ser_dev, &options) != 0)
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        running = false;
    }
    comms_init_protocol(&ser_dev);
    comms_init_topic_data();
    register_topics();

    printf("Starting the serial thread...\r\n");
    pthread_t serialThread;
    pthread_create(&serialThread, NULL, comms_listener_loop, NULL);

    printf("running!\r\n");
    pthread_join(lcmThread, NULL);

    printf("stopped the lcm thread...\r\n");
    pthread_cancel(serialThread);
    pthread_join(serialThread, NULL);
    printf("stopped the serial thread...\r\n");
    close(ser_dev);
    printf("closed the serial port...\r\n");
    printf("exiting!\r\n");
    return 0;
}
