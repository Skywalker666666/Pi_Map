#ifndef PI_MSG_COM_H_
#define PI_MSG_COM_H_

typedef enum pi_msg_send_status_
{
    
    PI_MSG_SEND_STATUS_FAILED = -1,
    PI_MSG_SEND_STATUS_SUCCESS = 0
}pi_msg_send_status;

typedef struct pi_msg_envelope_t_
{
    // used for message filtering
    unsigned char filter[16];

    // type of the package
    unsigned int type;

    // counts of bytes in data block
    unsigned int length;

    // placeholder for the start address of actual body info.
    void *data;

}pi_msg_envelope_t, *p_pi_msg_envelope_t;

#define SIZEOF_PI_MSG_ENVELOPE_HEADER ((sizeof(pi_msg_envelope_t)) - sizeof(void *))

typedef enum pi_msg_type_t_
{
	// dummy message
	PIMSG_NULL = 0,

	// system message
	PIMSG_SYSTEM_NULL = 1,

	// message from UI module
	PIMSG_UI_NULL = 128,
	PIMSG_UI_REQUEST_SET_TARGET,
	PIMSG_UI_REQUEST_LLOOP_START,
	PIMSG_UI_REQUEST_CANCEL,

	// message from chassis&control module
	PIMSG_CPC_NULL = 256,
	PIMSG_CPC_PUBLISH_POS,
	PIMSG_CPC_PUBLISH_PLANPATH,
	PIMSG_CPC_PUBLISH_RADAR,

    // message from PIRobot module
    PIMSG_ROBOT_POS_2D = 320,

	// message from Localization module
	PIMSG_LOCALIZATION_NULL = 384,
	PIMSG_LOCALIZATION_VIO_PUBLISH_POS,
	PIMSG_LOCALIZATION_STEREO_PUBLISH_OBSTACLE_INFO,
	PIMSG_LOCALIZATION_IMAGE_RAWDATA,
	PIMSG_LOCALIZATION_IMU_RAWDATA,
	PIMSG_LOCALIZATION_IMAGE_RAWDATA_DEBUG,

	// message from VIDAR module
	PIMSG_VIDAR_NULL = 512,

	// message from LaneDetection module
	PIMSG_LANEDET_NULL = 640
}pi_msg_type_t;

typedef struct gps_2d_
{
    double lat;
    double lon;
}gps_2d;

typedef struct chassis_pos_
{
    double lat;
    double lon;
    double heading;

    double wheel_angle;
}chassis_pos;


#endif
