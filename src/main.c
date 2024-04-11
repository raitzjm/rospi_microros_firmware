#include <version.h>

#if ZEPHYR_VERSION_CODE >= ZEPHYR_VERSION(3,1,0)
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/posix/time.h>
#include <zephyr/drivers/sensor.h>

#else
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <posix/time.h>
#endif

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/magnetic_field.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>
#include <microros_transports.h>
#include <type_utilities.h>

#include <math.h>



#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);for(;;){};}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t mag_publisher;
sensor_msgs__msg__MagneticField mag_msg;

double mag_x,mag_y,mag_z;

static micro_ros_utilities_memory_conf_t conf = {0};
bool start_publisher = 0;

K_MSGQ_DEFINE(mag_msgq, sizeof(struct sensor_value) *3 , 10, 1);

#ifdef CONFIG_LIS3MDL_TRIGGER


static void lis3mdl_trigger_handler(const struct device *dev,
				    const struct sensor_trigger *trig)
{
	struct sensor_value magn_xyz[3];
	sensor_sample_fetch_chan(dev, trig->chan);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, magn_xyz);
	if(start_publisher==1)
	{
		if(k_msgq_put(&mag_msgq, &magn_xyz, K_NO_WAIT)!=0)
		{
			k_msgq_purge(&mag_msgq);
			printf("MSG QUEUE Purged\n");
		}
	}
	
	
	
}
#endif


void publish_mag(struct sensor_value magn_xyz[3])					//(rcl_timer_t * timer, int64_t last_call_time)
{
	
	mag_x=sensor_value_to_double(&magn_xyz[0]);
	mag_y=sensor_value_to_double(&magn_xyz[1]),
	mag_z=sensor_value_to_double(&magn_xyz[2]);
	
	
	mag_msg.magnetic_field.x = mag_x * 10e4;
	mag_msg.magnetic_field.y = mag_y * 10e4;
	mag_msg.magnetic_field.z = mag_z * 10e4;
	uint64_t time_ns =rmw_uros_epoch_nanos();
	mag_msg.header.stamp.nanosec = time_ns % RCL_S_TO_NS(1);
	mag_msg.header.stamp.sec = (int32_t) RCL_NS_TO_S(time_ns);
	sprintf(mag_msg.header.frame_id.data, "imu1");
	mag_msg.header.frame_id.size  = strlen(mag_msg.header.frame_id.data);
	RCSOFTCHECK(rcl_publish(&mag_publisher, &mag_msg, NULL));
		
}

void main(void)
{
	
	printk("Hello Test: \n");
	const struct device *const mag_dev = DEVICE_DT_GET(DT_NODELABEL(lis3mdl));

	if (mag_dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		printf("\nError: no device found.\n");
	}
	
	
	if (!device_is_ready(mag_dev)) {                              
		printf("%s: device not ready.\n",mag_dev);
	
	}
	else
	{
		//printf("%s Device ready!\n",mag_dev);
	}

	
	#ifdef CONFIG_LIS3MDL_TRIGGER
	struct sensor_trigger trig;
	int cnt = 1;
	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_MAGN_XYZ;
	sensor_trigger_set(mag_dev, &trig, lis3mdl_trigger_handler);
	#endif
	
	rmw_uros_set_custom_transport(
		MICRO_ROS_FRAMING_REQUIRED,
		(void *) DEVICE_DT_GET(DT_ALIAS(uros_serial_port)),
		//NULL,
		zephyr_transport_open,
		zephyr_transport_close,
		zephyr_transport_write,
		zephyr_transport_read
	);

	/**
	 * Loop until micro-ROS Agent is up
	 */
	while (RMW_RET_OK != rmw_uros_ping_agent(1000, 1)) {
		printf("Please, start your micro-ROS Agent first\n");
		k_sleep(K_MSEC(100));
	}

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));


	//Allocate memory for mag message
	conf.max_string_capacity =50;
	conf.max_ros2_type_sequence_capacity = 5;
	conf.max_basic_type_sequence_capacity = 5;



	bool succes = micro_ros_utilities_create_message_memory(
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,MagneticField),
		&mag_msg,
		conf
	);

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "mag", "", &support));
	rmw_uros_sync_session(1000);

	// create publisher
	RCCHECK(rclc_publisher_init_best_effort(
		&mag_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
		"imu/mag"));


	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	//RCCHECK(rclc_executor_add_timer(&executor, &timer));

	start_publisher=1;

	while(1){
		#ifndef CONFIG_LIS3MDL_TRIGGER
		printf("Called NO Trigger \n");
		if (sensor_sample_fetch(mag_dev) < 0) {
			//printf("LIS3MDL Sensor sample update error\n");
			return 0;
		}
		#endif

		
		//rclc_executor_spin_some(&executor,1000);
		struct sensor_value magn_xyz [3]; 
		
		if(k_msgq_get(&mag_msgq,&magn_xyz, K_FOREVER)==0)
		{
			printf("Received,  queue len: %d\n",k_msgq_num_used_get(&mag_msgq));
			publish_mag(magn_xyz);
		}
		
	

		
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&mag_publisher, &node))
	RCCHECK(rcl_node_fini(&node))
}
