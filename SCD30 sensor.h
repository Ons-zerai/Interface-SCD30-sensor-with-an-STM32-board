
void co2_sensor_measure_start(void);
void co2_sensor_measure_read(void);
void co2_sensor_get_ready_status(void);



#define sensor_address  0xC2 //address:0x61 with write argument:0
#define sensor_address_read  0xC3 //address:0x61 with read argument:1
#define senor_address 0x61<<1 // 7bits sensor address





