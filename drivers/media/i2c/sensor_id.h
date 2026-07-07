#ifndef _SENSOR_ID_H_
#define _SENSOR_ID_H_

#define SENSOR_MODE_STRING_LENGTH 32

#define GENERIC_SENSOR_ID_REG 0x3015
#define GENERIC_SENSOR_ID_REG2 0x300A
#define GENERIC_SENSOR_ID_REG3 0x3894

enum sensor_id {
    SENSOR_ID_IMX334_IMX715 = 0x00,
    SENSOR_ID_IMX675 = 0x04,
    SENSOR_ID_IMX678 = 0x02,
    SENSOR_ID_IMX664 = 0x04
};

// if a second order comparison is needed
enum sensor_val {
    IMX715_SENSOR_ID_VAL = 0xB6,
    IMX334_SENSOR_ID_VAL = 0x0a,
    IMX664_SENSOR_ID_VAL = 0x08,
    IMX675_SENSOR_ID_VAL = 0x04 
};

#endif
