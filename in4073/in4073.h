/*------------------------------------------------------------------
 *  in4073.h -- defines, globals, function prototypes
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */

#ifndef IN4073_H__
#define IN4073_H__

#include <inttypes.h>
#include <stdio.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ml.h"
#include "app_util_platform.h"
#include <math.h>
#include "msg2payload.h"

#define RED		22
#define YELLOW		24
#define GREEN		28
#define BLUE		30
#define INT_PIN		5

bool demo_done;

enum states {
  Safe_Mode,            // Mode 0
  Panic_Mode,           // Mode 1
  Manual_Mode,          // Mode 2
  Calibration_Mode,     // Mode 3
  Yaw_Mode,             // Mode 4
  Full_Mode,            // Mode 5
  Raw_Mode,             // Mode 6
  Height_Mode,          // Mode 7
  Wireless_Mode         // Mode 8
} state;

// Control
int16_t motor[4],ae[4];
int16_t axis[4];
int16_t sp_avg, sq_avg, sr_avg;
int16_t sax_avg, say_avg, saz_avg;
bool calibration_done; // Update after the calibration is done
bool motors_off; // Update according to the readings
void run_filters_and_control();
int b, d, p;


// Timers
#define TIMER_PERIOD	25 //50ms=20Hz (MAX 23bit, 4.6h)
void timers_init(void);
uint32_t get_time_us(void);
bool check_timer_flag(void);
void clear_timer_flag(void);

// GPIO
void gpio_init(void);

// Queue
#define QUEUE_SIZE 256
typedef struct {
	uint8_t Data[QUEUE_SIZE];
	uint16_t first,last;
  	uint16_t count;
} queue;
void init_queue(queue *q);
void enqueue(queue *q, char x);
char dequeue(queue *q);
void processPkt();

// UART
#define RX_PIN_NUMBER  16
#define TX_PIN_NUMBER  14
queue rx_queue;
queue tx_queue;
uint32_t last_correct_checksum_time;
void uart_init(void);
void uart_put(uint8_t);

// TWI
#define TWI_SCL	4
#define TWI_SDA	2
void twi_init(void);
bool i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t const *data);
bool i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);

// MPU wrapper
int16_t phi, theta, psi;
int16_t sp, sq, sr;
int16_t sax, say, saz;
uint8_t sensor_fifo_count;
void imu_init(bool dmp, uint16_t interrupt_frequency); // if dmp is true, the interrupt frequency is 100Hz - otherwise 32Hz-8kHz
void get_dmp_data(void);
void get_raw_sensor_data(void);
bool check_sensor_int_flag(void);
void clear_sensor_int_flag(void);

// Barometer
int32_t pressure;
int32_t temperature;
void read_baro(void);
void baro_init(void);

// ADC
uint16_t bat_volt;
void adc_init(void);
void adc_request_sample(void);

// Flash
bool spi_flash_init(void);
bool flash_chip_erase(void);
bool flash_write_byte(uint32_t address, uint8_t data);
bool flash_write_bytes(uint32_t address, uint8_t *data, uint32_t count);
bool flash_read_byte(uint32_t address, uint8_t *buffer);
bool flash_read_bytes(uint32_t address, uint8_t *buffer, uint32_t count);

// BLE
queue ble_rx_queue;
queue ble_tx_queue;
volatile bool radio_active;
void ble_init(void);
void ble_send(void);

// PROCESSING
void process_key(uint8_t c);
void processRecMsg();
void changeMode();
void changeMov(uint8_t *msg);
void changeKbParam(uint8_t *msg);

#endif // IN4073_H__
