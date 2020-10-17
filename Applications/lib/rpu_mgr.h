#ifndef Rpu_Mgr_h
#define Rpu_Mgr_h

#include "../lib/twi0_bsd.h"

// enumeraiton names for ADC from manager
typedef enum ADC_CH_MGR_enum {
    ADC_CH_MGR_ALT_I, // manager analog channel 0
    ADC_CH_MGR_ALT_V, // manager analog channel 1
    ADC_CH_MGR_PWR_I, // manager analog channel 6
    ADC_CH_MGR_PWR_V, // manager analog channel 7
    ADC_CH_MGR_MAX_NOT_A_CH // not a channel
} ADC_CH_MGR_t;

extern uint8_t mgr_twiErrorCode;

extern void i2c_ping(void);
extern uint8_t i2c_set_Rpu_shutdown(void);
extern uint8_t i2c_detect_Rpu_shutdown(void);
extern char i2c_get_Rpu_address(void);
extern int i2c_get_adc_from_manager(uint8_t channel, TWI0_LOOP_STATE_t *loop_state);
extern uint8_t i2c_read_status(void);
extern void i2c_daynight_cmd(uint8_t dn_callback_addr, uint8_t dn_callback_route, uint8_t d_callback_route, uint8_t n_callback_route);
extern void i2c_battery_cmd(uint8_t bm_callback_addr, uint8_t bm_callback_route, uint8_t bm_enable);
extern void i2c_shutdown_cmd(uint8_t hs_callback_addr, uint8_t hs_callback_route, uint8_t hs_cntl);
extern unsigned long i2c_ul_rwoff_access_cmd(uint8_t command, uint8_t rw_offset, unsigned long update_with, TWI0_LOOP_STATE_t *loop_state);
extern int i2c_int_access_cmd(uint8_t command, int update_with, TWI0_LOOP_STATE_t *loop_state);
extern int i2c_int_rwoff_access_cmd(uint8_t command, uint8_t rw_offset, int update_with, TWI0_LOOP_STATE_t *loop_state);
float i2c_float_access_cmd(uint8_t command, uint8_t select, float *update_with, TWI0_LOOP_STATE_t *loop_state);

// values used for i2c_*_rwoff_access_cmd
#define RW_READ_BIT 0x00
#define RW_WRITE_BIT 0x80

// values used for i2c_uint8_access_cmd
#define DAYNIGHT_STATE 23

// values used for i2c_ul_rwoff_access_cmd
#define SHUTDOWN_UL_CMD 6
#define SHUTDOWN_TTL 0 /*offset to access shutdown_halt_ttl_limit */
#define SHUTDOWN_DELAY 1 /*offset to access shutdown_delay_limit */
#define SHUTDOWN_WEARLEVEL 2 /*offset to access shutdown_wearleveling_limit */
#define SHUTDOWN_KRUNTIME 3
#define SHUTDOWN_STARTED_AT 4
#define SHUTDOWN_HALT_CHK_AT 5
#define SHUTDOWN_WEARLVL_DONE_AT 6
#define DAYNIGHT_UL_CMD 21
#define DAYNIGHT_MORNING_DEBOUNCE 0 /*offset to access daynight_morning_debounce */
#define DAYNIGHT_EVENING_DEBOUNCE 1 /*offset to access daynight_evening_debounce */
#define DAYNIGHT_ELAPSED_TIMER 2
#define DAYNIGHT_ELAPSED_NIGHT 3
#define DAYNIGHT_ELAPSED_DAY 4
#define DAYNIGHT_ALT_MTI_NIGHT 5 /* (accumulation of ALT_I readings every 10mSec / 10**6) at night event */
#define DAYNIGHT_PWR_MTI_NIGHT 6 /* (accumulation of PWR_I readings every 10mSec / 10**6) at night event */
#define DAYNIGHT_ALT_MTI_DAY 7 /* (accumulation of ALT_I readings every 10mSec / 10**6) at day event */
#define DAYNIGHT_PWR_MTI_DAY 8 /* (accumulation of PWR_I readings every 10mSec / 10**6) at day event */
#define BATTERY_UL_CMD 18
#define BATTERY_CHARGE_PWM 0 /*offset to access alt_pwm_accum_charge_time*/

// values used for i2c_int_rwoff_access_cmd
#define SHUTDOWN_INT_CMD 5
#define SHUTDOWN_HALT_CURR_OFFSET 0
#define DAYNIGHT_INT_CMD 20
#define DAYNIGHT_MORNING_THRESHOLD 0
#define DAYNIGHT_EVENING_THRESHOLD 1
#define BATTERY_INT_CMD 17
#define BATTERY_HIGH 0 /*offset to access battery_high_limit */
#define BATTERY_LOW 1 /*offset to access battery_low_limit */
#define BATTERY_HOST 2 /*offset to access battery_host_limit */

#endif // Rpu_Mgr_h
