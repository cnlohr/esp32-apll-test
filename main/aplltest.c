/* APLL Test

   This is my test of the APLL on the ESP32. Based on blink.c, all other code is

   Copyright <>< 2020 Charles Lohr, under the MIT-x11 or NewBSD Licenses.

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   This is also based off of https://gist.github.com/cnlohr/2b9f8a26e891aa1a118f634cff9d04fe
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "soc/i2s_reg.h"
#include "esp32/rom/lldesc.h"
#include "driver/periph_ctrl.h"
#include "hal/i2s_types.h"
#include "soc/rtc_cntl_reg.h"
#include "regi2c_apll.h"
#include "soc/rtc.h"

#include "soc/efuse_reg.h" //For reading chip rev.
#include "regi2c_ctrl.h" //For APLL setup

#define I2S_NO 0
#define SERIAL_TEST 0

#define ETS_I2Sx_INUM(x) (x*10+13)

#define PERIPH_I2Sn_MODULE(x) \
	((x)?(PERIPH_I2S1_MODULE):(PERIPH_I2S0_MODULE))

#if I2S_NO == 1
#define I2SnI_BCK_OUT_IDX I2S1I_BCK_OUT_IDX
#define I2SnI_WS_OUT_IDX  I2S1I_WS_OUT_IDX
#define I2SnI_DATA_IN0_IDX I2S1I_DATA_IN0_IDX
#define I2SnI_DATA_IN1_IDX I2S1I_DATA_IN1_IDX
#define I2SnI_DATA_IN2_IDX I2S1I_DATA_IN2_IDX
#define I2SnI_DATA_IN3_IDX I2S1I_DATA_IN3_IDX
#define I2SnI_DATA_IN4_IDX I2S1I_DATA_IN4_IDX
#define I2SnI_DATA_IN5_IDX I2S1I_DATA_IN5_IDX
#define I2SnI_DATA_IN6_IDX I2S1I_DATA_IN6_IDX
#define I2SnI_DATA_IN7_IDX I2S1I_DATA_IN7_IDX
#define I2SnI_DATA_IN9_IDX I2S1I_DATA_IN9_IDX
#define I2SnI_H_ENABLE_IDX I2S1I_H_ENABLE_IDX
#define I2SnI_H_SYNC_IDX   I2S1I_H_SYNC_IDX
#define I2SnI_V_SYNC_IDX   I2S1I_V_SYNC_IDX
#define ETS_I2Sn_INTR_SOURCE ETS_I2S1_INTR_SOURCE
#else
#define I2SnI_BCK_OUT_IDX I2S0I_BCK_OUT_IDX
#define I2SnI_WS_OUT_IDX  I2S0I_WS_OUT_IDX
#define I2SnI_DATA_IN0_IDX I2S0I_DATA_IN0_IDX
#define I2SnI_DATA_IN1_IDX I2S0I_DATA_IN1_IDX
#define I2SnI_DATA_IN2_IDX I2S0I_DATA_IN2_IDX
#define I2SnI_DATA_IN3_IDX I2S0I_DATA_IN3_IDX
#define I2SnI_DATA_IN4_IDX I2S0I_DATA_IN4_IDX
#define I2SnI_DATA_IN5_IDX I2S0I_DATA_IN5_IDX
#define I2SnI_DATA_IN6_IDX I2S0I_DATA_IN6_IDX
#define I2SnI_DATA_IN7_IDX I2S0I_DATA_IN7_IDX
#define I2SnI_DATA_IN9_IDX I2S0I_DATA_IN9_IDX
#define I2SnI_H_ENABLE_IDX I2S0I_H_ENABLE_IDX
#define I2SnI_H_SYNC_IDX   I2S0I_H_SYNC_IDX
#define I2SnI_V_SYNC_IDX   I2S0I_V_SYNC_IDX
#define ETS_I2Sn_INTR_SOURCE ETS_I2S0_INTR_SOURCE
#endif


static void IRAM_ATTR i2s_isr(void* arg);
static esp_err_t dma_desc_init();
static void i2s_init();
static void i2s_run();
static void enable_out_clock();

#define BUFF_SIZE_BYTES 2048

uint32_t i2sbuffer[4][BUFF_SIZE_BYTES/4] __attribute__((aligned(128)));
static lldesc_t s_dma_desc[4];
static int i2s_running;
volatile unsigned isr_count;

static void IRAM_ATTR i2s_isr(void* arg) {
    REG_WRITE(I2S_INT_CLR_REG(I2S_NO), (REG_READ(I2S_INT_RAW_REG(I2S_NO)) & 0xffffffc0) | 0x3f);
    ++isr_count;
}

static esp_err_t dma_desc_init()
{
    for (int i = 0; i < 4; ++i) {
        s_dma_desc[i].length = BUFF_SIZE_BYTES;     // size of a single DMA buf
        s_dma_desc[i].size = BUFF_SIZE_BYTES;       // total size of the chain
        s_dma_desc[i].owner = 1;
        s_dma_desc[i].sosf = 1;
        s_dma_desc[i].buf = (uint8_t*) i2sbuffer[i];
        s_dma_desc[i].offset = i;
        s_dma_desc[i].empty = 0;
        s_dma_desc[i].eof = 1;
        s_dma_desc[i].qe.stqe_next = &s_dma_desc[(i+1)&1];
    }
    return ESP_OK;
}


void local_rtc_clk_apll_enable(bool enable, uint32_t sdm0, uint32_t sdm1, uint32_t sdm2, uint32_t o_div)
{
	#define APLL_SDM_STOP_VAL_2_REV1    0x49
	#define APLL_SDM_STOP_VAL_2_REV0    0x69
	#define APLL_SDM_STOP_VAL_1         0x09
	#define APLL_CAL_DELAY_1            0x0f
	#define APLL_CAL_DELAY_2            0x3f
	#define APLL_CAL_DELAY_3            0x1f

    REG_SET_FIELD(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_PLLA_FORCE_PD, enable ? 0 : 1);
    REG_SET_FIELD(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_PLLA_FORCE_PU, enable ? 1 : 0);

    if (!enable &&
        REG_GET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_SOC_CLK_SEL) != RTC_CNTL_SOC_CLK_SEL_PLL) {
        REG_SET_BIT(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_BIAS_I2C_FORCE_PD);
    } else {
        REG_CLR_BIT(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_BIAS_I2C_FORCE_PD);
    }

    if (enable) {
        uint8_t sdm_stop_val_2 = APLL_SDM_STOP_VAL_2_REV1;
        uint32_t is_rev0 = (GET_PERI_REG_BITS2(EFUSE_BLK0_RDATA3_REG, 1, 15) == 0);
        if (is_rev0) {
            sdm0 = 0;
            sdm1 = 0;
            sdm_stop_val_2 = APLL_SDM_STOP_VAL_2_REV0;
        }
        REGI2C_WRITE_MASK(I2C_APLL, I2C_APLL_DSDM2, sdm2);
        REGI2C_WRITE_MASK(I2C_APLL, I2C_APLL_DSDM0, sdm0);
        REGI2C_WRITE_MASK(I2C_APLL, I2C_APLL_DSDM1, sdm1);
        REGI2C_WRITE(I2C_APLL, I2C_APLL_SDM_STOP, APLL_SDM_STOP_VAL_1);
        REGI2C_WRITE(I2C_APLL, I2C_APLL_SDM_STOP, sdm_stop_val_2);
        REGI2C_WRITE_MASK(I2C_APLL, I2C_APLL_OR_OUTPUT_DIV, o_div);

        /* calibration */
        REGI2C_WRITE(I2C_APLL, I2C_APLL_IR_CAL_DELAY, APLL_CAL_DELAY_1);
        REGI2C_WRITE(I2C_APLL, I2C_APLL_IR_CAL_DELAY, APLL_CAL_DELAY_2);
        REGI2C_WRITE(I2C_APLL, I2C_APLL_IR_CAL_DELAY, APLL_CAL_DELAY_3);

		printf( "%08x\n", REGI2C_READ(I2C_APLL, I2C_APLL_DSDM0 ) );
        /* wait for calibration end */
		int retries = 0;
        while (!(REGI2C_READ_MASK(I2C_APLL, I2C_APLL_OR_CAL_END)) && retries < 1000 ) {
		printf( "%08x\n", REGI2C_READ(I2C_APLL, I2C_APLL_DSDM0 ) );
            /* use esp_rom_delay_us so the RTC bus doesn't get flooded */
            esp_rom_delay_us(1);
			retries++;
        }
    }
}


static void enable_out_clock() {

#if 0
    periph_module_enable(PERIPH_LEDC_MODULE);
    ledc_timer_config_t timer_conf;
    timer_conf.bit_num = 3;
    timer_conf.freq_hz = I2S_HZ;
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_conf.timer_num = LEDC_TIMER_0;
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        //ESP_LOGE(TAG, "ledc_timer_config failed, rc=%x", err);
    }

    ledc_channel_config_t ch_conf;
    ch_conf.channel = LEDC_CHANNEL_0;
    ch_conf.timer_sel = LEDC_TIMER_0;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.duty = 4;
    ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    ch_conf.gpio_num = 17; //s_config.pin_xclk; 
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) {
        //ESP_LOGE(TAG, "ledc_channel_config failed, rc=%x", err);
    }
#endif

}


static void i2s_init()
{
	//Actually enable PLLA (My calculations are a little low, comparing to reality not sure why.)
	//Example SDM2=8 -> 40 * (8+4) = 480 / (2*(10+2)) = 20 MHz received 5MHz (why div4?)
	//local_rtc_clk_apll_enable( 1, 0/*sdm0*/, 0 /*sdm1*/, 8 /*sdm2*/, 10 /*div*/ );

	//480/4 = 120 -> 120 MHz out
	local_rtc_clk_apll_enable( 1, 0/*sdm0*/, 200 /*sdm1*/, 8 /*sdm2*/, 0 /*div*/ );


//#define I2S_D0 4
#define I2S_D1 5
#define I2S_D2 22
#define I2S_D3 19
#define I2S_D4 36
#define I2S_D5 39
#define I2S_D6 34
#define I2S_D7 35
#define I2S_CLK 21
#define I2S_WCLK 18

    gpio_num_t pins[] = {
            I2S_D7,
            I2S_D6,
            I2S_D5,
            I2S_D4,
            I2S_D3,
            I2S_D2,
            I2S_D1,
            //I2S_D0,
			I2S_CLK,
			I2S_WCLK,
    };

    xt_set_interrupt_handler(ETS_I2Sx_INUM(I2S_NO), &i2s_isr, NULL);
    intr_matrix_set(0, ETS_I2Sn_INTR_SOURCE, ETS_I2Sx_INUM(I2S_NO));

    gpio_config_t conf = {
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
    };
    for (int i = 0; i < sizeof(pins)/sizeof(gpio_num_t); ++i) {
        conf.pin_bit_mask = 1LL << pins[i];
        gpio_config(&conf);
    }


	//Override some bits so we can tell what the actual channel mapping is.
    gpio_matrix_in(0x38,    I2SnI_DATA_IN0_IDX, false);
    gpio_matrix_in(I2S_D1,    I2SnI_DATA_IN1_IDX, false);
    gpio_matrix_in(I2S_D2,    I2SnI_DATA_IN2_IDX, false);
    gpio_matrix_in(I2S_D3,    I2SnI_DATA_IN3_IDX, false);
    gpio_matrix_in(I2S_D4,    I2SnI_DATA_IN4_IDX, false);
    gpio_matrix_in(I2S_D5,    I2SnI_DATA_IN5_IDX, false);
    gpio_matrix_in(I2S_D6,    I2SnI_DATA_IN6_IDX, false);
    gpio_matrix_in(I2S_D7,    I2SnI_DATA_IN7_IDX, false);

    gpio_matrix_in(0x38,    I2SnI_DATA_IN9_IDX, false);

	//This magic is needed to enable the bus!
    gpio_matrix_in(0x38,    I2SnI_V_SYNC_IDX, false);
    gpio_matrix_in(0x38, 	I2SnI_H_SYNC_IDX, false);
	gpio_matrix_in(0x38,    I2SnI_H_ENABLE_IDX, false);


	conf.mode = GPIO_MODE_OUTPUT;
    conf.pin_bit_mask = 1LL << I2S_CLK;
    conf.pin_bit_mask = 1LL << I2S_WCLK;
    gpio_config(&conf);

    gpio_matrix_out(I2S_CLK,  I2SnI_BCK_OUT_IDX, false, 0 );
//    gpio_matrix_out(I2S_WCLK,  I2SnI_WS_OUT_IDX, false, 0 );



    periph_module_enable(PERIPH_I2Sn_MODULE(I2S_NO));

    SET_PERI_REG_BITS(I2S_LC_CONF_REG(I2S_NO), 0x1, 1, I2S_IN_RST_S);
    SET_PERI_REG_BITS(I2S_LC_CONF_REG(I2S_NO), 0x1, 0, I2S_IN_RST_S);
    SET_PERI_REG_BITS(I2S_LC_CONF_REG(I2S_NO), 0x1, 1, I2S_AHBM_RST_S);
    SET_PERI_REG_BITS(I2S_LC_CONF_REG(I2S_NO), 0x1, 0, I2S_AHBM_RST_S);
    SET_PERI_REG_BITS(I2S_LC_CONF_REG(I2S_NO), 0x1, 1, I2S_AHBM_FIFO_RST_S);
    SET_PERI_REG_BITS(I2S_LC_CONF_REG(I2S_NO), 0x1, 0, I2S_AHBM_FIFO_RST_S);

    SET_PERI_REG_BITS(I2S_LC_CONF_REG(I2S_NO), 0x1, 0, I2S_INDSCR_BURST_EN_S);

    SET_PERI_REG_BITS(I2S_CONF_REG(I2S_NO), 0x1, 1, I2S_RX_RESET_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(I2S_NO), 0x1, 0, I2S_RX_RESET_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(I2S_NO), 0x1, 1, I2S_RX_FIFO_RESET_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(I2S_NO), 0x1, 0, I2S_RX_FIFO_RESET_S);


#if SLAVE_MOD
    SET_PERI_REG_BITS(I2S_CONF_REG(I2S_NO), 0x1, 1, I2S_RX_SLAVE_MOD_S);
#else
    SET_PERI_REG_BITS(I2S_CONF_REG(I2S_NO), 0x1, 0, I2S_RX_SLAVE_MOD_S);
#endif

//If we were doing output, we would want this.
//    SET_PERI_REG_BITS(I2S_CONF2_REG(I2S_NO), 0x1, 1, I2S_LCD_EN_S);

#if SERIAL_TEST
    SET_PERI_REG_BITS(I2S_CONF2_REG(I2S_NO), 0x1, 0, I2S_CAMERA_EN_S);
	//No idea what these were for.
	//WRITE_PERI_REG( I2S_TIMING_REG(I2S_NO), 0xffffffff );
	//WRITE_PERI_REG( I2S_CONF_SIGLE_DATA_REG(0), 0xffffffff );
    SET_PERI_REG_BITS(I2S_CONF_REG(I2S_NO), 0x1, 0, I2S_RX_RIGHT_FIRST_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(I2S_NO), 0x1, 0, I2S_RX_MSB_RIGHT_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(I2S_NO), 0x1, 0, I2S_RX_MSB_SHIFT_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(I2S_NO), 0x1, 1, I2S_RX_SHORT_SYNC_S);
#else
    SET_PERI_REG_BITS(I2S_CONF2_REG(I2S_NO), 0x1, 1, I2S_CAMERA_EN_S);
#endif


	//Not sure, using SET_PERI_REG_BITS on this register is buggy.
    WRITE_PERI_REG(I2S_CLKM_CONF_REG(I2S_NO),
                   I2S_CLKA_ENA | I2S_CLK_EN |
                   (0 << I2S_CLKM_DIV_A_S) |
                   (0 << I2S_CLKM_DIV_B_S) |
                   (1 << I2S_CLKM_DIV_NUM_S)); //Supposed to be 2 at minimum switching to 1 is fruitless.


    SET_PERI_REG_BITS(I2S_CONF_REG(I2S_NO), 0x1, 1, I2S_RX_MONO_S);       //Seem ignored in parallel mode

    SET_PERI_REG_BITS(I2S_FIFO_CONF_REG(I2S_NO), 0x1, 1, I2S_DSCR_EN_S);
    SET_PERI_REG_BITS(I2S_FIFO_CONF_REG(I2S_NO), 0x1, 1, I2S_RX_FIFO_MOD_FORCE_EN_S);

	//Changing these to 16/8 seems to have no impact.
	//This is 32 on boot by default.
    SET_PERI_REG_BITS(I2S_FIFO_CONF_REG(I2S_NO), I2S_RX_DATA_NUM, 32, I2S_RX_DATA_NUM_S);

	//16 default at startup, seems ignored.
	SET_PERI_REG_BITS(I2S_SAMPLE_RATE_CONF_REG(I2S_NO), I2S_RX_BITS_MOD, 8, I2S_RX_BITS_MOD_S);

/* 
  For PARALLEL I2S INPUT...

  Configriation Map:
		Meaning:  cells: {bits per clock}/4-byte configuration {msb...lsb, C-style HEX}
		Meaning:  header: FIFO_MOD(MBytes/Sec @ 10 MHz bit clock)

         * I2S_RX_BITS_MOD = 8 / 16 (no impact noticed)
         * I2S_RX_MONO = 0/1 seems to have no impact.
      FIFO_MOD->    0(20)        1*(10)           2(40)         3*(20)  4...(0)
  CHAN MOD
      0            16/HLHL       8/0000          32/HL00       16/0000   (dead)
      1            16/HLHL       8/HLHL          32/HL00       16/HL00
      2            16/HLHL       8/HLHL          32/HL00       16/HL00
      3            16/HLHL       8/0000          32/HL00       16/0000
          * = I'm confused, where do the other bits go?

*/

#if SERIAL_TEST
    SET_PERI_REG_BITS(I2S_FIFO_CONF_REG(I2S_NO), I2S_RX_FIFO_MOD, 0, I2S_RX_FIFO_MOD_S); //0 = 16-bit 2-channel data.
    SET_PERI_REG_BITS(I2S_CONF_CHAN_REG(I2S_NO), I2S_RX_CHAN_MOD, 0, I2S_RX_CHAN_MOD_S); //No modification to channel.
#else
    SET_PERI_REG_BITS(I2S_FIFO_CONF_REG(I2S_NO), I2S_RX_FIFO_MOD, 0, I2S_RX_FIFO_MOD_S); //0...7 valid
    SET_PERI_REG_BITS(I2S_CONF_CHAN_REG(I2S_NO), I2S_RX_CHAN_MOD, 0, I2S_RX_CHAN_MOD_S); //0...4 valid
#endif


	SET_PERI_REG_BITS(I2S_SAMPLE_RATE_CONF_REG(I2S_NO), I2S_RX_BCK_DIV_NUM, 2, I2S_RX_BCK_DIV_NUM_S);  

	//ERR NOTE -> Under current config these values are 2x the Hz of what is listed here.
	//Once set, 1 = you can read all 8 bits at 40 MHz. -> Warning, this is "technically" prhibited. It seems to be lying about this speed.
	//Once set, 2 = you can read all 8 bits at 20 MHz. -> Practically highest speed.
	//			3 = 13.33333 MHz
	//			4 = 10.0MHz
}


static void i2s_fill_buf() {
    ESP_INTR_DISABLE(ETS_I2Sx_INUM(I2S_NO));

    SET_PERI_REG_BITS(I2S_RXEOF_NUM_REG(I2S_NO), I2S_RX_EOF_NUM, (BUFF_SIZE_BYTES - 8) / 2, I2S_RX_EOF_NUM_S);
    SET_PERI_REG_BITS(I2S_IN_LINK_REG(I2S_NO), I2S_INLINK_ADDR, ((uint32_t) &s_dma_desc), I2S_INLINK_ADDR_S);
    SET_PERI_REG_BITS(I2S_IN_LINK_REG(I2S_NO), 0x1, 1, I2S_INLINK_START_S);

    REG_WRITE(I2S_INT_CLR_REG(I2S_NO), (REG_READ(I2S_INT_RAW_REG(I2S_NO)) & 0xffffffc0) | 0x3f);

    REG_WRITE(I2S_CONF_REG(I2S_NO), REG_READ(I2S_CONF_REG(I2S_NO)) & 0xfffffff0);
    (void) REG_READ(I2S_CONF_REG(I2S_NO));
    REG_WRITE(I2S_CONF_REG(I2S_NO), (REG_READ(I2S_CONF_REG(I2S_NO)) & 0xfffffff0) | 0xf);
    (void) REG_READ(I2S_CONF_REG(I2S_NO));
    REG_WRITE(I2S_CONF_REG(I2S_NO), REG_READ(I2S_CONF_REG(I2S_NO)) & 0xfffffff0);
    while (GET_PERI_REG_BITS2(I2S_STATE_REG(I2S_NO), 0x1, I2S_TX_FIFO_RESET_BACK_S));

    SET_PERI_REG_BITS(I2S_INT_ENA_REG(I2S_NO), 0x1, 1, I2S_IN_DONE_INT_ENA_S);
    ESP_INTR_ENABLE(ETS_I2Sx_INUM(I2S_NO));
	SET_PERI_REG_BITS(I2S_CONF_REG(I2S_NO), 0x1, 1, I2S_RX_START_S);
}



void TickI2S()
{
	i2s_run();
}

static void i2s_run()
{
    // wait for vsync
    //ESP_LOGD(TAG, "Waiting for VSYNC");
    //while(gpio_get_level(s_config.pin_vsync) != 0);
    //while(gpio_get_level(s_config.pin_vsync) == 0);
    ///ESP_LOGD(TAG, "Got VSYNC");
    // wait a bit
    //delay(2);

    // start RX
//    line_count = 0;
    isr_count = 0;
    i2s_running = true;
    i2s_fill_buf();
}


void SetupI2S()
{
	enable_out_clock();
    i2s_init();
	dma_desc_init();
	i2s_run();
}


void app_main(void)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    //gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    //gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

	SetupI2S();

	int lastl = 0;

	int chm = 0;
	int fim = 0;

        uint32_t is_rev0 = (GET_PERI_REG_BITS2(EFUSE_BLK0_RDATA3_REG, 1, 15) == 0);
	printf( "Is rev0: %d\n", is_rev0 );


/*
	//Scan over all locations in the matrix in case one of the hidden IOs are hiding something neat.
	//Conclusion: No.

	int i;
	for( i = 0; i < 500; i++ )
	{
	    gpio_matrix_out(I2S_WCLK,  i, false, 0 );
        vTaskDelay(1);
	}
*/


//	int sdm2;
//	sdm2 = 1;
    while(1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
		printf("Tick %d %d %d %08x %08x\n", (isr_count-lastl)*BUFF_SIZE_BYTES, fim, chm, i2sbuffer[0][0], READ_PERI_REG(I2S_CLKM_CONF_REG(I2S_NO)) );
 //       REGI2C_WRITE_MASK(I2C_APLL, I2C_APLL_DSDM0, sdm2);
//		sdm2++;
		lastl = isr_count;
#if !SERIAL_TEST
		chm++;
		if( chm == 8 ) { chm = 0; fim++; }
		if( fim == 8 ) { fim = 0; }

		SET_PERI_REG_BITS(I2S_FIFO_CONF_REG(I2S_NO), I2S_RX_FIFO_MOD, fim, I2S_RX_FIFO_MOD_S);
		SET_PERI_REG_BITS(I2S_CONF_CHAN_REG(I2S_NO), I2S_RX_CHAN_MOD, chm, I2S_RX_CHAN_MOD_S);
#endif
    }
}
