/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>

#include "bno055.h"     // Fichiers de la librairie BNO055
#include "bno_config.h" // Configuration du BNO055 (adresses, échelles)

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	SC035_REG_SLP = 0x0100,
	SC035_REG_RST = 0x0103,

	SC035_REG_DVP_FSYNC_WIDTH = 0x3d01,
	SC035_REG_DVP_POL_CTRL = 0x3d08,
	SC035_REG_PAD_DRIVER_CAP = 0x3641,
	SC035_REG_PCLK_DLY = 0x3640,

	SC035_REG_PAD_MSB = 0x3000,
	SC035_REG_PAD_LSB = 0x3001,

	SC035_REG_LED_STROBE = 0x3361,

	// Group hold

	SC035_REG_BLC_EN = 0x3900,
	SC035_REG_BLC_AUTO = 0x3902,
	SC035_REG_BLC_CHANNEL_1 = 0x3928,
	SC035_REG_BLC_CHANNEL_2 = 0x3905,
	SC035_REG_BLC_TARGET_MSB = 0x3907,
	SC035_REG_BLC_TARGET_LSB = 0x3908,

	SC035_REG_MIRROR = 0x3221,
	SC035_REG_FLIP = 0x3221,

	SC035_REG_WIN_WIDTH_MSB = 0x3208,
	SC035_REG_WIN_WIDTH_LSB = 0x3209,
	SC035_REG_WIN_HEIGHT_MSB = 0x320a,
	SC035_REG_WIN_HEIGHT_LSB = 0x320b,
	SC035_REG_COL_START_POS_MSB = 0x3210,
	SC035_REG_COL_START_POS_LSB = 0x3211,
	SC035_REG_LINE_START_POS_LSB = 0x3212,
	SC035_REG_LINE_START_POS_MSB = 0x3213,

	SC035_REG_FRAME_LENGTH_MSB = 0x320e,
	SC035_REG_FRAME_LENGTH_LSB = 0x320f,
	SC035_REG_LINE_LENGTH_MSB = 0x320c,
	SC035_REG_LINE_LENGTH_LSB = 0x320d,

	SC035_REG_TEST_PATTERN = 0x4501,
}SC035HGS_Register;

const SC035HGS_Register sc035hgs_registers[] = {
    SC035_REG_SLP,
    SC035_REG_RST,

    SC035_REG_DVP_FSYNC_WIDTH,
    SC035_REG_DVP_POL_CTRL,
    SC035_REG_PAD_DRIVER_CAP,
    SC035_REG_PCLK_DLY,

    SC035_REG_PAD_MSB,
    SC035_REG_PAD_LSB,

    SC035_REG_LED_STROBE,

    SC035_REG_BLC_EN,
    SC035_REG_BLC_AUTO,
    SC035_REG_BLC_CHANNEL_1,
    SC035_REG_BLC_CHANNEL_2,
    SC035_REG_BLC_TARGET_MSB,
    SC035_REG_BLC_TARGET_LSB,

    SC035_REG_MIRROR, // or SC035_REG_FLIP, since they share the same address

    SC035_REG_WIN_WIDTH_MSB,
    SC035_REG_WIN_WIDTH_LSB,
    SC035_REG_WIN_HEIGHT_MSB,
    SC035_REG_WIN_HEIGHT_LSB,
    SC035_REG_COL_START_POS_MSB,
    SC035_REG_COL_START_POS_LSB,
    SC035_REG_LINE_START_POS_LSB,
    SC035_REG_LINE_START_POS_MSB,

    SC035_REG_FRAME_LENGTH_MSB,
    SC035_REG_FRAME_LENGTH_LSB,
    SC035_REG_LINE_LENGTH_MSB,
    SC035_REG_LINE_LENGTH_LSB,

    SC035_REG_TEST_PATTERN
};

struct sc031gs_regval {
	uint16_t addr;
	uint8_t val;
};

static const struct sc031gs_regval sc031gs_init_regs[] = {
	{0x0103,0x01},
	{0x0100,0x00},
	{0x36e9,0x80},
	{0x36f9,0x80},
	{0x300f,0x0f},
	{0x3018,0x1f},
	{0x3019,0xff},
	{0x301c,0xb4},
	{0x301f,0xd4},
	{0x3028,0x82},
	{0x3031,0x0a},
	{0x3037,0x20},
	{0x320c,0x04},
	{0x320d,0x70},
	{0x320e,0x08},
	{0x320f,0x40},
	{0x3217,0x00},
	{0x3218,0x00},
	{0x3220,0x10},
	{0x3223,0x48},
	{0x3226,0x74},
	{0x3227,0x07},
	{0x323b,0x00},
	{0x3250,0xf0},
	{0x3251,0x02},
	{0x3252,0x08},
	{0x3253,0x38},
	{0x3254,0x02},
	{0x3255,0x07},
	{0x3304,0x48},
	{0x3305,0x00},
	{0x3306,0x98},
	{0x3309,0x50},
	{0x330a,0x01},
	{0x330b,0x18},
	{0x330c,0x18},
	{0x330f,0x40},
	{0x3310,0x10},
	{0x3314,0x6b},
	{0x3315,0x30},
	{0x3316,0x68},
	{0x3317,0x14},
	{0x3329,0x5c},
	{0x332d,0x5c},
	{0x332f,0x60},
	{0x3335,0x64},
	{0x3344,0x64},
	{0x335b,0x80},
	{0x335f,0x80},
	{0x3366,0x06},
	{0x3385,0x31},
	{0x3387,0x39},
	{0x3389,0x01},
	{0x33b1,0x03},
	{0x33b2,0x06},
	{0x33bd,0xe0},
	{0x33bf,0x10},
	{0x3621,0xa4},
	{0x3622,0x05},
	{0x3624,0x47},
	{0x3630,0x4a},
	{0x3631,0x58},
	{0x3633,0x52},
	{0x3635,0x03},
	{0x3636,0x25},
	{0x3637,0x8a},
	{0x3638,0x0f},
	{0x3639,0x08},
	{0x363a,0x00},
	{0x363b,0x48},
	{0x363c,0x86},
	{0x363e,0xf8},
	{0x3640,0x00},
	{0x3641,0x02},
	{0x36ea,0x36},
	{0x36eb,0x0e},
	{0x36ec,0x1e},
	{0x36ed,0x00},
	{0x36fa,0x36},
	{0x36fb,0x10},
	{0x36fc,0x00},
	{0x36fd,0x00},
	{0x3908,0x91},
	{0x391b,0x81},
	{0x3d08,0x01},
	{0x3e01,0x18},
	{0x3e02,0xf0},
	{0x3e03,0x2b},
	{0x3e06,0x0c},
	{0x3f04,0x04},
	{0x3f05,0x50},
	{0x4500,0x59},
	{0x4501,0xc4},
	{0x4800,0x64},
	{0x4809,0x01},
	{0x4810,0x00},
	{0x4811,0x01},
	{0x4837,0x38},
	{0x5011,0x00},
	{0x5988,0x02},
	{0x598e,0x04},
	{0x598f,0x30},
	{0x36e9,0x03},
	{0x36f9,0x03},
	{0x0100,0x01},
	{0x4418,0x0a},
	{0x363d,0x10},
	{0x4419,0x80},
};


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CAMERA_ADDR 0x30<<1    // 7-bit I2C address
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c4;

/* USER CODE BEGIN PV */
bno055_t bno;
error_bno err;

// Variable pour choisir la fréquence d'acquisition (Hz)
const uint32_t BNO_SAMPLE_RATE = 150;

// Caméra
bool PIXCLK_TRIG;
bool VSYNC_TRIG;
bool HREF_TRIG;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_DMA_Init(void);
static void MX_GPIO_Init(void);
static void MX_DCMI_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C4_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef SC035HGS_WriteReg(uint16_t reg, uint8_t value);
HAL_StatusTypeDef SC035HGS_ReadReg8(uint16_t reg, uint16_t *value);
HAL_StatusTypeDef SC035HGS_ReadReg16(uint16_t reg, uint16_t *value);
bool SC035HGS_VerifyReg8(uint16_t reg, uint16_t expected_value);
bool SC035HGS_VerifyReg16(uint16_t reg, uint16_t expected_value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_DMA_Init();
  MX_GPIO_Init();
  MX_DCMI_Init();
  MX_I2C1_Init();
  MX_I2C4_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("\r\n============= Camera Setup Start =============\r\n\r\n");

  printf("Reseting camera.\r\n");
  HAL_GPIO_WritePin(CAM_RST_GPIO_Port, CAM_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(CAM_RST_GPIO_Port, CAM_RST_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CAM_PWDN_GPIO_Port, CAM_PWDN_Pin, GPIO_PIN_SET);

  HAL_Delay(50);

  // 1. Reset the sensor
  SC035HGS_WriteReg(0x0103, 0x01); // Soft reset
  HAL_Delay(10);
  SC035HGS_WriteReg(0x0103, 0x00); // Exit reset
  HAL_Delay(10);


  // Read high byte of Chip ID (register 0x300A	)
  uint16_t chipID, Rst_pon, slp_reg, pad_out_low, pad_out_high;
  uint16_t DVP_FSYNC_WIDTH, DVP_POL_CTRL, PAD_DRIVER_CAP, PCLK_DLY, MIPI_clock;
  uint16_t frameLength, lineLength;


  SC035HGS_ReadReg16(0x300A, &chipID);
  printf("Chip ID: 0x%04X\r\n", chipID);

  SC035HGS_ReadReg8(SC035_REG_RST, &Rst_pon);
  printf("Soft reset : 0x%02X\r\n", Rst_pon);

  SC035HGS_ReadReg8(SC035_REG_SLP, &slp_reg);
  printf("Sleep mode : 0x%02X\r\n", slp_reg);

  SC035HGS_ReadReg8(SC035_REG_PAD_MSB, &pad_out_low);
  printf("Pad output low : 0x%02X\r\n", pad_out_low);

  SC035HGS_ReadReg8(SC035_REG_PAD_LSB, &pad_out_high);
  printf("Pad output high : 0x%02X\r\n", pad_out_high);

  SC035HGS_ReadReg8(SC035_REG_DVP_FSYNC_WIDTH, &DVP_FSYNC_WIDTH);
  printf("DVP_FSYNC_WIDTH : 0x%02X\r\n", DVP_FSYNC_WIDTH);

  SC035HGS_ReadReg8(SC035_REG_DVP_POL_CTRL, &DVP_POL_CTRL);
  printf("DVP_POL_CTRL : 0x%02X\r\n", DVP_POL_CTRL);

  SC035HGS_ReadReg8(SC035_REG_PAD_DRIVER_CAP, &PAD_DRIVER_CAP);
  printf("PAD_DRIVER_CAP : 0x%02X\r\n", PAD_DRIVER_CAP);

  SC035HGS_ReadReg8(SC035_REG_PCLK_DLY, &PCLK_DLY);
  printf("PCLK_DLY : 0x%02X\r\n", PCLK_DLY);

  SC035HGS_ReadReg16(SC035_REG_FRAME_LENGTH_MSB, &frameLength);
  printf("Frame length : 0x%04X\r\n", frameLength);

  SC035HGS_ReadReg16(SC035_REG_LINE_LENGTH_MSB, &lineLength);
  printf("Line length : 0x%04X\r\n", lineLength);

  SC035HGS_ReadReg16(0x303f, &MIPI_clock);
  printf("MIPI clock : 0x%02X\r\n", MIPI_clock);

  printf("\r\nWriting to register...\r\n");

  // Ecriture des registes, manuellement ou à l'aide des données fournies par Sinoseen

  // I. Manuellement
  // 2. Select DVP mode
  //SC035HGS_WriteReg(SC035_REG_PAD_MSB, 0xF); // Pad control (high)
  //SC035HGS_WriteReg(SC035_REG_PAD_LSB, 0xFF); // Pad control (low)

  // 3. Set line length (e.g., 800 pixel clocks)
  //SC035HGS_WriteReg(SC035_REG_LINE_LENGTH_MSB, 0x03); // Line length
  //SC035HGS_WriteReg(SC035_REG_LINE_LENGTH_LSB, 0x20); // Line length

  // 4. Set frame length (e.g., 500 lines)
  //SC035HGS_WriteReg(SC035_REG_FRAME_LENGTH_MSB, 0x01); // Frame length
  //SC035HGS_WriteReg(SC035_REG_FRAME_LENGTH_LSB, 0xF4); // Frame length

  // 5. Set exposure time (e.g., 100 rows)
  //SC035HGS_WriteReg(0x3e01, 0x64); // Exposure time

  // 6. Set analog gain (e.g., 1x gain)
  //SC035HGS_WriteReg(0x3e08, 0x00); // Coarse gain
  //SC035HGS_WriteReg(0x3e09, 0x10); // Fine gain

  // 7. Set digital gain (e.g., 1x gain)
  //SC035HGS_WriteReg(0x3e06, 0x00); // Digital gain
  //SC035HGS_WriteReg(0x3e07, 0x00); // Fine digital gain

  // 8. Configure DVP timing and polarity
  //SC035HGS_WriteReg(SC035_REG_DVP_FSYNC_WIDTH, 0x01); // FSYNC width
  //SC035HGS_WriteReg(SC035_REG_DVP_POL_CTRL, 0x01); // Polarity control
  //SC035HGS_WriteReg(SC035_REG_PAD_DRIVER_CAP, 0xFF); // Pad driver capability
  //SC035HGS_WriteReg(SC035_REG_PCLK_DLY, 0x00); // PCLK delay

  //SC035HGS_WriteReg(0x303f, 0x80); // SMipi clock

  // 9. Start streaming
  //SC035HGS_WriteReg(SC035_REG_SLP, 0x01); // Start streaming



  // II. A l'aide des registres de Sinoseen
  printf("\r\nWriting initialization registers (cf. Sinoseen) \r\n");

  // Calculate the number of elements in the array
  int num_regs = sizeof(sc031gs_init_regs) / sizeof(sc031gs_init_regs[0]);

  // Loop over all register and value pairs
  for (int i = 0; i < num_regs; i++) {
          uint16_t reg = sc031gs_init_regs[i].addr;
          uint8_t value = sc031gs_init_regs[i].val;

          // Write the value to the register
          SC035HGS_WriteReg(reg, value);

          printf("0x%04X : 0x%04X\r\n", reg, value);
  }

  printf("\r\n");


  // Petit code permettant de vérifier si un signal est recu sur les ports de clock
  PIXCLK_TRIG = false;
  VSYNC_TRIG = false;
  HREF_TRIG = false;

  HAL_Delay(500);
  printf("PIXCLK : %d - VSYNC : %d - HREF : %d\r\n", PIXCLK_TRIG, VSYNC_TRIG, HREF_TRIG);



  printf("\r\n============= BNO055 Setup =============\r\n");

  // Set BNO adress pin to low
  HAL_GPIO_WritePin(BNO_ADR_GPIO_Port, BNO_ADR_Pin, GPIO_PIN_RESET);

  // Reset BNO
  HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_SET);

  // Create BNO object
  bno = (bno055_t){
  	  .i2c = &hi2c4, .addr = BNO_ADDR, .mode = BNO_MODE_NDOF,
     //.ptr = &bno,
  };
  HAL_Delay(500);

  // Initialize the BNO055
  if ((err = bno055_init(&bno)) != BNO_OK) {
  	printf("[!] BNO055 init failed, trying other address..\r\n");
  	bno = (bno055_t){
  		.i2c = &hi2c4, .addr = BNO_ADDR_ALT, .mode = BNO_MODE_NDOF,
  		//.ptr = &bno,
		};
  	err = bno055_init(&bno);
  }
  if (err == BNO_OK){
  	printf("[+] BNO055 init success\r\n");
  	HAL_Delay(100);
  }
  else {
  	printf("[!] BNO055 init failed\r\n");
  	printf("%s\r\n", bno055_err_str(err));
  	Error_Handler();
  }

  HAL_Delay(100);

  // Set output units
  err = bno055_set_unit(&bno, BNO_TEMP_UNIT_C, BNO_GYR_UNIT_RPS,
  						BNO_ACC_UNITSEL_M_S2, BNO_EUL_UNIT_DEG);
  if (err != BNO_OK) {
    printf("[+] Failed to set units. Err: %d\r\n", err);
  } else {
  	printf("[+] Unit selection success\r\n");
  }

  HAL_Delay(500);
  //bno055_vec3_t acc_lin = {0, 0, 0};
  bno055_vec3_t acc = {0, 0, 0};
  bno055_vec3_t gyr = {0, 0, 0};
  //bno055_euler_t euler = {0, 0, 0};
  uint32_t t = HAL_GetTick();


  // On allume la led verte pour signaler la fin du setup.
  BSP_LED_On(LED_GREEN);


  while (1)
  {
	uint32_t t_next = HAL_GetTick();

 	if (t_next-t > (1.0 / BNO_SAMPLE_RATE) * 1000.0) {
	  t = t_next;
	  err = bno.acc(&bno, &acc);
	  if (err != BNO_OK){
	    printf("Error reading BNO data\r\n");
	  	printf("%s\r\n", bno055_err_str(err));
	  }
	  bno.gyro(&bno, &gyr);

	  //bno.linear_acc(&bno, &acc_lin);
 	  //bno.euler(&bno, &euler);

	  printf("%lu;%f;%f;%f;%f;%f;%f\r\n", t, acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z);
	 }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 25;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI48, RCC_MCODIV_2);
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00C0EAFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x10C0ECFF;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAM_PWDN_GPIO_Port, CAM_PWDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BNO_RST_Pin|BNO_ADR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAM_RST_GPIO_Port, CAM_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PIXCLK_Pin HREF_Pin VSYNC_Pin */
  GPIO_InitStruct.Pin = PIXCLK_Pin|HREF_Pin|VSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CAM_PWDN_Pin */
  GPIO_InitStruct.Pin = CAM_PWDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CAM_PWDN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BNO_RST_Pin */
  GPIO_InitStruct.Pin = BNO_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BNO_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BNO_ADR_Pin */
  GPIO_InitStruct.Pin = BNO_ADR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BNO_ADR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : CAM_RST_Pin */
  GPIO_InitStruct.Pin = CAM_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CAM_RST_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(PIXCLK_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(PIXCLK_EXTI_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN){
	switch (GPIO_PIN){
		case PIXCLK_Pin:
			//printf("PixClk interrupt\r\n");
			PIXCLK_TRIG = true;
			break;
		case VSYNC_Pin:
			//printf("VSYNC interrupt\r\n");
			VSYNC_TRIG = true;
			break;
		case HREF_Pin:
			//printf("HREF interrupt\r\n");
			HREF_TRIG = true;
			break;
	}
}

HAL_StatusTypeDef SC035HGS_ReadReg8(uint16_t reg, uint16_t *value) {
	*value = 0;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, CAMERA_ADDR, reg, I2C_MEMADD_SIZE_16BIT, value, 1, 100);
    return status;
}

HAL_StatusTypeDef SC035HGS_ReadReg16(uint16_t reg, uint16_t *value) {
	*value = 0;
    uint8_t data[2];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, CAMERA_ADDR, reg, I2C_MEMADD_SIZE_16BIT, data, 2, 100);
    *value = (data[0] << 8) | data[1]; // Combine MSB and LSB
    return status;
}

HAL_StatusTypeDef SC035HGS_WriteReg(uint16_t reg, uint8_t value) {
    uint8_t data[1];
    data[0] = value & 0xFF;        // LSB
    return HAL_I2C_Mem_Write(&hi2c1, CAMERA_ADDR, reg, I2C_MEMADD_SIZE_16BIT, data, 1, 100);
}

bool SC035HGS_VerifyReg8(uint16_t reg, uint16_t expected_value) {
    uint16_t read_value;
    if (SC035HGS_ReadReg8(reg, &read_value) != HAL_OK) {
        return false; // I²C read error
    }
    if (read_value != expected_value) {
        return false; // Register value mismatch
    }
    return true; // Register value matches
}

bool SC035HGS_VerifyReg16(uint16_t reg, uint16_t expected_value) {
    uint16_t read_value;
    if (SC035HGS_ReadReg16(reg, &read_value) != HAL_OK) {
        return false; // I²C read error
    }
    if (read_value != expected_value) {
        return false; // Register value mismatch
    }
    return true; // Register value matches
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
