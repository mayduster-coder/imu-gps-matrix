/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  *
  *  Uses modules:
  *    IMU.c           MPU6050_Read_All(), imu_data, imu_zero,
  *                    MAX7219_DrawPattern(), ARROW_N/NE/.../SMILEY_FACE
  *    GPS.c           Parse_GPS_Sentence(), g_gps_data
  *    ETH_Telemetry.c ETH_Telemetry_Init(), ETH_Telemetry_Send(), ETH_Retry_Task()
  *
  *  Tasks (each hardware bus has exactly ONE owner):
  *
  *   imuTask    (10 ms)  owns I2C1: MPU6050_Read_All -> tilt from HOME -> [imu_mailbox]
  *   matrixTask (50 ms)  owns SPI2: [imu_mailbox] -> arrow -> MAX7219_DrawPattern
  *   gpsTask             USART2 DMA ISR -> [gps_chunk_queue] -> Parse_GPS_Sentence -> [gps_mailbox]
  *   defaultTask         lwIP; telemetry send (1 s) + retry (200 ms) run in the lwIP thread
  *   ethRxTask           [eth_rx_queue] -> prints every received UDP packet (hex + bits)
  *
  *  Other code that wants the I2C or the matrix (e.g. ETH commands) does NOT touch
  *  the hardware - it sends a request: App_RequestImuZero(), App_ShowPattern().
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include "queue.h"
#include "semphr.h"
#include "usart.h"
#include "i2c.h"
#include "lwip/tcpip.h"
#include "lwip/netif.h"
#include "lwip/ip_addr.h"
#include "IMU.h"
#include "GPS.h"
#include "ETH_Telemetry.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* ================= settings you may want to change ================= */

/* Arrow: shown when the device is tilted this much from HOME */
#define IMU_ARROW_DEG          5.0f   /* tilt >= 5 deg -> arrow */
#define IMU_HOME_DEG           4.0f   /* back under 4 deg -> smiley (1 deg hysteresis) */
#define IMU_FILTER             0.25f  /* low-pass: 0.25 = smooth, 1.0 = no filter */
#define IMU_HOME_SAMPLES       50U    /* 50 samples averaged when saving HOME */
#define IMU_SETTLE_MS          300U   /* wait before saving HOME (hands off) */

/* If the arrow points the wrong way, flip these (0/1) */
#define IMU_SWAP_XY            0
#define IMU_INVERT_X           0
#define IMU_INVERT_Y           0

/* periods */
#define IMU_PERIOD_MS          10U //Period sample
#define MATRIX_PERIOD_MS       50U // Period sample for fluent switch
#define NET_PERIOD_MS          200U   /* ETH_Retry_Task */
#define TELEMETRY_PERIOD_MS    1000U  /* ETH_Telemetry_Send */
#define IMU_PRINT_MS           500U
#define GPS_SILENCE_MS         3000U // SILENCE GPS to not fill the terminal
#define GPS_PRINT_RAW_NMEA     0      /* 1 = print every raw NMEA line */
#define PATTERN_OVERRIDE_MS    3000U  /* ETH command 3 shows its pattern this long */

/* Ethernet RX printing */
#define ETH_RX_MAX_BYTES       64U // max byte of the packet
#define ETH_RX_PRINT_MAX       32U// number of packet that is printed

/* GPS */
#define GPS_DMA_BUF_SIZE       128U   // multiple of 32
#define GPS_CHUNK_QUEUE_LEN    4U // if TASK GPS is busy  there is 4 parts waiting
#define NMEA_LINE_MAX          96U // NMEA length

/* Nucleo-144 LEDs */
#define LED_MATRIX_PIN         GPIO_PIN_0    /* green: matrixTask alive */
#define LED_IMU_PIN            GPIO_PIN_7    /* blue:  imuTask alive    */
#define LED_ERROR_PIN          GPIO_PIN_14   /* red:   I2C error        */

#define RAD2DEG                57.29577951f
#define DEG2RAD                0.01745329f

/* ================= data passed between tasks ================= */

typedef struct {
  float    ax, ay, az;      /* accel in g (from imu_data) */
  float    gx, gy, gz;      /* gyro in deg/s */
  float    du, dv;          /* distance from HOME on the two tilt axes (g) */
  uint8_t  axis_u, axis_v;  /* which axes (0=X 1=Y 2=Z) */
  uint8_t  ok;              /* 0 = I2C error */
  uint8_t  has_home;
  uint32_t tick;
} ImuSample_t;

typedef struct {
  uint16_t len;
  uint8_t  data[GPS_DMA_BUF_SIZE];
} GpsChunk_t;

typedef struct {
  uint32_t ip;
  uint16_t port;
  uint16_t len;
  uint16_t total_len;
  uint8_t  data[ETH_RX_MAX_BYTES];
} EthRxMsg_t;

typedef enum {
  DIR_HOME = 0,
  DIR_N, DIR_S, DIR_E, DIR_W,
  DIR_NE, DIR_NW, DIR_SE, DIR_SW,
  DIR_WAIT,
  DIR_ERROR,
  DIR_COUNT
} ArrowDir_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* from modules (declared here too, in case the headers don't export them) */
extern MPU6050_t      imu_data;
extern IMU_ZeroState  imu_zero;
extern GPS_GGA_Data   g_gps_data;
extern const uint8_t  SMILEY_FACE[8];
extern const uint8_t  ARROW_N[8], ARROW_S[8], ARROW_E[8], ARROW_W[8];
extern const uint8_t  ARROW_NE[8], ARROW_NW[8], ARROW_SE[8], ARROW_SW[8];

extern struct netif gnetif;              /* lwip.c */

static QueueHandle_t     imu_mailbox;     /* length 1: latest IMU sample   */
static QueueHandle_t     gps_mailbox;     /* length 1: latest GPS_GGA_Data */
static QueueHandle_t     gps_chunk_queue; /* raw bytes from the UART ISR   */
static QueueHandle_t     eth_rx_queue;    /* UDP packets -> ethRxTask      */
static SemaphoreHandle_t print_mutex;

static uint8_t gps_dma_buf[GPS_DMA_BUF_SIZE] __attribute__((aligned(32)));

static volatile ArrowDir_t current_dir     = DIR_WAIT; // can change during running state
static volatile uint8_t    imu_zero_req    = 0;     /* set by App_RequestImuZero() */
static volatile int16_t    override_id     = -1;    /* set by App_ShowPattern()   */
static volatile TickType_t override_until  = 0;
static volatile uint32_t   eth_rx_count    = 0;
static volatile uint32_t   gps_lines       = 0;

/* extra pictures (not in IMU.c) */
static const uint8_t PIC_WAIT[8]  = {0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00};
static const uint8_t PIC_ERROR[8] = {0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81};

static const uint8_t *pics[DIR_COUNT];   /* filled in StartMatrixTask */

static const char *const DIR_NAME[DIR_COUNT] = {
  "HOME", "N", "S", "E", "W", "NE", "NW", "SE", "SW", "WAIT", "I2C-ERR"
};
// create threads
osThreadId gpsTaskHandle;
osThreadId imuTaskHandle;
osThreadId matrixTaskHandle;
osThreadId ethRxTaskHandle;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void StartGpsTask(void const * argument);
void StartImuTask(void const * argument);
void StartMatrixTask(void const * argument);
void StartEthRxTask(void const * argument);


void MPU6050_Read_All(void);
void MAX7219_DrawPattern(const uint8_t pattern[8]);
void Parse_GPS_Sentence(char *sentence);

/* public helpers - used by ETH_Telemetry.c */
void App_Log(const char *fmt, ...);
void App_EthRxPrint(const uint8_t *data, uint16_t len, const ip_addr_t *addr, u16_t port);
void App_RequestImuZero(void);
void App_ShowPattern(uint8_t pattern_id);

static void log_lock(void);
static void log_unlock(void);
static void log_raw(const char *fmt, ...);
static const char *f2s(char *buf, size_t n, double v, uint8_t decimals);
static void ip_to_str(uint32_t ip, char *out, size_t size);

static void gps_start_dma(void);
static void gps_print(const GPS_GGA_Data *g);

static uint8_t imu_read(float a[3]);
static uint8_t imu_save_home(void);
static void imu_pick_axes(uint8_t *u, uint8_t *v);
static uint8_t button_pressed(void);
static void leds_init(void);
static ArrowDir_t imu_to_dir(const ImuSample_t *s, ArrowDir_t prev);

static void eth_init_cb(void *ctx);
static void eth_send_cb(void *ctx);
static void eth_retry_cb(void *ctx);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  print_mutex = xSemaphoreCreateMutex();
  configASSERT(print_mutex != NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  imu_mailbox     = xQueueCreate(1, sizeof(ImuSample_t));
  gps_mailbox     = xQueueCreate(1, sizeof(GPS_GGA_Data));
  gps_chunk_queue = xQueueCreate(GPS_CHUNK_QUEUE_LEN, sizeof(GpsChunk_t));
  eth_rx_queue    = xQueueCreate(4, sizeof(EthRxMsg_t));
  configASSERT(imu_mailbox != NULL);
  configASSERT(gps_mailbox != NULL);
  configASSERT(gps_chunk_queue != NULL);
  configASSERT(eth_rx_queue != NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(gpsTask, StartGpsTask, osPriorityHigh, 0, 768);
  gpsTaskHandle = osThreadCreate(osThread(gpsTask), NULL);

  osThreadDef(imuTask, StartImuTask, osPriorityAboveNormal, 0, 512);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

  osThreadDef(matrixTask, StartMatrixTask, osPriorityAboveNormal, 0, 512);
  matrixTaskHandle = osThreadCreate(osThread(matrixTask), NULL);

  osThreadDef(ethRxTask, StartEthRxTask, osPriorityBelowNormal, 0, 512);
  ethRxTaskHandle = osThreadCreate(osThread(ethRxTask), NULL);

  printf("[RTOS] tasks: default=%s gps=%s imu=%s matrix=%s ethRx=%s, heap left %u\r\n",
         defaultTaskHandle ? "ok" : "FAIL",
         gpsTaskHandle     ? "ok" : "FAIL",
         imuTaskHandle     ? "ok" : "FAIL",
         matrixTaskHandle  ? "ok" : "FAIL",
         ethRxTaskHandle   ? "ok" : "FAIL",
         (unsigned)xPortGetFreeHeapSize());
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Network: lwIP init, ETH_Telemetry module, status.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;

  /* ETH_Telemetry_Init() uses the lwIP raw API -> run it inside the lwIP thread */
  tcpip_callback(eth_init_cb, NULL);
  App_Log("[NET] lwIP up, ETH_Telemetry listening on UDP port 5000\r\n");

  uint8_t    ip_printed = 0;
  uint32_t   loops = 0;
  TickType_t last_wake = xTaskGetTickCount();

  for(;;)
  {
    uint32_t my_ip = ip4_addr_get_u32(netif_ip4_addr(&gnetif));
    if (!ip_printed && netif_is_link_up(&gnetif) && my_ip != 0U)
    {
      char ip_str[16];
      ip_to_str(my_ip, ip_str, sizeof ip_str);
      App_Log("[NET] cable connected, board IP = %s\r\n", ip_str);
      ip_printed = 1;
    }

    tcpip_callback(eth_retry_cb, NULL);                 /* ETH_Retry_Task every 200 ms */

    if (++loops >= TELEMETRY_PERIOD_MS / NET_PERIOD_MS)
    {
      loops = 0;
      tcpip_callback(eth_send_cb, NULL);                /* ETH_Telemetry_Send every 1 s */
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(NET_PERIOD_MS));
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* =========================================================================
 *  PRINTING (thread safe) - App_Log can be used from any task, also ETH_Telemetry.c
 * ========================================================================= */

static void log_lock(void)// lock terminal so that one task can print
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    xSemaphoreTake(print_mutex, portMAX_DELAY);
}

static void log_unlock(void)// unlock terminal
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    xSemaphoreGive(print_mutex);
}

static void log_vraw(const char *fmt, va_list ap)// build text and transmit
{
  char buf[256];
  int n = vsnprintf(buf, sizeof buf, fmt, ap);
  if (n <= 0) return;
  if (n > (int)sizeof buf - 1) n = (int)sizeof buf - 1;
  HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)n, HAL_MAX_DELAY);
}

static void log_raw(const char *fmt, ...) //a changeable argument number
{
  va_list ap;
  va_start(ap, fmt);
  log_vraw(fmt, ap);
  va_end(ap);
}

void App_Log(const char *fmt, ...)
{
  va_list ap;
  log_lock();
  va_start(ap, fmt);
  log_vraw(fmt, ap);
  va_end(ap);
  log_unlock();
}

// float to text
static const char *f2s(char *buf, size_t n, double v, uint8_t decimals)
{
  static const int32_t P[] = {1, 10, 100, 1000, 10000, 100000, 1000000};
  if (decimals > 6U) decimals = 6U;
  int neg = (v < 0.0);
  if (neg) v = -v;
  int32_t ip = (int32_t)v;
  int32_t fp = (int32_t)((v - (double)ip) * P[decimals] + 0.5);
  if (fp >= P[decimals]) { ip++; fp -= P[decimals]; }
  if (decimals > 0U)
    snprintf(buf, n, "%s%ld.%0*ld", neg ? "-" : "", (long)ip, (int)decimals, (long)fp);
  else
    snprintf(buf, n, "%s%ld", neg ? "-" : "", (long)ip);
  return buf;
}

static void ip_to_str(uint32_t ip, char *out, size_t size)
{
  snprintf(out, size, "%u.%u.%u.%u",
           (unsigned)(ip & 0xFFU), (unsigned)((ip >> 8) & 0xFFU),
           (unsigned)((ip >> 16) & 0xFFU), (unsigned)((ip >> 24) & 0xFFU));
}

/* =========================================================================
 *  REQUESTS from other modules (ETH commands) - they never touch I2C/SPI directly
 * ========================================================================= */
// One task accesses the I2C, rather than two simultaneously.
void App_RequestImuZero(void)            /* ETH command 1 */
{
  imu_zero_req = 1;
}

void App_ShowPattern(uint8_t pattern_id) /* ETH command 3: 0 = smiley, 1..8 = arrows */
{
  override_id    = (int16_t)pattern_id;
  override_until = xTaskGetTickCount() + pdMS_TO_TICKS(PATTERN_OVERRIDE_MS);
}

/* =========================================================================
 *  ETHERNET
 * ========================================================================= */

/* called from udp_receive_callback in ETH_Telemetry.c (lwIP thread):
 * copy the packet to the queue, ethRxTask prints it */
//print packet
void App_EthRxPrint(const uint8_t *data, uint16_t len, const ip_addr_t *addr, u16_t port)
{
  EthRxMsg_t msg;
  msg.ip        = ip4_addr_get_u32(ip_2_ip4(addr));
  msg.port      = port;
  msg.total_len = len;
  msg.len       = (len < ETH_RX_MAX_BYTES) ? len : (uint16_t)ETH_RX_MAX_BYTES;
  memcpy(msg.data, data, msg.len);
  xQueueSend(eth_rx_queue, &msg, 0);              /* full -> drop, never block lwIP */
}

/* these 3 run inside the lwIP thread (tcpip_callback) -> raw API is safe */
static void eth_init_cb(void *ctx)  { (void)ctx; ETH_Telemetry_Init(); }
static void eth_retry_cb(void *ctx) { (void)ctx; ETH_Retry_Task(); }
static void eth_send_cb(void *ctx)
{
  (void)ctx;
  GPS_GGA_Data gps;
  memset(&gps, 0, sizeof gps);
  xQueuePeek(gps_mailbox, &gps, 0);  //Receive data and not letting it out
  ETH_Telemetry_Send(&gps);
}
//Eth task
void StartEthRxTask(void const * argument)
{
  (void)argument;
  EthRxMsg_t msg;
  char ip_str[16];

  for(;;)// Receive data from the cp to mcu
  {
    xQueueReceive(eth_rx_queue, &msg, portMAX_DELAY);
    eth_rx_count++;
    ip_to_str(msg.ip, ip_str, sizeof ip_str);

    log_lock();
    log_raw("\r\n[ETH RX #%lu] from %s:%u, %u bytes\r\n",
            (unsigned long)eth_rx_count, ip_str, msg.port, msg.total_len);
    log_raw("  idx  hex   bits      char\r\n");

    uint16_t n = (msg.len < ETH_RX_PRINT_MAX) ? msg.len : (uint16_t)ETH_RX_PRINT_MAX;
    for (uint16_t i = 0; i < n; i++)
    {
      uint8_t b = msg.data[i];
      char bits[9];
      for (int k = 0; k < 8; k++)
        bits[k] = (b & (0x80U >> k)) ? '1' : '0';
      bits[8] = '\0';
      char ch = (b >= 0x20U && b < 0x7FU) ? (char)b : '.';
      log_raw("  [%2u] 0x%02X  %s  %c\r\n", i, b, bits, ch);
    }
    if (msg.total_len > n)
      log_raw("  ... (%u more bytes)\r\n", (unsigned)(msg.total_len - n));
    log_unlock();
  }
}

/* =========================================================================
 *  GPS: USART2 DMA -> gpsTask -> Parse_GPS_Sentence() (GPS.c)
 * ========================================================================= */

static void gps_start_dma(void)// receive gps data
{
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, gps_dma_buf, GPS_DMA_BUF_SIZE);
  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance != USART2)
  {
    return;
  }

  BaseType_t woken = pdFALSE;
  GpsChunk_t chunk;

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  if (SCB->CCR & SCB_CCR_DC_Msk)
  {
    SCB_InvalidateDCache_by_Addr((uint32_t *)gps_dma_buf, (int32_t)GPS_DMA_BUF_SIZE);
  }
#endif

  if (Size > GPS_DMA_BUF_SIZE)
  {
    Size = GPS_DMA_BUF_SIZE;
  }
  chunk.len = Size;
  memcpy(chunk.data, gps_dma_buf, Size); // copy data

  gps_start_dma();

  xQueueSendFromISR(gps_chunk_queue, &chunk, &woken);// send to queue with non blocking mood
  portYIELD_FROM_ISR(woken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    gps_start_dma();
  }
}

static void gps_print(const GPS_GGA_Data *g)
{
  char la[16], lo[16], al[16], hd[16];
  uint32_t utc = (uint32_t)g->utc_time;           /* hhmmss */

  if (g->fix_quality == 0)
  {
    App_Log("[GPS] %02lu:%02lu:%02lu  NO FIX yet, satellites=%u (go near a window / outside)\r\n",
            (unsigned long)(utc / 10000U), (unsigned long)((utc / 100U) % 100U),
            (unsigned long)(utc % 100U), (unsigned)g->num_satellites);
    return;
  }
  App_Log("[GPS] %02lu:%02lu:%02lu  lat=%s lon=%s  alt=%sm  fix=%u sats=%u hdop=%s\r\n",
          (unsigned long)(utc / 10000U), (unsigned long)((utc / 100U) % 100U),
          (unsigned long)(utc % 100U),
          f2s(la, sizeof la, (double)g->latitude, 6),
          f2s(lo, sizeof lo, (double)g->longitude, 6),
          f2s(al, sizeof al, (double)g->altitude, 1),
          (unsigned)g->fix_quality, (unsigned)g->num_satellites,
          f2s(hd, sizeof hd, (double)g->hdop, 1));
}
// GPS task
void StartGpsTask(void const * argument)
{
  (void)argument;

  GpsChunk_t chunk;
  char       line[NMEA_LINE_MAX];
  uint16_t   line_len = 0;

  gps_start_dma();
  App_Log("[GPS] listening on USART2 (PD6 = RX)\r\n");

  for(;;)
  {
    if (xQueueReceive(gps_chunk_queue, &chunk, pdMS_TO_TICKS(GPS_SILENCE_MS)) != pdTRUE) // receive data and print
    {
      App_Log("[GPS] no data for %u s - check GPS TX -> PD6, GND, power, baud rate\r\n",
              (unsigned)(GPS_SILENCE_MS / 1000U));
      continue;
    }

    for (uint16_t i = 0; i < chunk.len; i++)
    {
      char c = (char)chunk.data[i];

      if (c == '$')
      {
        line_len = 0;
      }

      if (c == '\r' || c == '\n')
      {
        if (line_len > 0)
        {
          line[line_len] = '\0';
          gps_lines++;
#if GPS_PRINT_RAW_NMEA
          App_Log("[NMEA] %s\r\n", line);
#endif
          uint8_t is_gga = (line_len >= 6U) && (strncmp(&line[3], "GGA", 3) == 0);

          Parse_GPS_Sentence(line);        /* parser (it changes 'line' - fine) */

          if (is_gga)
          {
            /* g_gps_data is written only by this task -> safe to copy here */
            GPS_GGA_Data copy = g_gps_data;
            xQueueOverwrite(gps_mailbox, &copy);// write to mailbox for other tasks
            gps_print(&copy);              /* GGA comes once a second */
          }
          line_len = 0;
        }
      }
      else if (line_len < (NMEA_LINE_MAX - 1U))
      {
        line[line_len++] = c;
      }
      else
      {
        line_len = 0;
      }
    }
  }
}

/* =========================================================================
 *  IMU: MPU6050_Read_All() (IMU.c), HOME = imu_zero (struct)
 *
 *  Gravity always points down. Tilting the device moves gravity between the
 *  sensor axes. HOME = gravity vector at the start (imu_zero, the same thing
 *  IMU_CalibrateZero() measures). The difference on the two horizontal axes
 *  = how far and which way the device is tilted.
 * ========================================================================= */

/* one reading through function. Returns 1 if the I2C read worked */
static uint8_t imu_read(float a[3])
{
  MPU6050_Read_All();
  if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_NONE)
  {
    return 0;
  }
  a[0] = imu_data.accel_x;
  a[1] = imu_data.accel_y;
  a[2] = imu_data.accel_z;
  return 1;
}

/* same as IMU_CalibrateZero(), but with vTaskDelay instead of HAL_Delay
 * (HAL_Delay would keep the CPU busy and block the other tasks for 0.5 s) */
static uint8_t imu_save_home(void)
{
  float sum[3] = {0, 0, 0};
  uint32_t good = 0;

  vTaskDelay(pdMS_TO_TICKS(IMU_SETTLE_MS));
  for (uint32_t i = 0; i < IMU_HOME_SAMPLES; i++)
  {
    float a[3];
    if (imu_read(a))
    {
      sum[0] += a[0]; sum[1] += a[1]; sum[2] += a[2];
      good++;
    }
    vTaskDelay(pdMS_TO_TICKS(IMU_PERIOD_MS));
  }
  if (good < IMU_HOME_SAMPLES / 2U) return 0;

  imu_zero.ax_offset = sum[0] / (float)good;
  imu_zero.ay_offset = sum[1] / (float)good;
  imu_zero.az_offset = sum[2] / (float)good;
  return 1;
}

/* vertical axis = the one with most gravity; the other two are the tilt axes */
static void imu_pick_axes(uint8_t *u, uint8_t *v)
{
  float a0 = fabsf(imu_zero.ax_offset);
  float a1 = fabsf(imu_zero.ay_offset);
  float a2 = fabsf(imu_zero.az_offset);
  if (a2 >= a0 && a2 >= a1)      { *u = 0; *v = 1; }  /* Z vertical: lying flat   */
  else if (a1 >= a0 && a1 >= a2) { *u = 0; *v = 2; }  /* Y vertical: standing     */
  else                           { *u = 1; *v = 2; }  /* X vertical: on its side  */
}

static uint8_t button_pressed(void)   //new home
{
  static uint8_t last = 0;
  uint8_t now = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) ? 1U : 0U;
  uint8_t pressed = (now && !last) ? 1U : 0U;
  last = now;
  return pressed;
}

static void leds_init(void) // configure 3 leds & 1 button
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef gi = {0};
  gi.Pin   = LED_MATRIX_PIN | LED_IMU_PIN | LED_ERROR_PIN;
  gi.Mode  = GPIO_MODE_OUTPUT_PP;
  gi.Pull  = GPIO_NOPULL;
  gi.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gi);

  gi.Pin  = GPIO_PIN_13;                 /* B1 button */
  gi.Mode = GPIO_MODE_INPUT;
  gi.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &gi);
}
// imu task
void StartImuTask(void const * argument)
{
  (void)argument;
  leds_init();

  ImuSample_t s;
  memset(&s, 0, sizeof s);
  float    filt[3] = {0, 0, 0};
  uint8_t  u = 0, v = 1;
  uint32_t print_cnt = 0, err_run = 0, err_count = 0;

  /* main.c already ran IMU_CalibrateZero() -> use it as HOME if it looks valid */
  float g = fabsf(imu_zero.ax_offset) + fabsf(imu_zero.ay_offset) + fabsf(imu_zero.az_offset);// calculate degrees
  uint8_t need_home = (g < 0.5f) ? 1U : 0U;     /* ~0 = calibration failed */
  if (!need_home)// display arrow
  {
    imu_pick_axes(&u, &v);
    filt[0] = imu_zero.ax_offset; filt[1] = imu_zero.ay_offset; filt[2] = imu_zero.az_offset;
    s.has_home = 1;
    App_Log("[IMU] HOME from IMU_CalibrateZero, tilt axes %c,%c\r\n", "XYZ"[u], "XYZ"[v]);
  }

  TickType_t last_wake = xTaskGetTickCount();

  for(;;)
  {
    /* ---------- save HOME: missing, B1 button, or ETH command 1 ---------- */
    if (need_home || button_pressed() || imu_zero_req)
    {
      imu_zero_req = 0;
      s.has_home = 0;
      s.ok = 1;
      xQueueOverwrite(imu_mailbox, &s);           /* matrix shows "wait" */
      App_Log("[IMU] saving HOME - don't move the device...\r\n");

      if (imu_save_home())
      {
        imu_pick_axes(&u, &v);
        filt[0] = imu_zero.ax_offset; filt[1] = imu_zero.ay_offset; filt[2] = imu_zero.az_offset;
        s.has_home = 1;
        need_home  = 0;
        char x[12], y[12], z[12];
        App_Log("[IMU] HOME saved: x=%sg y=%sg z=%sg, tilt axes %c,%c\r\n",
                f2s(x, sizeof x, imu_zero.ax_offset, 3),
                f2s(y, sizeof y, imu_zero.ay_offset, 3),
                f2s(z, sizeof z, imu_zero.az_offset, 3), "XYZ"[u], "XYZ"[v]);
      }
      else
      {
        s.ok = 0;
        xQueueOverwrite(imu_mailbox, &s);
        App_Log("[IMU] HOME FAILED - no data from MPU6050, retry in 1 s\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
        need_home = 1;
      }
      last_wake = xTaskGetTickCount();
      continue;
    }

    /* ---------- normal reading ---------- */
    float a[3];
    if (imu_read(a))
    {
      const float home[3] = {imu_zero.ax_offset, imu_zero.ay_offset, imu_zero.az_offset};
      for (int k = 0; k < 3; k++)
        filt[k] += (a[k] - filt[k]) * IMU_FILTER;

      s.ax = a[0]; s.ay = a[1]; s.az = a[2];
      s.gx = imu_data.gyro_x; s.gy = imu_data.gyro_y; s.gz = imu_data.gyro_z;
      s.du = filt[u] - home[u];
      s.dv = filt[v] - home[v];
      s.axis_u = u; s.axis_v = v;
      s.ok = 1;
      err_run = 0;
    }
    else
    {
      s.ok = 0;
      err_count++;
      if (++err_run >= 20U)                       /* 0.2 s of errors: reset I2C1 */
      {
        err_run = 0;
        App_Log("[IMU] I2C stuck (error=0x%02lX) -> resetting I2C1\r\n",
                (unsigned long)HAL_I2C_GetError(&hi2c1));
        HAL_I2C_DeInit(&hi2c1);
        MX_I2C1_Init();
        MPU6050_Init();                           /*init: wake + config */
      }
    }
    s.tick = xTaskGetTickCount();
    xQueueOverwrite(imu_mailbox, &s);
    HAL_GPIO_WritePin(GPIOB, LED_ERROR_PIN, s.ok ? GPIO_PIN_RESET : GPIO_PIN_SET);

    /* ---------- print every 0.5 s ---------- */
    if (++print_cnt >= IMU_PRINT_MS / IMU_PERIOD_MS)
    {
      print_cnt = 0;
      HAL_GPIO_TogglePin(GPIOB, LED_IMU_PIN);
      if (s.ok)
      {
        char b[8][12];
        float du = s.du > 1.0f ? 1.0f : (s.du < -1.0f ? -1.0f : s.du);
        float dv = s.dv > 1.0f ? 1.0f : (s.dv < -1.0f ? -1.0f : s.dv);
        App_Log("[IMU] acc x=%s y=%s z=%s g | gyro x=%s y=%s z=%s dps | tilt %c=%s %c=%s deg | %s\r\n",
                f2s(b[0], 12, s.ax, 2), f2s(b[1], 12, s.ay, 2), f2s(b[2], 12, s.az, 2),
                f2s(b[3], 12, s.gx, 1), f2s(b[4], 12, s.gy, 1), f2s(b[5], 12, s.gz, 1),
                "XYZ"[u], f2s(b[6], 12, asinf(du) * RAD2DEG, 1),
                "XYZ"[v], f2s(b[7], 12, asinf(dv) * RAD2DEG, 1),
                DIR_NAME[current_dir]);
      }
      else
      {
        App_Log("[IMU] I2C read ERROR x%lu (error=0x%02lX)\r\n",
                (unsigned long)err_count, (unsigned long)HAL_I2C_GetError(&hi2c1));
      }
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_PERIOD_MS));
  }
}

/* tilt from HOME -> arrow pointing the way BACK to HOME */
static ArrowDir_t imu_to_dir(const ImuSample_t *s, ArrowDir_t prev)
{
  static float th_arrow = 0.0f, th_home = 0.0f;
  if (th_arrow == 0.0f)
  {
    th_arrow = sinf(IMU_ARROW_DEG * DEG2RAD);    /* 5 deg = 0.087 g */
    th_home  = sinf(IMU_HOME_DEG  * DEG2RAD);    /* 4 deg = 0.070 g */
  }

  if (!s->ok)       return DIR_ERROR;
  if (!s->has_home) return DIR_WAIT;

  float x = -s->du;                               /* opposite of the tilt = way back */
  float y = -s->dv;
#if IMU_SWAP_XY
  float t = x; x = y; y = t;
#endif
#if IMU_INVERT_X
  x = -x;
#endif
#if IMU_INVERT_Y
  y = -y;
#endif

  float axv = fabsf(x), ayv = fabsf(y);
  float big = (axv > ayv) ? axv : ayv;

  uint8_t arrow_shown = (prev >= DIR_N && prev <= DIR_SW) ? 1U : 0U;
  if (big < (arrow_shown ? th_home : th_arrow)) return DIR_HOME;

  /* an axis counts if it is at least half of the bigger one -> 8 directions */
  int sx = (axv * 2.0f >= big) ? ((x > 0) ? 1 : -1) : 0;
  int sy = (ayv * 2.0f >= big) ? ((y > 0) ? 1 : -1) : 0;

  if (sy > 0) return (sx > 0) ? DIR_NE : (sx < 0) ? DIR_NW : DIR_N;
  if (sy < 0) return (sx > 0) ? DIR_SE : (sx < 0) ? DIR_SW : DIR_S;
  return (sx > 0) ? DIR_E : DIR_W;
}

/* =========================================================================
 *  MATRIX: MAX7219_DrawPattern() +  patterns from IMU.c
 *  (MAX7219_Init() is called in main.c before the scheduler)
 * ========================================================================= */
void StartMatrixTask(void const * argument)
{
  (void)argument;

  pics[DIR_HOME]  = SMILEY_FACE;
  pics[DIR_N]     = ARROW_N;   pics[DIR_S]  = ARROW_S;
  pics[DIR_E]     = ARROW_E;   pics[DIR_W]  = ARROW_W;
  pics[DIR_NE]    = ARROW_NE;  pics[DIR_NW] = ARROW_NW;
  pics[DIR_SE]    = ARROW_SE;  pics[DIR_SW] = ARROW_SW;
  pics[DIR_WAIT]  = PIC_WAIT;
  pics[DIR_ERROR] = PIC_ERROR;

  /* same order as ETH command 3: 0 smiley, 1 E, 2 NE, 3 N, 4 NW, 5 W, 6 SW, 7 S, 8 SE */
  static const ArrowDir_t cmd_to_dir[9] = {
    DIR_HOME, DIR_E, DIR_NE, DIR_N, DIR_NW, DIR_W, DIR_SW, DIR_S, DIR_SE
  };

  /* self test: all 8 arrows once */
  App_Log("[MTX] self test: all arrows...\r\n");
  for (int i = 1; i <= 8; i++)
  {
    MAX7219_DrawPattern(pics[cmd_to_dir[i]]);
    vTaskDelay(pdMS_TO_TICKS(250));
  }
  App_Log("[MTX] ready\r\n");

  ImuSample_t imu;
  memset(&imu, 0, sizeof imu); // set imu
  ArrowDir_t shown = DIR_COUNT;
  uint32_t   refresh = 0, blink = 0;
  TickType_t last_wake = xTaskGetTickCount();

  for(;;)
  {
    if (++blink >= 10U)
    {
      blink = 0;
      HAL_GPIO_TogglePin(GPIOB, LED_MATRIX_PIN);
    }

    /* ETH command 3: show the requested pattern for a few seconds */
    int16_t ov = override_id; // shape from ETH
    if (ov >= 0 && ov <= 8 && (int32_t)(override_until - xTaskGetTickCount()) > 0)
    {
      ArrowDir_t d = cmd_to_dir[ov]; // current shape
      if (d != shown) // display if it is different then what is already display
      {
        MAX7219_DrawPattern(pics[d]);
        App_Log("[MTX] pattern %d from Ethernet\r\n", ov);
        shown = d;
      }
    }
    else if (xQueuePeek(imu_mailbox, &imu, 0) == pdTRUE) // we display according to imu
    {
      override_id = -1;
      ArrowDir_t dir = imu_to_dir(&imu, shown);
      if (dir != shown || ++refresh >= 20U)
      {
        MAX7219_DrawPattern(pics[dir]);
        if (dir != shown)
          App_Log("[MTX] -> %s\r\n", DIR_NAME[dir]);
        shown = dir;
        current_dir = dir;
        refresh = 0;
      }
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MATRIX_PERIOD_MS));
  }
}

/* =========================================================================
 *  FreeRTOS hooks - used only if enabled in CubeMX
 * ========================================================================= */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  static const char msg[] = "\r\n!!! STACK OVERFLOW in task: ";
  HAL_UART_Transmit(&huart3, (uint8_t *)msg, sizeof msg - 1, HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart3, (uint8_t *)pcTaskName, (uint16_t)strlen(pcTaskName), HAL_MAX_DELAY);
  taskDISABLE_INTERRUPTS();
  for(;;) { }
}

void vApplicationMallocFailedHook(void)
{
  static const char msg[] = "\r\n!!! MALLOC FAILED - increase TOTAL_HEAP_SIZE\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t *)msg, sizeof msg - 1, HAL_MAX_DELAY);
  taskDISABLE_INTERRUPTS();
  for(;;) { }
}

/* USER CODE END Application */

