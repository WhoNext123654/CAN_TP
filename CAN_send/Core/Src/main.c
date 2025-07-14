/* USER CODE BEGIN Header */
/**
  ******************************************************************************
	Author: Phạm Văn Nam
	Started date: 17-06-2025
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- Cấu hình các frame trong CAN-TP ---
#define PCI_TYPE_SF 0x00
#define PCI_TYPE_FF 0x10
#define PCI_TYPE_CF 0x20
#define PCI_TYPE_FC 0x30
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2S_HandleTypeDef hi2s3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

static uint32_t g_wait_fc_start_tick = 0;

// --- CAN-TP Sender State Machine ---
typedef enum {
	TP_SENDER_IDLE,
	TP_SENDER_WAIT_FC,
	TP_SENDER_SEND
} TP_Sender_State_t;

volatile TP_Sender_State_t sender_state = TP_SENDER_IDLE;

// --- Cấu hình CAN ---
static uint32_t g_tx_mailbox;
static CAN_TxHeaderTypeDef g_tx_header = {
    .DLC = 8,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .StdId = 0x123
};
#define CAN_TP_FC_RECEIVE_ID 0x125 // ID để nhận Flow Control

// --- Buffer và cờ báo từ UART ---
uint8_t g_uart_rx_byte = 0;
uint8_t g_uart_data_buffer[128];
uint16_t g_uart_data_len = 0;
volatile uint8_t g_uart_data_ready = 0;

// --- Quản lý dữ liệu và trạng thái phiên gửi ---
static uint8_t g_tp_data_to_send[128];
static uint16_t g_tp_total_len = 0;
static uint16_t g_tp_sent_index = 0;
static uint8_t  g_tp_seq_num = 0;

// --- Thông số Flow Control nhận được ---
static uint8_t  g_fc_block_size = 0;
static uint8_t  g_fc_st_min = 0;
static uint32_t g_last_cf_sent_tick = 0;
static uint8_t  g_cf_sent_in_block = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2S3_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// --- Các hàm helper để gửi frame ---
void print_frame(const char* label, uint8_t* frame, uint8_t len) {
    char msg[100];
    int offset = snprintf(msg, sizeof(msg), "%s (len=%u): ", label, len);
    for (uint8_t i = 0; i < len; i++) {
        offset += snprintf(msg + offset, sizeof(msg) - offset, "%02X ", frame[i]);
    }
    strncat(msg, "\r\n", sizeof(msg) - strlen(msg) - 1);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

void can_tp_send_frame(uint8_t* data, uint8_t dlc) {
    g_tx_header.DLC = dlc;
    // Luôn kiểm tra kết quả trả v�? khi gửi CAN
    if (HAL_CAN_AddTxMessage(&hcan1, &g_tx_header, data, &g_tx_mailbox) != HAL_OK) {

    }
}

// --- Hàm quản lý State Machine chính ---
void can_tp_manager(void) {
    switch (sender_state) {
        case TP_SENDER_IDLE:
            // Nếu có dữ liệu mới từ UART và đang ở trạng thái nghỉ
            if (g_uart_data_ready) {
                // 1. Xóa c�? và copy dữ liệu để xử lý
                g_uart_data_ready = 0;
                memcpy(g_tp_data_to_send, g_uart_data_buffer, g_uart_data_len);
                g_tp_total_len = g_uart_data_len;

                // 2. Quyết định gửi SF hay FF
                if (g_tp_total_len <= 7) {
                    // --- Gửi Single Frame (SF) ---
                    uint8_t frame[8] = {0};
                    frame[0] = PCI_TYPE_SF | (g_tp_total_len & 0x0F);
                    memcpy(&frame[1], g_tp_data_to_send, g_tp_total_len);
                    print_frame("TX SF", frame, g_tp_total_len + 1);
                    can_tp_send_frame(frame, g_tp_total_len + 1);
                    // Gửi xong, vẫn ở trạng thái IDLE
                } else {
                    // --- Gửi First Frame (FF) ---
                    uint8_t frame[8] = {0};
                    frame[0] = PCI_TYPE_FF | ((g_tp_total_len >> 8) & 0x0F);
                    frame[1] = g_tp_total_len & 0xFF;
                    memcpy(&frame[2], g_tp_data_to_send, 6);
                    print_frame("TX FF", frame, 8);
                    can_tp_send_frame(frame, 8);

                    // 3. Cập nhật state và các biến cho phiên gửi
                    g_tp_sent_index = 6;
                    g_tp_seq_num = 1;
                    sender_state = TP_SENDER_WAIT_FC;
                    g_wait_fc_start_tick = HAL_GetTick(); // Bắt đầu đếm th�?i gian ch�? FC
                }
                g_uart_data_len = 0; // chỉ reset sau khi copy
            }
            break;

        case TP_SENDER_WAIT_FC:
            // Kiểm tra timeout cho FC
            if (HAL_GetTick() - g_wait_fc_start_tick >= 5000) {
                HAL_UART_Transmit(&huart2, (uint8_t*)"Timeout: Don't receive FC frame. \r\n", 36, 100);
                sender_state = TP_SENDER_IDLE; // Quay lại trạng thái nghỉ
            }
            break;

        case TP_SENDER_SEND:
        {
            // 1. Kiểm tra Separation Time (STmin)
            uint32_t st_min_ms = g_fc_st_min;
            if (st_min_ms > 0 && (HAL_GetTick() - g_last_cf_sent_tick < st_min_ms)) {
                return;
            }

            // 2. Kiểm tra Block Size (BS)
            if (g_fc_block_size > 0 && g_cf_sent_in_block >= g_fc_block_size) {
                // �?ã gửi hết block, phải ch�? FC tiếp theo
                sender_state = TP_SENDER_WAIT_FC;
                return;
            }

            // 3. Chuẩn bị và gửi CF
            uint16_t remaining = g_tp_total_len - g_tp_sent_index;
            if (remaining > 0) {
                uint8_t chunk_size = (remaining < 7) ? remaining : 7;
                uint8_t frame[8] = {0};
                frame[0] = PCI_TYPE_CF | (g_tp_seq_num & 0x0F);
                memcpy(&frame[1], &g_tp_data_to_send[g_tp_sent_index], chunk_size);

                char label[16];
                snprintf(label, sizeof(label), "TX CF [%d]", g_tp_seq_num);
                print_frame(label, frame, chunk_size + 1);
                can_tp_send_frame(frame, chunk_size + 1);

                // 4. Cập nhật các biến trạng thái
                g_last_cf_sent_tick = HAL_GetTick();
                g_tp_sent_index += chunk_size;
                g_tp_seq_num = (g_tp_seq_num + 1) % 16;
                g_cf_sent_in_block++;
            }

            // 5. Kiểm tra nếu đã gửi xong toàn bộ tin nhắn
            if (g_tp_sent_index >= g_tp_total_len) {
                HAL_UART_Transmit(&huart2, (uint8_t*)"TP Send Complete\r\n", 18, 100);
                sender_state = TP_SENDER_IDLE; // Quay lại trạng thái nghỉ
            }
            break;
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2S3_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  CAN_FilterTypeDef filter_config = {
         .FilterBank = 0,
         .FilterMode = CAN_FILTERMODE_IDMASK,
         .FilterScale = CAN_FILTERSCALE_32BIT,
		 .FilterIdHigh = 0x0000,
		 .FilterIdLow = 0x0000,
		 .FilterMaskIdHigh = 0x0000,
		 .FilterMaskIdLow = 0x0000,
         .FilterFIFOAssignment = CAN_FILTER_FIFO0,
         .FilterActivation = ENABLE,
         .SlaveStartFilterBank = 14
     };

     if (HAL_CAN_ConfigFilter(&hcan1, &filter_config) != HAL_OK) {
         Error_Handler();
     }

     // 4. Bật CAN và kích hoạt ngắt FIFO0
     if (HAL_CAN_Start(&hcan1) != HAL_OK) {
         Error_Handler();
     }

     if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
         Error_Handler();
     }

     // 5. Bắt đầu nhận UART3 từng byte (gửi lệnh điều khiển)
     HAL_UART_Receive_IT(&huart2, &g_uart_rx_byte, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Hàm quản lý state machine sẽ được g�?i liên tục
    can_tp_manager();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */


  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           PD4 */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
//        HAL_UART_Transmit(&huart2, &g_uart_rx_byte, 1, 10);  // In từng byte ra USART2 để xem luồng nhận

        // Chỉ xử lý nếu state machine đang ở trạng thái nghỉ
        if (sender_state == TP_SENDER_IDLE) {
            if (g_uart_rx_byte != '\n' && g_uart_rx_byte != '\r' && g_uart_data_len < sizeof(g_uart_data_buffer) - 1)
            {
				g_uart_data_buffer[g_uart_data_len++] = g_uart_rx_byte;
            } else {
                if (g_uart_data_len > 0) {
                    g_uart_data_ready = 1; // Báo cho main loop biết có dữ liệu
                }
            }
        }

        // Luôn kích hoạt lại ngắt để nhận byte tiếp theo
        HAL_UART_Receive_IT(&huart2, &g_uart_rx_byte, 1);
    }
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return;
    }

    // Chỉ xử lý nếu đang ở trạng thái ch�? Flow Control và ID là chính xác
    if (sender_state == TP_SENDER_WAIT_FC && rx_header.StdId == CAN_TP_FC_RECEIVE_ID) {
        uint8_t pci_type = (rx_data[0] & 0xF0);
        uint8_t flow_status = (rx_data[0] & 0x0F);

     if (pci_type == PCI_TYPE_FC) {
    	 // In nội dung frame ra UART2
    	 print_frame("RX FC", rx_data, rx_header.DLC);

    	if (flow_status == 0x00) { // Continue To Send (CTS)
    	   g_fc_block_size = rx_data[1];
    	   g_fc_st_min = rx_data[2];

			 // Chuyển đổi nếu STmin là micro giây (0xF1 - 0xF9)
			 if (g_fc_st_min >= 0xF1 && g_fc_st_min <= 0xF9) {
			   g_fc_st_min = 1; // Giả định tối thiểu 1ms
			 }

    	 g_cf_sent_in_block = 0;
    	 sender_state = TP_SENDER_SEND;

    	} else if (flow_status == 0x01) { // Wait
    	  // Có thể xử lý thêm nếu muốn
    	  HAL_UART_Transmit(&huart2, (uint8_t*)"FC = WAIT\r\n", 11, 100);
    	} else if (flow_status == 0x02) { // Overflow/Abort
    	  HAL_UART_Transmit(&huart2, (uint8_t*)"FC = OVERFLOW/ABORT\r\n", 22, 100);
    	  sender_state = TP_SENDER_IDLE;
    	} else {
    	  HAL_UART_Transmit(&huart2, (uint8_t*)"FC = UNKNOWN\r\n", 15, 100);
        }
    }
}
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

#ifdef  USE_FULL_ASSERT
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
