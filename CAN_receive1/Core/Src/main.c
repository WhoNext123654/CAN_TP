/* USER CODE BEGIN Header */
/**
  ******************************************************************************
	Author: Pham Van Nam
	Date: 20-06-2025
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
// �?ịnh nghĩa các trạng thái cho bộ nhận CAN-TP
typedef enum {
    TP_RECEIVER_IDLE,      // Trạng thái nghỉ, sẵn sàng nhận phiên mới
    TP_RECEIVER_WAIT_CF    // �?ã nhận First Frame, đang ch�? các Consecutive Frames
} TP_Receiver_State_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PCI_TYPE_SF 0x0 // Single Frame
#define PCI_TYPE_FF 0x1 // First Frame
#define PCI_TYPE_CF 0x2 // Consecutive Frame
#define PCI_TYPE_FC 0x3 // Flow Control

#define CAN_TP_REQUEST_ID   0x123 // ID mà chúng ta lắng nghe (ví dụ: request chẩn đoán)
#define CAN_TP_RESPONSE_ID  0x125 // ID mà chúng ta dùng để gửi trả l�?i (FC)

#define TP_BUFFER_SIZE 256 // Kích thước buffer tối đa cho tin nhắn ghép nối
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// --- Biến cho State Machine và việc ghép nối dữ liệu ---
volatile TP_Receiver_State_t g_tp_receiver_state = TP_RECEIVER_IDLE;
uint8_t  g_tp_reassembly_buffer[TP_BUFFER_SIZE];
volatile uint16_t g_tp_total_size = 0;
volatile uint16_t g_tp_bytes_received = 0;
volatile uint8_t  g_tp_expected_seq_num = 0;

// C�? báo cho vòng lặp main biết có tin nhắn hoàn chỉnh
volatile uint8_t g_tp_message_ready = 0;

// Biến cho FDCAN
FDCAN_TxHeaderTypeDef g_tx_header;
FDCAN_RxHeaderTypeDef g_rx_header;
uint8_t g_rx_data[8]; // Buffer nhận dữ liệu từ 1 frame CAN

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void FDCAN_ConfigFilter(void);
void send_flow_control_frame(void);
void print_reassembled_message(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief In tin nhắn đã được ghép nối hoàn chỉnh ra UART.
  *        Hàm này được g�?i từ main loop để tránh làm chậm ngắt.
  */
void print_reassembled_message(void) {
    char msg[100];
    snprintf(msg, sizeof(msg), "\r\n--- Complete TP Message Received ---\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    snprintf(msg, sizeof(msg), "Total Size: %u bytes\r\nData: ", g_tp_total_size);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    for (uint16_t i = 0; i < g_tp_total_size; i++) {
        snprintf(msg, sizeof(msg), "%02X ", g_tp_reassembly_buffer[i]);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n----------------------------------\r\n", 36, 100);
}

/**
  * @brief Gửi frame Flow Control (FC) để báo cho bên gửi tiếp tục.
  */
void send_flow_control_frame(void) {
    g_tx_header.Identifier = CAN_TP_RESPONSE_ID;
    g_tx_header.IdType = FDCAN_STANDARD_ID;
    g_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    g_tx_header.DataLength = FDCAN_DLC_BYTES_3;
    g_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    g_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    g_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    g_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    g_tx_header.MessageMarker = 0;

    uint8_t tx_data[3];
    // [PCI - 0x30]: FlowStatus = CTS, BlockSize = 0, STmin = 0
    tx_data[0] = 0x30; // 0b0011 0000: Flow Control, Continue To Send
    tx_data[1] = 0x00; // Block Size (BS) = 0 (gửi tất cả mà không cần FC nữa)
    tx_data[2] = 0x00; // Separation Time (STmin) = 0 ms

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &g_tx_header, tx_data) != HAL_OK) {
        Error_Handler();
    }

    char msg[] = "-> Sent Flow Control (CTS)\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100); // Gửi log nhanh
}

/**
  * @brief Ngắt được g�?i khi có tin nhắn mới trong FIFO0 của FDCAN.
  *        �?ây là trái tim của bộ nhận CAN-TP.
  */

void process_can_tp_frame(FDCAN_RxHeaderTypeDef* rx_header, uint8_t* rx_data) {
    uint8_t pci_type = (rx_data[0] & 0xF0) >> 4;

    switch (g_tp_receiver_state) {
        case TP_RECEIVER_IDLE:
        {
            if (pci_type == PCI_TYPE_SF) {
                uint8_t sf_len = rx_data[0] & 0x0F;
                if (sf_len > 0 && sf_len < 8) {
                    memcpy(g_tp_reassembly_buffer, &rx_data[1], sf_len);
                    g_tp_total_size = sf_len;
                    g_tp_message_ready = 1;
                }
            } else if (pci_type == PCI_TYPE_FF) {
                g_tp_total_size = ((uint16_t)(rx_data[0] & 0x0F) << 8) | rx_data[1];
                if (g_tp_total_size > TP_BUFFER_SIZE) return;

                memcpy(g_tp_reassembly_buffer, &rx_data[2], 6);
                g_tp_bytes_received = 6;
                g_tp_expected_seq_num = 1;

                g_tp_receiver_state = TP_RECEIVER_WAIT_CF;
                send_flow_control_frame();
            }
            break;
        }

        case TP_RECEIVER_WAIT_CF:
        {
            if (pci_type == PCI_TYPE_CF) {
                uint8_t seq_num = rx_data[0] & 0x0F;
                if (seq_num != g_tp_expected_seq_num) {
                    g_tp_receiver_state = TP_RECEIVER_IDLE;
                    return;
                }

                // Chuyển DLC sang byte
                uint32_t dlc_in_bytes;
                switch (rx_header->DataLength) {
                    case FDCAN_DLC_BYTES_1: dlc_in_bytes = 1; break;
                    case FDCAN_DLC_BYTES_2: dlc_in_bytes = 2; break;
                    case FDCAN_DLC_BYTES_3: dlc_in_bytes = 3; break;
                    case FDCAN_DLC_BYTES_4: dlc_in_bytes = 4; break;
                    case FDCAN_DLC_BYTES_5: dlc_in_bytes = 5; break;
                    case FDCAN_DLC_BYTES_6: dlc_in_bytes = 6; break;
                    case FDCAN_DLC_BYTES_7: dlc_in_bytes = 7; break;
                    case FDCAN_DLC_BYTES_8: dlc_in_bytes = 8; break;
                    default: dlc_in_bytes = 0; break;
                }

                uint32_t bytes_to_copy = dlc_in_bytes - 1;
                if (g_tp_bytes_received + bytes_to_copy > g_tp_total_size) {
                    bytes_to_copy = g_tp_total_size - g_tp_bytes_received;
                }

                memcpy(&g_tp_reassembly_buffer[g_tp_bytes_received], &rx_data[1], bytes_to_copy);
                g_tp_bytes_received += bytes_to_copy;
                g_tp_expected_seq_num = (g_tp_expected_seq_num + 1) % 16;

                if (g_tp_bytes_received >= g_tp_total_size) {
                    g_tp_message_ready = 1;
                    g_tp_receiver_state = TP_RECEIVER_IDLE;
                }
            } else {
                g_tp_receiver_state = TP_RECEIVER_IDLE;
            }
            break;
        }
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &g_rx_header, g_rx_data) != HAL_OK) {
            g_tp_receiver_state = TP_RECEIVER_IDLE;
            return;
        }

        // --- In log xác nhận frame đã nhận ---
        char log_msg[100];

        // In ID
        snprintf(log_msg, sizeof(log_msg), "\r\n[CAN RX] ID: 0x%03lX | Data: ", (unsigned long)g_rx_header.Identifier);
        HAL_UART_Transmit(&huart2, (uint8_t*)log_msg, strlen(log_msg), 100);

        // In dữ liệu theo độ dài thực tế (DLC)
        uint32_t dlc_in_bytes;
        switch (g_rx_header.DataLength) {
            case FDCAN_DLC_BYTES_0: dlc_in_bytes = 0; break;
            case FDCAN_DLC_BYTES_1: dlc_in_bytes = 1; break;
            case FDCAN_DLC_BYTES_2: dlc_in_bytes = 2; break;
            case FDCAN_DLC_BYTES_3: dlc_in_bytes = 3; break;
            case FDCAN_DLC_BYTES_4: dlc_in_bytes = 4; break;
            case FDCAN_DLC_BYTES_5: dlc_in_bytes = 5; break;
            case FDCAN_DLC_BYTES_6: dlc_in_bytes = 6; break;
            case FDCAN_DLC_BYTES_7: dlc_in_bytes = 7; break;
            case FDCAN_DLC_BYTES_8: dlc_in_bytes = 8; break;
            default: dlc_in_bytes = 0; break;
        }

        for (uint32_t i = 0; i < dlc_in_bytes; i++) {
            snprintf(log_msg, sizeof(log_msg), "%02X ", g_rx_data[i]);
            HAL_UART_Transmit(&huart2, (uint8_t*)log_msg, strlen(log_msg), 100);
        }
        HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);

        // --- G�?i hàm xử lý CAN-TP ---
        process_can_tp_frame(&g_rx_header, g_rx_data);
    }
}


void FDCAN_ConfigFilter(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;               // L�?c ID 11-bit
    sFilterConfig.FilterIndex = 0;                          // Filter đầu tiên
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;           // Dùng chế độ mask
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;   // Frame hợp lệ sẽ vào FIFO0

    sFilterConfig.FilterID1 = 0x000;    // Giá trị so sánh = 0
    sFilterConfig.FilterID2 = 0x000;    // Mask = 0 -> b�? qua so sánh tất cả bit

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
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
  MX_FDCAN1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  FDCAN_ConfigFilter();

  // Khởi động FDCAN
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
      Error_Handler();
  }

  // Kích hoạt ngắt báo có tin nhắn mới trong FIFO0
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
      Error_Handler();
  }

  char uartBuf[100];
  sprintf(uartBuf, "CAN-TP Receiver Ready. Listening on ID 0x%03X...\r\n", CAN_TP_REQUEST_ID);
  HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 42;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 3;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 10;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
