
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "LSM6DSL.h"
#include "stdlib.h"

void SystemClock_Config(void);

void bubbleSort(float *num, int size)
{
  // Для всех элементов
  for (int i = 0; i < size - 1; i++)
  {
    for (int j = (size - 1); j > i; j--) // для всех элементов после i-ого
    {
      if (num[j - 1] > num[j]) // если текущий элемент меньше предыдущего
      {
        float temp = num[j - 1]; // меняем их местами
        num[j - 1] = num[j];
        num[j] = temp;
      }
    }
  }
}

float median(float *array, int size, float value)
{
  //---В цикле сдвигаем массив на одну ячяейку влево
  for (int i = 0; i < size; i++)
  {
    array[i] = array[i + 1];
  }
  //---В последнюю ячейку массива записываем значение гироскопа-
  array[size - 1] = value;

  //---Выделяем память под динамический массив buffer
  //--- и записываем элементы c массива array
  float buffer[size];
  for (int i = 0; i < size; i++)
  {
    buffer[i] = array[i];
  }

  //---сортируем массив buffer
  bubbleSort(buffer, size);

  //---Выбираем из сортированного массива серединное значение
  float ansver;
  if (size % 2 == 1)
  {
    ansver = buffer[(int)size / 2];
  }
  else
  {
    ansver = ((buffer[(int)(size / 2)] + buffer[((int)(size / 2)) + 1])) / 2;
  }
  //---возвращаем ответ
  return ansver;
}

float srednee(float *array, int size, float value)
{
  float sum = 0;
  //---В цикле сдвигаем массив на одну ячяейку влево
  for (int i = 0; i < size; i++)
  {
    array[i] = array[i + 1];
  }
  //---В последнюю ячейку массива записываем значение гироскопа-
  array[size - 1] = value;
  //---суммируем все ячейки массива
  for (int i = 0; i < size; i++)
  {
    sum += array[i];
  }
  //---рассчитываем среднеарифметическое
  sum = sum / size;
  return sum;
}

int main(void)
{
  //--------------инициализация перефирии микроконтроллера--------------
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  //--------------объявляем переменную структуры--------------
  LSM6DSL_CONFIG config;
  //--------------передаем структуру i2c----------------------
  config.i2c = &hi2c1;
  //--------------задаем частоту обновления-------------------
  config.odr = GYRO_ODR_6664HZ;
  //--------------задаем диапазон чувствительности------------
  config.full_scale_select = GYRO_FULL_SCALE_SELECT_500;
  //--------------выбираем какой фильтр использовать----------
  config.filter_select = GYRO_FILTER_SELECT_NONE_FILTER;
  //--------------задаем полосу пропускания LP фильтра--------
  config.lpf1_bandwidth_select = GYRO_LPF1_BANDWIDTH_SELECT_THREE;
  //--------------задаем полосу пропускания HP фильтра--------
  config.hpf_bandwidth_select = GYRO_HP_FILTER_BANDWITDTH_SELECT_1040;
  //--------------включиить режим высокой производительности--
  config.high_perfomance_mode_select = GYRO_HIGH_PERFOMANCE_MODE_ENABLED;
  //----------------------------------------------------------
  char str[50];
  int32_t getx, gety, getz;
  float x_conv, y_conv, z_conv;
  int16_t temp;
  LSM6DSL_Init(&config);
  float calibrate[3];
  Gyro_Calibrate(&config, calibrate);
  float medbuf[15];
  float sredbuf[15];
  while (1)
  {
    temp = Read_temp(&config);              //получаем температуру с датчика
    getx = Gyro_getX(&config);              //получаем 16 битное значение оси X
    gety = Gyro_getY(&config);              //получаем 16 битное значение оси Y
    getz = Gyro_getZ(&config);              //получаем 16 битное значение оси Z
    x_conv = Gyro_Convert(&config, (getx)); //конвертируем (полученное значение-калибровка) по оси X в град/сек
    y_conv = Gyro_Convert(&config, (gety)); //конвертируем (полученное значение-калибровка) по оси Y в град/сек
    z_conv = Gyro_Convert(&config, (getz)); //конвертируем (полученное значение-калибровка) по оси Z в град/сек
    //-----------------формат строки-----------------------------------------
    uint8_t value;
    HAL_I2C_Mem_Read(&hi2c1, DEVICE_ADDRESS, CTRL7_G, I2C_MEMADD_SIZE_8BIT, &value, 1, 0xFF);

    sprintf(str, "$%x %0.2f %0.2f %0.2f;\r\n", value, x_conv, y_conv, z_conv);
    //-----------------передача строки по UART-------------------------------
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);
  }
}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

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