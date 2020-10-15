/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include "LiquidCrystal.h"
#include "microdelay.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t data;
char text[200];
int count = 0;


uint16_t led[] = {
	GPIO_PIN_9,
	GPIO_PIN_8,
	GPIO_PIN_15,
	GPIO_PIN_14,
	GPIO_PIN_13,
	GPIO_PIN_12,
	GPIO_PIN_11,
	GPIO_PIN_10
};

uint16_t pins[] = {
	GPIO_PIN_7,
	GPIO_PIN_6,
	GPIO_PIN_5,
	GPIO_PIN_4
};

char keys[4][4] = {
	{'x', '*', ' ', '#'},
	{'x', '7', '8', '9'},
	{'x', '4', '5', '6'},
	{'x', '1', '2', '3'}
};

int clickCursor = 0;
int clickLast = -1;
int clickTimer = 0;
int clickCount = 0;
char keyPlace[4][4] = {
	{0, 9, 10, 11},
	{0, 6, 7, 8},
	{0, 3, 4, 5},
	{0, 0, 1, 2}
};

char lets[][5] = {
	{'1', ' ', ' ', ' ', ' '},
	{'a', 'b', 'c', '2', ' '},
	{'d', 'e', 'f', '3', ' '},
	
	{'g', 'h', 'i', '4', ' '},
	{'j', 'k', 'l', '5', ' '},
	{'m', 'n', 'o', '6', ' '},
	
	{'p', 'q', 'r', 's', '7'},
	{'t', 'u', 'v', '8', ' '},
	{'w', 'x', 'y', 'z', '9'},
	
	{'*', '+', ' ', ' ', ' '},
	{'0', ' ', ' ', ' ', ' '},
	{'#', ' ', ' ', ' ', ' '}

};

int letCount[12] = {1, 4, 4,		 4, 4, 4,		 5, 4, 5,		 2, 2, 1}; 

uint16_t bcdPins[] = {
	GPIO_PIN_0,
	GPIO_PIN_1,
	GPIO_PIN_2,
	GPIO_PIN_3
};

uint8_t airplane[] = {
  0x06,
  0x04,
  0x0D,
  0x1F,
  0x1F,
  0x0D,
  0x04,
  0x06
};

uint8_t gas[] = {
  0x00,
  0x00,
  0x1F,
  0x1F,
  0x0B,
  0x0F,
  0x00,
  0x00
};

uint8_t normalBomb[] = {
  0x00,
  0x0A,
  0x1F,
  0x11,
  0x11,
  0x1F,
  0x0A,
  0x00
};

uint8_t deathlyBomb[] = {
  0x15,
  0x0E,
  0x1F,
  0x15,
  0x15,
  0x1F,
  0x0E,
  0x15
};

uint8_t explotion[] = {
  0x13,
  0x1A,
  0x0E,
  0x1F,
  0x0E,
  0x17,
  0x0A,
  0x11
};

uint8_t drop[] = {
  0x00,
  0x04,
  0x0E,
  0x0E,
  0x1F,
  0x1F,
  0x0E,
  0x00
};

uint8_t skull[] = {
  0x00,
  0x06,
  0x19,
  0x17,
  0x17,
  0x19,
  0x06,
  0x00
};

const int MATCH_LEN = 105;
const int SEG_LEN = 40;
uint8_t allMap[MATCH_LEN][4] = {0};

uint8_t map[4][20] = {
	{0, 0, 0, 1, 0, 0, 0, 3, 0, 0, 0, 0, 2, 0, 0, 3, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0},
	{0, 0, 0, 2, 0, 0, 2, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0}
};

uint8_t omap[4][20] = {
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

int option = 0;
int level = 1;
int state = 0;
int lp = 20;
int blinkC = 0;
int blinkS = -1;
int started = 0;

int fuel = 8;

int keyState = 0;
int frame = 9000;

int speed = 0;
int maxSpeed = 400;
int engine = 0;

int score = 0;
int coef = 0;

int x = 19, y = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void printScene(void);
void onCharPressed(char x);
void show(void);
void generateMap(void);
void drawAirplane(void);
void showAboutUs(void);
void printMenu(void);
void startBlinker(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void blueButton(){
	if(state == 0){
		if(option == 0){	
			generateMap();
			show();
			drawAirplane();
			state = 1;
		}else if(option == 1){
			state = 4;
			startBlinker();
		}else if(option == 2){
			showAboutUs();
			state = 2;
		}
		option = 0;
	}else if(state == 1){
		started = 1;
		engine = 1;
	}else if(state == 2){
		state = 0;
		printMenu();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart2, &data, sizeof(char));
	if(huart->Instance == USART2){
		text[count] = data;
		if(data == 'E'){
			setCursor(0, 0);
			text[count] = 0;
			if(text[0] == 'M'){
				onCharPressed(text[1]);
			}else if(text[0] == 'S'){
				blueButton();
			}
			
			count = 0;
		}else{
			count++;
			if(count > 200){
				count = 0;
			}
		}
	}
}

void setup() {
  createChar(0, airplane);
  createChar(1, gas);
  createChar(2, normalBomb);
  createChar(3, deathlyBomb);
	createChar(4, explotion);
	createChar(5, skull);
}

int myrand(){
		HAL_ADC_Start(&hadc2);
		int r = HAL_ADC_GetValue(&hadc2);
		return r;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) && GPIO_Pin == GPIO_PIN_0){
		blueButton();
	}
	else if(HAL_GPIO_ReadPin(GPIOD, GPIO_Pin) && keyState == 0){
		keyState = 1;
		int col = 0, row = 0;
		if(GPIO_Pin == GPIO_PIN_1){
			col = 1;
		}else if(GPIO_Pin == GPIO_PIN_2){
			col = 2;
		}else if(GPIO_Pin == GPIO_PIN_3){
			col = 3;
		}
		
		for(int i = 0; i < 4; i++){
			for(int j = 0; j < 4; j++){
				if(j == i){
					HAL_GPIO_WritePin(GPIOD, pins[j], GPIO_PIN_SET);
				}else{
					HAL_GPIO_WritePin(GPIOD, pins[j], GPIO_PIN_RESET);
				}
			}
			
			if(HAL_GPIO_ReadPin(GPIOD, GPIO_Pin)){
				row = i;
				break;
			}
		}
		
		for(int j = 0; j < 4; j++){
			HAL_GPIO_WritePin(GPIOD, pins[j], GPIO_PIN_SET);
		}
		
		char x = keys[row][col];
		
		onCharPressed(x);
		
		while(HAL_GPIO_ReadPin(GPIOD, GPIO_Pin));
		HAL_Delay(10);
		keyState = 0;
	}
}

void fall(){
	for(int i = 0; i < 4; i++){
		for(int j = 1; j < 20; j++){
			map[i][j] = omap[i][j-1];
		}
	}
	
	if(lp < MATCH_LEN){
		for(int i = 0; i < 4; i++){
			map[i][0] = allMap[lp][i];
		}
		lp++;
	}else{
		for(int i = 0; i < 4; i++){
			map[i][0] = 0;
		}
	}
}

void show(){
	int p = 0;
	for(int i = 0; i < 4; i++){
		setCursor(0, i);
		char temp[21];
		for(int j = 0; j < 20; j++){
			if(map[i][j] == 0){
				temp[p++] = ' ';
			}else{
				temp[p++] = '\0';
				if(p > 1){
					print(temp);
				}
				p = 0;
				write(map[i][j]);
			}
		}
		temp[p++] = '\0';
		if(p > 1){
			print(temp);
		}
		p = 0;
	}
}

int segNumber = 0;
void setNumber(int num){
		GPIO_PinState state[4];
		int temp = num;
		for(int i = 0; i < 4; i++){
			if(temp % 2 == 0){
				state[i] = GPIO_PIN_RESET;
			}else{
				state[i] = GPIO_PIN_SET;
			}
			temp = temp / 2;
		}
		
		for(int i = 0; i < 4; i++){
			if(segNumber == i){
				HAL_GPIO_WritePin(GPIOC, bcdPins[i], GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(GPIOC, bcdPins[i], GPIO_PIN_RESET);
			}
		}
		
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, state[0]);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, state[1]);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, state[2]);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, state[3]);
}

void gameOver(){
	state = 6;
	startBlinker();
}

void win(){
	state = 7;
	startBlinker();
}

void setFuel(){
	if(fuel == 0){
		engine = 0;
	}else if(fuel == 1){
		for(int i = 1; i < 8; i++){
			HAL_GPIO_WritePin(GPIOE, led[i], GPIO_PIN_RESET);
		}
		HAL_GPIO_TogglePin(GPIOE, led[0]);
	}else{
		for(int i = 0; i < 8; i++){
			if(i < fuel){
				HAL_GPIO_WritePin(GPIOE, led[i], GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(GPIOE, led[i], GPIO_PIN_RESET);
			}
		}
	}
}

void removeAirplane(){
	setCursor(x, y);
	print(" ");
}

void drawAirplane(){
	setCursor(x, y);
	if(map[y][x] == 1){
		fuel = (fuel + 1 > 8) ? 8 : fuel + 1;
		write(0);
		map[y][x] = 0;
	}else if(map[y][x] == 2){
		fuel--;
		write(4);
		map[y][x] = 0;
	}else if(map[y][x] == 3){
		write(5);
		map[y][x] = 0;
		HAL_Delay(1000);
		gameOver();
	}else{
		write(0);
	}
}

void onRefresh(){
	if(state == 1){
		if(segNumber == 0){
			setNumber(level);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		}else{
			int x = speed;
			for(int i = 3; i > segNumber; i--){
				x /= 10;
			}
			x = x % 10;
			setNumber(x);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
		}
		segNumber++;
		segNumber %= 4;
	}else if(state == 0){
		for(int i = 0; i < 4; i++){
			HAL_GPIO_WritePin(GPIOC, bcdPins[i], GPIO_PIN_SET);
		}
		
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	}
}

void onFrame(){
	if(state == 1){
		if(speed == 0){
			HAL_Delay(30);
			return;
		}
		
		if(frame++ < (900 / speed)){
			return;
		}
		frame = 0;
		
		memcpy(omap, map, 80);
		fall();

		show();
		drawAirplane();
		
	}
}

int accelC = 0;
void accelerate(){
	if(accelC++ < 250){
			return;
	}
	accelC = 0;

	setFuel();
	if(engine){
		HAL_ADC_Start(&hadc1);
		maxSpeed = (410.f * HAL_ADC_GetValue(&hadc1)) / 63;
		maxSpeed = (maxSpeed < 0) ? 0 : maxSpeed;
		maxSpeed += 90;
	
		speed = (maxSpeed > (speed + 1)) ? (speed + 1) : maxSpeed;
	}else{
		speed = (0 > (speed - 1)) ? 0 : speed - 1;
	}
	
	if(speed == 0 && engine == 0 && started == 1){
		gameOver();
	}
}

void startBlinker(){
	clear();
	blinkS = -1;
	blinkC = 999999;
}

void blinker(){
	if(blinkC++ < ((blinkS == 1) ? 15000 : 5000)){
		return;
	}
	blinkC = 0;
	blinkS *= -1;
	
	if(state == 4){
			if(blinkS == 1){
				setCursor(0, 0);
				print("Waiting For Data...");
			}else if(blinkS == -1){
				clear();
			}
	}else if(state == 6){
		if(blinkS == 1){
			setCursor(5, 1);
			print("Game Over");
		}else if(blinkS == -1){
			clear();
		}
	}else if(state == 7){
		if(blinkS == 1){
			setCursor(6, 1);
			print("You Won");
		}else if(blinkS == -1){
			clear();
		}
	}
}

void printMenu(){
	clear();
	setCursor(0, 0);
	print("ALIEN WAR");
	setCursor(2, 1);
	if(option == 0){
		print(">");
	}else{
		print(" ");
	}
	setCursor(4, 1);
	print("START GAME");
	
	setCursor(2, 2);
	if(option == 1){
		print(">");
	}else{
		print(" ");
	}
	setCursor(4, 2);
	print("LOAD  GAME");
	
	setCursor(2, 3);
	if(option == 2){
		print(">");
	}else{
		print(" ");
	}
	setCursor(4, 3);
	print("ABOUT");
	
}

void showAboutUs(){
	clear();
	setCursor(0, 0);
	print("Mobin Vahdati");
	setCursor(0, 1);
	print("Homa Habashi");
}

void onCharPressed(char ch){
	if(state == 1){
		if(ch == '*'){
			removeAirplane();
			y = (y + 1 > 3) ? 3 : y + 1;
			drawAirplane();
		}else if(ch == '#'){
			removeAirplane();
			y = (y - 1 < 0) ? 0 : y - 1;
			drawAirplane();
		}else if(ch == ' '){
			removeAirplane();
			x = (x + 1 > 19) ? 19 : x + 1;
			drawAirplane();
		}else if(ch == '8'){
			removeAirplane();
			x = (x - 1 < 1) ? 1 : x - 1;
			drawAirplane();
		}else{
			
		}
	}else if(state == 0){
		if(ch == ' '){
			option = (option + 1 > 2) ? 2 : option + 1;
			printMenu();
		}else if(ch == '8'){
			option = (option - 1 < 0) ? 2 : option - 1;
			printMenu();
		}
	}
}

void generateMap(){
	for(int k = 10; k < MATCH_LEN; k+=SEG_LEN){
		uint8_t temp[SEG_LEN][4] = {0};
		int fuelCount = 5 / level;
		int normalBomb = 4 * level;
		int deathlyBomb = level;
		
		for(int i = 0; i < fuelCount; i++){
			int x = myrand() % 4;
			int y = myrand() % SEG_LEN;
			temp[y][x] = 1;
		}
		
		for(int i = 0; i < normalBomb; i++){
			int x = myrand() % 4;
			int y = myrand() % SEG_LEN;
			temp[y][x] = 2;
		}
		
		for(int i = 0; i < deathlyBomb; i++){
			int x = myrand() % 4;
			int y = myrand() % SEG_LEN;
			temp[y][x] = 3;
		}
		
		memcpy(allMap + k, temp, SEG_LEN * 4);
	}
	
	for(int i = 0; i < 20; i++){
		for(int j = 0; j < 4; j++){
			map[j][i] = allMap[19 - i][j];
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	init_DWT();
	LiquidCrystal(GPIOD, GPIO_PIN_8,GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14);
	begin(20, 4);
	setup();
	
	printMenu();
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
	
	
	unsigned char text[] = "Hello\n";
	
	HAL_UART_Transmit(&huart2, text, sizeof(unsigned char) * 6, 1000);
	HAL_UART_Receive_IT(&huart2, &data, sizeof(char));
	
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_6B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 899;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 15999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 89;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 79;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_2;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USB init function */
static void MX_USB_PCD_Init(void)
{

  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.ep0_mps = DEP0CTL_MPS_64;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin 
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin 
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pins : PE2 PE6 PE7 PE0 
                           PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_0 
                          |GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin 
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin 
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin 
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin 
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT3_Pin MEMS_INT4_Pin */
  GPIO_InitStruct.Pin = MEMS_INT3_Pin|MEMS_INT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC4 PC5 PC6 
                           PC7 PC8 PC9 PC10 
                           PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF9 PF10 PF4 PF6 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA8 PA9 PA10 
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 PB4 PB5 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 
                           PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11 
                           PD12 PD13 PD14 PD15 
                           PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD1 PD2 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_TSC_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
