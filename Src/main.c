/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  *Volatile Variables: Declaring a variable as volatile tells the compiler that
  *the variable can be modified at any time externally to the implementation,
  *for example, by the operating system or by hardware. Because the value of a
  *volatile-qualified variable can change at any time, the physical address of
  *the variable in memory must always be accessed whenever the variable is
  *referenced in code. This means the compiler cannot perform optimizations on
  *the variable, for example, caching it in a local register to avoid memory
  *accesses.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include "stm32f769i_discovery.h"
#include "stm32f769i_discovery_lcd.h"
#include "stm32f769i_discovery_ts.h"
#include "image.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIZE					8
#define SQUARE                	BSP_LCD_GetYSize()/10 //Square edge
#define BALL					16
#define CIRCLE					21
//Place Pieces
#define EMPTY					0
#define CHANTILLY				1
#define STRAWBERRY				2
#define POSS1					3//Possible moves of player 1
#define POSS2					4//Possible moves of player 2
//Temperature
#define TEMP_REFRESH_PERIOD   	1000//Internal temperature refresh period */
#define MAX_CONVERTED_VALUE   	4095//Max converted value */
#define AMBIENT_TEMP            25//Ambient Temperature */
#define VSENS_AT_AMBIENT_TEMP  	760//VSENSE value (mv) at ambient temperature */
#define AVG_SLOPE               25//Avg_Solpe multiply by 10 */
#define VREF                  	3300//Voltage: 3.3V (3300mV)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DMA2D_HandleTypeDef hdma2d;

DSI_HandleTypeDef hdsi;

LTDC_HandleTypeDef hltdc;

SD_HandleTypeDef hsd2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
TS_StateTypeDef TS_State; //Touch screen state
volatile uint8_t flagTS = 0; //Flag Touch screen
volatile uint8_t flagPB = 0; //Flag Push Button
volatile uint8_t flagMenu = 0; //Flag start Menu
volatile uint8_t flagT2 = 0; //Flag timer 2 (20s down)
volatile uint8_t flagT6 = 0; //Flag timer 6
volatile uint8_t flagT7 = 0; //Flag timer 7
volatile uint8_t flagOnePlayer = 0;//Flag One Player (playing against ARM - Option in Initial Menu)
volatile uint8_t flagTwoPlayers = 0;//Flag Two Players (playing with 2 persons - Option in Initial Menu)

volatile uint8_t counterPieces1 = 0; //counter player 1 pieces
volatile uint8_t counterPieces2 = 0; //counter player 2 pieces
volatile uint8_t counterPassTurn1 = 0;//Goal 7 - Play 20s and pass to next player
volatile uint8_t counterPassTurn2 = 0;//Goal 7 - Play 20s and pass to next player
volatile uint8_t counterMoves = 0;//Option to test SD card after some moves
volatile uint8_t counterPoss1 = 0; //counter possible moves player 1
volatile uint8_t counterPoss2 = 0; //counter possible moves player 1
volatile uint8_t counterT2 = 20; // counter timer 2 (start in 20 seconds - decreasing)
volatile uint8_t counterT6 = 0; // counter timer 6
volatile uint8_t counterT7 = 0; //counter timer 7

volatile uint16_t score1 = 0; //scores player 1
volatile uint16_t score2 = 0; //scores player 2

long int JTemp = 0; //Internal Temperature converted
uint32_t ConvertedValue; //Value to convert internal temperature
char string[100];//Write in SD card

volatile char board [SIZE][SIZE] = {0}; //initialise board empty
volatile uint16_t posX; //variable to put piece
volatile uint16_t posY; //variable to put piece
unsigned int nBytes; //
int valueX; //For the touch screen detection (TS_State.touchX[0])
int valueY; //For the touch screen detection (TS_State.touchY[0])
_Bool pf;//flag to be able to separate Menu(TS) and gameboard (TS)
int gf = 0; //another flag to be able to separate Menu(TS) and gameboard (TS)
int player = CHANTILLY;  //variable to swap players (starts with Player 1 - Chantilly)
_Bool flagReset = 0;//flag to perform goal 10 - Reset clock (gameTime)


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DMA2D_Init(void);
static void MX_DSIHOST_DSI_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_SDMMC2_SD_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
static void LCD_Config();
void gameTime();
void playerTime();
void temperature();
void startMenu();
void initPositions();
void gameboard();
void clearBoard();
void colorPieces();
void putPieces(uint16_t, uint16_t);
int detectTS();
int nextPlayer();
void change(int i, int j, int x, int y);
int findPath(int i, int j);
int validAdjacent(int i, int j);
int validEntrap (int i, int j);
int validatePosition(int i, int j);
void findPossible();
int listPossible(int playPoss);
void ARM();
void winner_looser();
void scores();
void gameInfo();
void writeSDcard();
void resetGameTime(int x, int y);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) //Called each time an external interrupt occurs (TS and PB)
{
	if(GPIO_Pin == GPIO_PIN_13) //External interrupt Pin 13 - Touch Screen
	{
		BSP_TS_GetState(&TS_State);
		HAL_Delay(100); //provides a delay in 100 ms

		if(TS_State.touchDetected)
		{
			flagTS = 1;
			valueX = TS_State.touchX[0];
			valueY = TS_State.touchY[0];
		}
	}

	if(GPIO_Pin == GPIO_PIN_0) //External interrupt Pin 0 - Push Button
	{
		flagPB = 1;
		BSP_LED_Toggle(LED_RED); //To test if it´s working
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adcHandle) //Analogic to Digital converter (ADC1 - Internal Temperature Sensor)
{
	if(adcHandle == &hadc1)
		ConvertedValue = HAL_ADC_GetValue(&hadc1);
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) //Callback Timer base interrupts
{
	if(htim->Instance == TIM6) //APB1 clock source 100MHz - Clock GameTime
	{
		counterT6++;
		flagT6 = 1;
	}

	if(htim->Instance == TIM7) //APB1 clock source 100MHz - Temperature each 2s
	{
		counterT7++;
		flagT7 = 1;
	}

	if(htim->Instance == TIM2) //APB1 clock source 100MHz - playerTime (20 sec decreasing)
	{
		counterT2--;
		flagT2 = 1;
	}
}

//TIMERS - Player Timer, Total Game Time and Temperature each 2s

void gameTime() //Total time of the game
{
	if(flagT6)
	{
	    int hh = 0, mm = 0, ss = 0;

		flagT6 = 0;
		//Show time in hh:mm:ss
		hh = counterT6 / 3600;
		mm = (counterT6 - 3600*hh)/ 60;
		ss = (counterT6 - 3600*hh - mm*60);
		BSP_LCD_SetBackColor(LCD_COLOR_LIGHTRED);
		sprintf(string, "Time: %2d:%2d:%2d", hh, mm, ss);
		BSP_LCD_SetFont(&Font12);
		BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 214, (uint8_t *)string, RIGHT_MODE);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

		if(flagReset) //Extra Goal 10 (Reset Time with Touch Screen)
		{
			flagReset = 0;
			counterT6 = 0;
		}
	}
}

void resetGameTime(int x, int y)//Extra Goal 10
{
	if(x > BSP_LCD_GetXSize()-120 && y < BSP_LCD_GetYSize()+30)
	{
		BSP_LED_Toggle(LED_GREEN);//To test if it´s working properly
		flagReset = 1;
	}
}

void playerTime() //Time for each player decide where to put the piece: 20s counting down
{
	if(flagT2)
	{
		flagT2 = 0;

		if((counterT2 >= 0 && counterT2 <=20) && (counterPoss1 != 0 || counterPoss2 != 0))
		{
			sprintf(string, "TIMER    %2d", counterT2);

			BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_SetFont(&Font12);

			if(player == CHANTILLY)
			{
				BSP_LCD_DisplayStringAt(BSP_LCD_GetYSize()-SIZE+40, BSP_LCD_GetYSize()/10+165, (uint8_t *)string, LEFT_MODE);
				BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
				BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
				BSP_LCD_SetFont(&Font16);
				BSP_LCD_DisplayStringAt(BSP_LCD_GetYSize()-SIZE+35, BSP_LCD_GetYSize()/10+255, (uint8_t *)"YOUR TURN", LEFT_MODE);
				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			}

			if(player == STRAWBERRY)
			{
				BSP_LCD_DisplayStringAt(BSP_LCD_GetYSize()-SIZE+195, BSP_LCD_GetYSize()/10+165, (uint8_t *)string, LEFT_MODE);
				BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
				BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
				BSP_LCD_SetFont(&Font16);
				BSP_LCD_DisplayStringAt(BSP_LCD_GetYSize()-SIZE+190, BSP_LCD_GetYSize()/10+255, (uint8_t *)"YOUR TURN", LEFT_MODE);
				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			}
		}

		if(counterT2 == 0 && (counterPoss1 != 0 || counterPoss2 != 0))
		{
			player = nextPlayer();
			colorPieces();
			counterT2 = 20;
			if(CHANTILLY)
				counterPassTurn1++;
			if(STRAWBERRY)
				counterPassTurn2++;
		}

		if(counterPassTurn1 == 3 || counterPassTurn2 == 3)
		{
			counterPoss1 = 0;
			counterPoss2 = 0;
		}

		if(counterPoss1 == 0 && counterPoss2 == 0)
		{
			BSP_LCD_SetFont(&Font20);
			BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
			BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
			BSP_LCD_DisplayStringAt(BSP_LCD_GetYSize()-SIZE+35, BSP_LCD_GetYSize()/10+255, (uint8_t *)"    GAME OVER    ", LEFT_MODE);
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			counterT2 = 0;
			flagT6 = 0;

			HAL_ADC_Stop_IT(&hadc1);
			writeSDcard();
			HAL_ADC_Start_IT(&hadc1);
		}
	}
}

void temperature() //Internal temperature of the ARM
{
	if(flagT7)
	{
		flagT7 = 0;

		JTemp = ((((ConvertedValue * VREF)/MAX_CONVERTED_VALUE) - VSENS_AT_AMBIENT_TEMP) * 10 / AVG_SLOPE) + AMBIENT_TEMP;
		//Show temperature in celsius
		BSP_LCD_SetBackColor(LCD_COLOR_LIGHTRED);
		sprintf(string, "Temp: %ld C", JTemp);
		BSP_LCD_SetFont(&Font12);
		BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 232, (uint8_t *)string, RIGHT_MODE);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	}
}

//Layouts, Menu and Boards

void startMenu() //Menu where we choose to play vs ARM or 2 players mode
{
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHTRED);
	BSP_LCD_DrawRect(246, 106, 308, 83);
	BSP_LCD_DrawRect(247, 107, 306, 81);
	BSP_LCD_FillRect(250, 110, 300, 75);
	BSP_LCD_DrawRect(246, 291, 308, 83);
	BSP_LCD_DrawRect(247, 292, 306, 81);
	BSP_LCD_FillRect(250, 295, 300, 75);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_LIGHTRED);
	BSP_LCD_DisplayStringAt(316, 320, (uint8_t *)"TWO PLAYERS", LEFT_MODE);
	BSP_LCD_DisplayStringAt(318, 135, (uint8_t *)"ONE PLAYER", LEFT_MODE);
}

void gameboard() //Print board
{
	  for(int i = 0; i<SIZE; i++)
	  {
		  int xPosition = SQUARE + i * SQUARE;

		  for(int j = 0; j<SIZE; j++)
		  {
			  int yPosition = SQUARE + j * SQUARE;

			  BSP_LCD_DrawRect(xPosition, yPosition, SQUARE, SQUARE);
			  BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
			  BSP_LCD_FillRect(xPosition, yPosition, SQUARE-2, SQUARE-2);
			  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		  }
	  }
}

void clearBoard() //Clean the board each time we came back to the start menu
{
	for(int i = 0; i<SIZE; i++)
	{
		for(int j = 0; j<SIZE; j++)
		{
			board[i][j] = 0;
		}
	}
}

void initPositions() //Place 4 first pieces
{
	board[3][3] = CHANTILLY;
	board[4][4] = CHANTILLY;
	board[3][4] = STRAWBERRY;
	board[4][3] = STRAWBERRY;
}

void gameInfo() //Board with game information (players, number of pieces, timers, scores, winner)
{
	 BSP_LCD_SetTextColor(LCD_COLOR_LIGHTRED);
	 BSP_LCD_FillRect(BSP_LCD_GetYSize()-SIZE, BSP_LCD_GetYSize()/10, 320, BSP_LCD_GetYSize()-2*SQUARE);
	 BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	 BSP_LCD_FillRect(BSP_LCD_GetYSize()-SIZE+2, BSP_LCD_GetYSize()/10+2, 316, BSP_LCD_GetYSize()-2*SQUARE-4);

	 BSP_LCD_SetTextColor(LCD_COLOR_LIGHTRED);
	 BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	 BSP_LCD_FillRect(BSP_LCD_GetYSize()-SIZE+4, BSP_LCD_GetYSize()/10+4, 312, 45);
	 BSP_LCD_SetFont(&Font16);
	 BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	 BSP_LCD_SetBackColor(LCD_COLOR_LIGHTRED);
	 BSP_LCD_DisplayStringAt(BSP_LCD_GetYSize()-SIZE+70, BSP_LCD_GetYSize()/10+20, (uint8_t *)"GAME INFORMATION", LEFT_MODE);
	 BSP_LCD_SetTextColor(LCD_COLOR_LIGHTRED);

	 BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	 BSP_LCD_FillRect(BSP_LCD_GetYSize()-SIZE+4, BSP_LCD_GetYSize()/10+325, 312, 55);
	 BSP_LCD_SetFont(&Font16);
	 BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

	 BSP_LCD_DisplayStringAt(BSP_LCD_GetYSize()-SIZE+30, BSP_LCD_GetYSize()/10+75, (uint8_t *)"CHANTILLY", LEFT_MODE);
	 BSP_LCD_DisplayStringAt(BSP_LCD_GetYSize()-SIZE+180, BSP_LCD_GetYSize()/10+75, (uint8_t *)"STRAWBERRY", LEFT_MODE);

	 BSP_LCD_SetFont(&Font12);
	 BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	 sprintf(string, "PIECES   %2d", counterPieces1);
	 BSP_LCD_DisplayStringAt(BSP_LCD_GetYSize()-SIZE+40, BSP_LCD_GetYSize()/10+120, (uint8_t *)string, LEFT_MODE);
	 sprintf(string, "PIECES   %2d", counterPieces2);
	 BSP_LCD_DisplayStringAt(BSP_LCD_GetYSize()-SIZE+195, BSP_LCD_GetYSize()/10+120, (uint8_t *)string, LEFT_MODE);
	 sprintf(string, "SCORE    %3d", score1);
	 BSP_LCD_DisplayStringAt(BSP_LCD_GetYSize()-SIZE+40, BSP_LCD_GetYSize()/10+210, (uint8_t *)string, LEFT_MODE);
	 sprintf(string, "SCORE    %3d", score2);
	 BSP_LCD_DisplayStringAt(BSP_LCD_GetYSize()-SIZE+195, BSP_LCD_GetYSize()/10+210, (uint8_t *)string, LEFT_MODE);
	 BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

	 BSP_LCD_SetBackColor(LCD_COLOR_LIGHTRED);
	 BSP_LCD_SetFont(&Font16);
	 BSP_LCD_DisplayStringAt(BSP_LCD_GetYSize()-SIZE+105, BSP_LCD_GetYSize()/10+336, (uint8_t *)"QUIT GAME", LEFT_MODE);
	 BSP_LCD_SetFont(&Font12);
	 BSP_LCD_DisplayStringAt(BSP_LCD_GetYSize()-SIZE+100, BSP_LCD_GetYSize()/10+360, (uint8_t *)"Press Blue Button", LEFT_MODE);

	 BSP_LCD_SetTextColor(LCD_COLOR_LIGHTRED);
	 BSP_LCD_DrawVLine(630, 95, 194);
	 BSP_LCD_DrawVLine(631, 95, 194);

	 BSP_LCD_DrawHLine(473, 289, 318);
	 BSP_LCD_DrawHLine(473, 290, 318);

	 BSP_LCD_DrawHLine(473, 335, 318);
	 BSP_LCD_DrawHLine(473, 336, 318);
	 BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
}

//Game and implemented rules

void colorPieces(void) //Define the piece colour of each player
{
	  for(int i = 0; i<SIZE; i++)
	  {
		  for(int j = 0; j<SIZE; j++)
		  {
				posX = SQUARE/2 + SQUARE*(i+1);
				posY = SQUARE/2 + SQUARE*(j+1);

			  if(board[i][j] == CHANTILLY)
			  {
				//Ball player 1 (X and Y)
				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
				BSP_LCD_FillCircle(posX, posY, BALL);
				BSP_LCD_DrawCircle(posX, posY, CIRCLE);
			  }
			  if(board[i][j] == STRAWBERRY)
			  {
				//Ball player 2 (X and Y)
				BSP_LCD_SetTextColor(LCD_COLOR_LIGHTRED);
				BSP_LCD_FillCircle(posX, posY, BALL);
				BSP_LCD_DrawCircle(posX, posY, CIRCLE);
			  }
			  if(board[i][j] == POSS1)
			  {
				//Ball player 2 (X and Y)
				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
				BSP_LCD_DrawCircle(posX, posY, CIRCLE);
			  }
			  if(board[i][j] == POSS2)
			  {
				//Ball player 2 (X and Y)
				BSP_LCD_SetTextColor(LCD_COLOR_LIGHTRED);
				BSP_LCD_DrawCircle(posX, posY, CIRCLE);
			  }
		  }
	  }
}

void putPieces(uint16_t x, uint16_t y) //Place pieces in the position (x,y) touched
{
    if((x >= SQUARE) && (y >= SQUARE) && (x <= (SQUARE*(SIZE+1))) && (y <= (SQUARE*(SIZE+1))))
    {
        for(int i=0; i<SIZE; i++)
		{
			for(int j=0; j<SIZE; j++)
			{
			    if((board[i][j] == POSS1 && player == CHANTILLY) || (board[i][j] == POSS2 && player == STRAWBERRY))
			    {
					if((x > SQUARE*(i+1)) && (x <= SQUARE*(i+2)) && (y > SQUARE*(j+1)) && (y <= SQUARE*(j+2)))
					{
						posX = SQUARE/2 + SQUARE*(i+1);
						posY = SQUARE/2 + SQUARE*(j+1);

						findPath(i,j); //Function to find validate move
						board[i][j] = player; //place piece in the board position
						colorPieces(); //print the piece
						HAL_Delay(1000);
						player = nextPlayer(); // pass to next player
						counterT2 = 20;
						return;
					}
			    }
			}
		}
	}
}

int detectTS()//Interrupt Touch Screen
{
	if(flagTS) //if flag touch screen activated
	{
		flagTS = 0;
		HAL_Delay(100);

		putPieces(valueX, valueY); //place piece in the position touched
		resetGameTime(valueX, valueY);//Goal 10 - reset timer when TS
		return 1;
	}
	return 0;
}

int nextPlayer() //swap to next player
{
	if(player == CHANTILLY)
		return STRAWBERRY;

	else
		return CHANTILLY;
}

void change(int i, int j, int x, int y) //change encapsulated pieces
{
    int auxX = i; //Coordenada auxX e auxY da peca inserida pelo jogador actual
    int auxY = j;

    if (x != i)
    {
        if (x < i)
            auxX--;
        else
            auxX++;
    }
    if (y != j)
    {
        if (y < j)
            auxY--;
        else
            auxY++;
    }

    while (board[auxX][auxY] == nextPlayer()) //Enquanto, no decorrer deste ciclo, nesta coordenada aparece a peca do jogador seguinte,
    {
        board[auxX][auxY] = player; // a mesma é substituida pela do jogador atual.
        if (x != i)
        {
            if (x < i)
                auxX--;
            else
                auxX++;
        }
        if (y != j)
        {
            if (y < j)
                auxY--;
            else
                auxY++;
        }
    }
}

int findPath(int i, int j) // Percorre todas as peças adjacentes
{
    int x, y, auxX = 0, auxY = 0; //Coordenada xy da peca inserida pelo jogador actual / auxX e auxY - coordenadas auxiliares

    for (x = i - 1; x <= i + 1; x++)
    {
        if (x < 0 || x > SIZE)
            continue;

        for (y = j - 1; y <= j + 1; y++)
        {
            if (y < 0 || y > SIZE)
                continue;

            if (board[x][y] == nextPlayer())
            {
                auxX = x; //Coordenada x do proximo ponto na mesma direcçao
                auxY = y; //Coordenada y do proximo ponto na mesma direcçao

                while (board[auxX][auxY] == nextPlayer()) // indica-nos a direcçao a seguir
                {
                    if (x != i)
                    {
                        if (x < i)
                            auxX--;
                        else
                            auxX++;
                    }

                    if (y != j)
                    {
                        if (y < j)
                            auxY--;
                        else
                            auxY++;
                    }
                    if (auxX < 0 || auxX > SIZE || auxY < 0 || auxY > SIZE)
                        break;

                    if (board[auxX][auxY] == EMPTY)
                        break;

                    if (board[auxX][auxY] == player)
                    {

                        change(i,j,auxX,auxY); //Funcao de troca de pecas do jogador seguinte pelas do jogador atual
                    }                          //encontradas pelo ciclo anterior quando verificada a condicao de "trap"
                }
            }
        }
    }
        return 0;
}

int validAdjacent(int i, int j) //Funcao que valida a regra de colocar a peça num espaco livre adjacente a peca do adversario
{                                 //horizontal, vertical, diagonal
    int x, y;

    for (x = i - 1; x <= i + 1; x++) //Ciclo de procura a esquerda e a direita duma posicao i
    {
        if (x < 0 || x > SIZE) //significa que esta a ler cabecalho - linha - e assim ignora esta posicao como valida
            continue;

        for (y = j - 1; y <= j + 1; y++)  //Ciclo de procura acima e abaixo duma posicao j
        {
            if (y < 0 || y > SIZE) //significa que esta a ler cabecalho - coluna - e assim ignora esta posicao como valida
                continue;
            if (board[x][y] == nextPlayer()) //Se na coordenada xy estiver uma peca do jogador seguinte:
                return 1;    //retorna 1 considerando a posicao valida
        }
    }
    return 0;
}

int validEntrap (int i, int j) //Funcao de validatePosition da regra de "trap"
{
    int x, y, auxX, auxY; //x e y coordenadas de posicao pretendida    auxX e auxY - coordenadas da posicao seguinte na mesma direcao

    for (x = i - 1; x <= i + 1; x++)   //Ciclo de procura acima e abaixo duma posicao i
    {
        if (x < 1 || x > SIZE) //significa que esta a ler cabecalho - linha - e assim ignora esta posicao como valida
            continue;

        for (y = j - 1; y <= j + 1; y++)   //Ciclo de procura acima e abaixo duma posicao j
        {
            if (y < 1 || y > SIZE)  //significa que esta a ler cabecalho - coluna -  e assim ignora esta posicao como valida
                continue;
            if (board[x][y] == nextPlayer())   // Se na coordenada xy estiver uma peca do jogador seguinte:
            {
                auxX = x; //proximo ponto na linha na mesma direcçao
                auxY = y;  //proximo ponto na coluna na mesma direcçao

                while (board[auxX][auxY] == nextPlayer()) //Enquanto na coordenada auxX auxY estiver uma peca do jogador seguinte
                {  // Este ciclo indica-nos a direcçao a seguir nas linhas e colunas
                    if (x != i) // Se x for diferente de i, a linha e diferente
                    {
                        if (x < i) //se x for inferior a i:
                            auxX--; //desloca o x para cima
                        else
                            auxX++; //caso contrario, desloca o x para baixo
                    }
                    if (y != j) //Se y for diferente de j, a coluna e diferente
                    {

                        if (y < j)
                            auxY--;
                        else
                            auxY++;
                    }
                    if (auxX < 1 || auxX > SIZE || auxY < 1 || auxY > SIZE) //Se se verificar, esta a ler o cabecalho
                        break; //interrompe o ciclo assim que o if se verificou

                    if (board[auxX][auxY] == 0) //Se se verificar, encontrou espaço vazio
                        break;

                    if (board[auxX][auxY] == player) //Se na coordenada auxX auxY, encontrar uma peca do jogador actual
                        return 1; //Reinicia o ciclo
                }
            }
        }
    }
    return 0;
}

int validatePosition(int i, int j) //se passar nestas condicoes todas retorna 1, que indica jogada valida - display.c
{
    if (board [i][j] != EMPTY && board [i][j] != POSS1 && board [i][j] != POSS2) //posicao disponivel em linha - i e coluna - j, dado que 0 indica espaço vazio
    {
        return 0;
    }

    if (validEntrap(i, j) == 0) //Funcao de validatePosition da regra de "trap"
    {
        return 0;
    }
    return 1;
}

void findPossible()
{
	for(int i=0; i<SIZE; i++)//
	{
		for(int j=0; j<SIZE; j++)
		{
			if(validatePosition(i,j))
			{
				if(player == CHANTILLY)
				{
					board[i][j] = POSS1;
				}
				if(player == STRAWBERRY)
				{
					board[i][j] = POSS2;
				}
			}
			else
			{
				if(board[i][j] != CHANTILLY && board[i][j] != STRAWBERRY)
				{
					board[i][j] = EMPTY;
				}
			}
		}
	}
}

int listPossible(int playPoss) //Cria o array de jogadas possiveis
{
    int n = 0;
    int i, j;

    for (i = 0; i < SIZE; i++) //Percorre array na linha i
    {
        for (j = 0; j < SIZE; j++) //Percorre array na coluna j
        {
            if (board[i][j]==playPoss) //Se a posicao psssar na validatePosition
            {
                n++; //Incrementa o numero de posiçoes validas
            }
        }
    }
    return n;
}

void ARM()//ARM will play random using list of possible plays
{
	int playPoss;

	if(player == CHANTILLY)
	{
		playPoss = POSS1;
	}

	if(player == STRAWBERRY)
	{
		playPoss = POSS2;
	}

	int nPoss = listPossible(playPoss);
	int r = rand() % nPoss;
	int aux = 0;

	for(int i=0; i<SIZE; i++)
	{
		for(int j=0; j<SIZE; j++)
		{
			if(board[i][j] == playPoss)
			{
				if(aux == r)
				{
					board[i][j] = player;
					HAL_Delay(1000);
					colorPieces();
					HAL_Delay(1000);
					findPath(i,j);
					player = nextPlayer();
					return;
				}
				aux++;
			}
		}
	}
}

void score () //Extra Goal 13
{
	for (int i=0; i < SIZE; i++) //Look in all the board
	{
		for(int j = 0; j < SIZE; j++)
		{
			if(board[i][j] == CHANTILLY) //If find player 1 piece
			{
				counterPieces1++; //count player 1 pieces
			}
			else if(board[i][j] == STRAWBERRY) //If find player 1 piece
			{
				counterPieces2++; //count player 2 pieces
			}
		}
	}
	score1 = counterPieces1 * 10; //Multiply pieces by 10 to give score
	score2 = counterPieces2 * 10; //Multiply pieces by 10 to give score
}

void countPoss () //Count possible places
{
	for (int i=0; i < SIZE; i++)
	{
		for(int j = 0; j < SIZE; j++)
		{
			if(board[i][j] == POSS1)
			{
				counterPoss1++; //Count possible validate places for player 1
			}
			else if(board[i][j] == POSS2)
			{
				counterPoss2++; //Count possible validate places for player 2
			}
		}
	}
}

void winner_looser()//Give the winner
{
		if(score1 > score2)
		{
			BSP_LCD_SetFont(&Font16);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
			BSP_LCD_DisplayStringAt(BSP_LCD_GetYSize()+25, BSP_LCD_GetYSize()/10+300, (uint8_t *)"THE WINNER IS CHANTILLY", LEFT_MODE);
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		}

		else if(score1 < score2)
		{
			BSP_LCD_SetFont(&Font16);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
			BSP_LCD_DisplayStringAt(BSP_LCD_GetYSize()+25, BSP_LCD_GetYSize()/10+300, (uint8_t *)"THE WINNER IS STRAWBERRY", LEFT_MODE);
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		}

		else
		{
			BSP_LCD_SetFont(&Font16);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
			BSP_LCD_DisplayStringAt(BSP_LCD_GetYSize()+30, BSP_LCD_GetYSize()/10+300, (uint8_t *)"ITS A TIE: LOOOSEEERS!", LEFT_MODE);
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		}
}

//Write to SD Card
void writeSDcard()
{
	char stringA[100];
	char winner[20] = "";
	int score;

	if(score1 > score2)
	{
		strcpy(winner, "CHANTILLY");
		score = score1;
	}
	else
	{
		strcpy(winner, "STRAWBERRY");
		score = score2;
	}

	sprintf(stringA,"Winner: %s, Score: %d, Game Time: %d seconds\n", winner, score, counterT6);

	if(f_mount(&SDFatFS, SDPath, 0)!= FR_OK)
		Error_Handler();

	if(f_open(&SDFile, "testing.txt", FA_WRITE | FA_OPEN_ALWAYS) != FR_OK)
		Error_Handler();

	else
		BSP_LED_Toggle(LED_GREEN);

	if(f_write(&SDFile, stringA, strlen(stringA), &nBytes) != FR_OK)
		Error_Handler();

	else
	{
		BSP_LED_Toggle(LED_RED);
		HAL_Delay(250);
	}

	f_close(&SDFile);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)	//MAIN
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  
  /* Enable I-Cache---------------------------------------------------------*/
   SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_ADC1_Init();
  MX_DMA2D_Init();
  MX_DSIHOST_DSI_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_SDMMC2_SD_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);
  LCD_Config();
  BSP_TS_Init(BSP_LCD_GetXSize(),BSP_LCD_GetYSize()); //Configs Touch Screen
  BSP_TS_ITConfig(); //Touch screen interrupt
  HAL_ADC_Start_IT(&hadc1); //Start ADC interrupt
  //Start timers interrupt
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);

  flagMenu = 1; //Put Menu flag 0 = 1 to start the menu
  pf = 1;

  srand(time(NULL)); //to play against ARM

  /* OPEN/CREATE THE FILE MODE APPEND */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  temperature(); //should appear all the time

	  if(flagPB) //if Push Button pressed
	  {
		  flagPB = 0;
		  flagOnePlayer = 0;
		  flagTwoPlayers = 0;
		  flagMenu=1;
		  pf = 1;
		  gf = 0;
		  clearBoard(); //clean board to restart the game
		  LCD_Config();

		  //Extra Goal 11:
		    /*flagPB = 0;
		  	player = nextPlayer();
			BSP_LCD_FillRect(0, 45, 800, 400);
			flagMenu=0;
			initPositions();
			gameboard();
			gameInfo();
			findPossible();
			counterPieces1 = 0;
			counterPieces2 = 0;
			counterPoss1 = 0;
			counterPoss2 = 0;
			colorPieces();
			score();
			countPoss();
			counterMoves++;
			winner_looser();*/
	  }

	  if(flagMenu)
	  {
		  if(pf)
		  {
			  pf = 0;
			  startMenu();
		  }
		  if(flagTS)//if touch screen
		  {
			  	flagTS = 0;
				if (valueX  > 246 && valueX  < 554 && valueY  >= 106 && valueY <= 189) //check rectangle position One Player
				{
					flagTS = 0;
					flagOnePlayer = 1;
					BSP_LCD_FillRect(0, 45, 800, 400);
					flagMenu=0;
					initPositions();
					gameboard();
					gameInfo();
					counterT2 = 20;
					counterT6 = 0;
					findPossible();
					counterPieces1 = 0;
					counterPieces2 = 0;
					counterPoss1 = 0;
					counterPoss2 = 0;
					colorPieces();
					score();
					countPoss();
					counterMoves++;
					winner_looser();
				}
				else if (valueX  > 246 && valueX  < 554 && valueY >= 291 && valueY <= 374)//check rectangle position Two Players
				{
					flagTS = 0;
					flagTwoPlayers = 1;
					BSP_LCD_FillRect(0, 45, 800, 400);
					flagMenu=0;
					initPositions();
					gameboard();
					gameInfo();
					counterT2 = 20;
					counterT6 = 0;
					findPossible();
					counterPieces1 = 0;
					counterPieces2 = 0;
					counterPoss1 = 0;
					counterPoss2 = 0;
					colorPieces();
					countPoss();
					score();
					counterMoves++;
					winner_looser();
				}
		  }
	  }

	  else if(flagOnePlayer)
	  {
		  gameTime();
		  playerTime();

		  if (detectTS())
		  {
			  counterPieces1 = 0;
			  counterPieces2 = 0;
			  counterPoss1 = 0;
			  counterPoss2 = 0;
			  gameboard();
			  score();
			  countPoss();
			  gameInfo();
			  findPossible();
			  colorPieces();
			  counterMoves++;
			  winner_looser();
			  ARM();
			  counterPieces1 = 0;
			  counterPieces2 = 0;
			  counterPoss1 = 0;
			  counterPoss2 = 0;
			  gameboard();
			  score();
			  countPoss();
			  gameInfo();
			  findPossible();
			  colorPieces();
			  counterMoves++;
			  winner_looser();
		  }
	  }

	  else if(flagTwoPlayers)
	  {
		  gameTime();
		  playerTime();

		  if (detectTS())
		  {
			  counterPieces1 = 0;
			  counterPieces2 = 0;
			  counterPoss1 = 0;
			  counterPoss2 = 0;
			  gameboard();
			  score();
			  countPoss();
			  gameInfo();
			  findPossible();
			  colorPieces();
			  counterMoves++;
			  winner_looser();

			  /*if(counterMoves >= 1)
			  {
				  HAL_ADC_Stop_IT(&hadc1);
				  writeSDcard();
				  HAL_ADC_Start_IT(&hadc1);
			  }*/
		  }
	  }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_SDMMC2
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  PeriphClkInitStruct.Sdmmc2ClockSelection = RCC_SDMMC2CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;
  hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief DSIHOST Initialization Function
  * @param None
  * @retval None
  */
static void MX_DSIHOST_DSI_Init(void)
{

  /* USER CODE BEGIN DSIHOST_Init 0 */

  /* USER CODE END DSIHOST_Init 0 */

  DSI_PLLInitTypeDef PLLInit = {0};
  DSI_HOST_TimeoutTypeDef HostTimeouts = {0};
  DSI_PHY_TimerTypeDef PhyTimings = {0};
  DSI_LPCmdTypeDef LPCmd = {0};
  DSI_CmdCfgTypeDef CmdCfg = {0};

  /* USER CODE BEGIN DSIHOST_Init 1 */

  /* USER CODE END DSIHOST_Init 1 */
  hdsi.Instance = DSI;
  hdsi.Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
  hdsi.Init.TXEscapeCkdiv = 4;
  hdsi.Init.NumberOfLanes = DSI_ONE_DATA_LANE;
  PLLInit.PLLNDIV = 20;
  PLLInit.PLLIDF = DSI_PLL_IN_DIV1;
  PLLInit.PLLODF = DSI_PLL_OUT_DIV1;
  if (HAL_DSI_Init(&hdsi, &PLLInit) != HAL_OK)
  {
    Error_Handler();
  }
  HostTimeouts.TimeoutCkdiv = 1;
  HostTimeouts.HighSpeedTransmissionTimeout = 0;
  HostTimeouts.LowPowerReceptionTimeout = 0;
  HostTimeouts.HighSpeedReadTimeout = 0;
  HostTimeouts.LowPowerReadTimeout = 0;
  HostTimeouts.HighSpeedWriteTimeout = 0;
  HostTimeouts.HighSpeedWritePrespMode = DSI_HS_PM_DISABLE;
  HostTimeouts.LowPowerWriteTimeout = 0;
  HostTimeouts.BTATimeout = 0;
  if (HAL_DSI_ConfigHostTimeouts(&hdsi, &HostTimeouts) != HAL_OK)
  {
    Error_Handler();
  }
  PhyTimings.ClockLaneHS2LPTime = 28;
  PhyTimings.ClockLaneLP2HSTime = 33;
  PhyTimings.DataLaneHS2LPTime = 15;
  PhyTimings.DataLaneLP2HSTime = 25;
  PhyTimings.DataLaneMaxReadTime = 0;
  PhyTimings.StopWaitTime = 0;
  if (HAL_DSI_ConfigPhyTimer(&hdsi, &PhyTimings) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigFlowControl(&hdsi, DSI_FLOW_CONTROL_BTA) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetLowPowerRXFilter(&hdsi, 10000) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigErrorMonitor(&hdsi, HAL_DSI_ERROR_NONE) != HAL_OK)
  {
    Error_Handler();
  }
  LPCmd.LPGenShortWriteNoP = DSI_LP_GSW0P_DISABLE;
  LPCmd.LPGenShortWriteOneP = DSI_LP_GSW1P_DISABLE;
  LPCmd.LPGenShortWriteTwoP = DSI_LP_GSW2P_DISABLE;
  LPCmd.LPGenShortReadNoP = DSI_LP_GSR0P_DISABLE;
  LPCmd.LPGenShortReadOneP = DSI_LP_GSR1P_DISABLE;
  LPCmd.LPGenShortReadTwoP = DSI_LP_GSR2P_DISABLE;
  LPCmd.LPGenLongWrite = DSI_LP_GLW_DISABLE;
  LPCmd.LPDcsShortWriteNoP = DSI_LP_DSW0P_DISABLE;
  LPCmd.LPDcsShortWriteOneP = DSI_LP_DSW1P_DISABLE;
  LPCmd.LPDcsShortReadNoP = DSI_LP_DSR0P_DISABLE;
  LPCmd.LPDcsLongWrite = DSI_LP_DLW_DISABLE;
  LPCmd.LPMaxReadPacket = DSI_LP_MRDP_DISABLE;
  LPCmd.AcknowledgeRequest = DSI_ACKNOWLEDGE_DISABLE;
  if (HAL_DSI_ConfigCommand(&hdsi, &LPCmd) != HAL_OK)
  {
    Error_Handler();
  }
  CmdCfg.VirtualChannelID = 0;
  CmdCfg.ColorCoding = DSI_RGB888;
  CmdCfg.CommandSize = 640;
  CmdCfg.TearingEffectSource = DSI_TE_EXTERNAL;
  CmdCfg.TearingEffectPolarity = DSI_TE_RISING_EDGE;
  CmdCfg.HSPolarity = DSI_HSYNC_ACTIVE_LOW;
  CmdCfg.VSPolarity = DSI_VSYNC_ACTIVE_LOW;
  CmdCfg.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
  CmdCfg.VSyncPol = DSI_VSYNC_FALLING;
  CmdCfg.AutomaticRefresh = DSI_AR_ENABLE;
  CmdCfg.TEAcknowledgeRequest = DSI_TE_ACKNOWLEDGE_DISABLE;
  if (HAL_DSI_ConfigAdaptedCommandMode(&hdsi, &CmdCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetGenericVCID(&hdsi, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DSIHOST_Init 2 */

  /* USER CODE END DSIHOST_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 7;
  hltdc.Init.VerticalSync = 3;
  hltdc.Init.AccumulatedHBP = 14;
  hltdc.Init.AccumulatedVBP = 5;
  hltdc.Init.AccumulatedActiveW = 654;
  hltdc.Init.AccumulatedActiveH = 485;
  hltdc.Init.TotalWidth = 660;
  hltdc.Init.TotalHeigh = 487;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief SDMMC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC2_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC2_Init 0 */

  /* USER CODE END SDMMC2_Init 0 */

  /* USER CODE BEGIN SDMMC2_Init 1 */

  /* USER CODE END SDMMC2_Init 1 */
  hsd2.Instance = SDMMC2;
  hsd2.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd2.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd2.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd2.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd2.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd2.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC2_Init 2 */

  /* USER CODE END SDMMC2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 9999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 9999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 19999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_13;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_32;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /*Configure GPIO pin : PI13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : PI15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
static void LCD_Config(void)
{
  uint32_t  lcd_status = LCD_OK;

  /* Initialize the LCD */
  lcd_status = BSP_LCD_Init();
  while(lcd_status != LCD_OK);

  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);

  /* Clear the LCD */
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_DrawBitmap(0, 0, (uint8_t*) stlogo);
  /* Set LCD Example description */
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE); //Rodapé
  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()- 20, (uint8_t *)"Copyright (c) Oak Tree Company", CENTER_MODE);

  BSP_LCD_SetTextColor(LCD_COLOR_LIGHTRED); //Header
  BSP_LCD_FillRect(0, 0, BSP_LCD_GetXSize(), 45);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_LIGHTRED);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(0, 12, (uint8_t *)"WELCOME TO REVERSI", CENTER_MODE);
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
    while(1)
    {
        BSP_LED_Toggle(LED_GREEN);
        HAL_Delay(250);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
