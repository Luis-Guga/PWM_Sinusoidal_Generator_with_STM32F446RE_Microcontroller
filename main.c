/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file main.c
 * @author Luís Gustavo Lauermann Hartmann e Pedro Lumazsewski Lima
 * Desenvolvido em novembro de 2021.
 * Projeto: Inversor de frequência monofásico.
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



/** Definindo os seguintes parâmetros:
 *  > Frequência do clock do micro (para cálculos posteriores)
 * 	> Número de PI (para calcular os valores de seno)
 * 	> Fator de multiplicação (será explicado mais abaixo)
 * 	> Frequências máximas e mínimas da modulante
 * 	> Números máximos e mínimos de pontos de seno
 */


#define CLOCK_FREQ 170000000 // Frequência do clock
#define PI 3.14159265358979323846264 // Número de PI
#define FACTOR 10000.0 // Fator de multiplicação
#define MAX_FREQ 70 // Frequência máxima da modulante
#define MIN_FREQ 50 // Frequência mínima da modulante
#define MAX_POINTS 1000 // Número máximo de pontos
#define MIN_POINTS 50 // Número mínimo de pontos







/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/** Variáveis do programa:
 *  > serial_receive - caracteres recebidos pela serial que serão convertidos para números
 *
 *  > msg - mensagens enviadas pela serial para a comunicação com o usuário
 *
 *  > config_mode - primeiro o usuário seleciona a opção do que mudar (pontos ou frequência),
 *  depois ele CONFIGURA, fazendo o algoritmo entrar no modo configuração.
 *
 *  > points_period - variável que armazena o número de pontos de seno que serão armazenados
 *
 *  > sin_feq - variável que armazena a frequência da modulante
 *
 *  > points - ponteiro que apontará para o bloco de memória onde serão armazenados os
 *  valores de seno.
 *
 *  > CH1_Works - variável que indica qual dos dois canais PWM trabalhará, caso o canal PWM 1
 *  esteja trabalhando, o PWM 2 necessariamente não trabalhará até que o canal PWM 1
 *  tenha terminado (Serve para evitar o curto-circuito).
 *
 *  > i - variável que indica o índice do bloco de memória, indica qual é o valor de
 *  seno que valerá como ciclo de trabalho a cada geração de pulso.
 */

uint8_t serial_receive[5];
char msg[100];
uint8_t config_mode=0;
uint16_t points_period=MIN_POINTS;
uint8_t sin_freq = MIN_FREQ;
uint16_t *points;
uint8_t CH1_Works=1;
uint16_t i=0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM8_Init(void);

/* USER CODE BEGIN PFP */

void adapt_PWM_freq(TIM_HandleTypeDef *htim, uint16_t total_points, uint16_t freq);
void trade_char(char *str, char target, char substitute);
uint16_t* sin_gen(uint16_t total_points);

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
	MX_USART2_UART_Init();
	MX_TIM8_Init();
	/* USER CODE BEGIN 2 */


	/** Inicialização dos periféricos
	 * > Inicializando o canal 1 e 2 de PWM
	 * > Iniciando a comunicação com o usuário
	 * > Instancia os pontos de seno na memória e aponta o ponteiro para o
	 * endereço inicial
	 * > Adapta a frequência de chaveamento de acordo com o número de
	 * pontos e da frequência da modulante.
	 */

	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);

	points = sin_gen(points_period);
	adapt_PWM_freq(&htim8, points_period, sin_freq);

	strcpy(msg,"Mudar pontos por periodo (1)\n\rMudar frequencia da modulante (2)\n\r");
	HAL_UART_Transmit_IT(&huart2,(uint8_t*) msg, strlen(msg));
	HAL_UART_Receive_IT(&huart2, &serial_receive[0], 1);
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
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 170;
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
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 169;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = ARR_5kHz;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */
	HAL_TIM_MspPostInit(&htim8);

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
	huart2.Init.BaudRate = 115200;
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

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**********************************************************************
 	 	 	 	 	 	 	 USER FUNCTIONS
 **********************************************************************/
/** Função trade_char
 *  Essa função serve para que a comunicação serial possa ocorrer de modo
 *  compreensível.
 *  Sempre que o usuário selecionar uma das duas opções iniciais
 *  -> Mudar pontos por periodo (1)
 *	-> Mudar frequencia da modulante (2)
 *	o programa entrará em modo configuração.
 *	Entretanto, como os dados enviados por comunicação serial são CARACTERES,
 *  é necessário que se possa identificar QUANDO um NÚMERO EM CARACTERE termina.
 *  Um exemplo para essa situação é o seguinte:
 *  -> Mudar pontos por periodo (1)
 *	-> Mudar frequencia da modulante (2)
 *
 *	O usuário escolhe 2 (o programa converte '2' em caractere para 2 em numérico)
 *
 *  ->Determine a frequencia da onda modulante (de 50 a 70Hz):
 *
 *  O usuário digita 65
 *  Como o programa reconhecerá que os caracteres '6' e '5' representam o
 *  número 65?
 *
 *  Por isso, foi definido no nosso parâmetro de comunicação que, sempre que
 *  o usuário desejar mudar a frequência ou o número de pontos, ao iniciar a
 *  configuração de pontos ou frequência, o usuário deve FINALIZAR o VALOR
 *  DESEJADO com um PONTO.
 *
 *  Dessa forma, o usuário deveria digitar 65 (sessenta e cindo PONTO).
 *
 *	Esse é o objetivo dessa função, juntar os caracteres '6' e '5' em "65".
 *	Depois de juntá-los, poderemos utilizar a função atoi() da "stdlib.h" para
 *	transformar uma STRING em um NÚMERO, efetivamente.
 *
 *	Parâmetros:
 *	*str aponta para o endereço inicial da string que deve ser convertida em número.
 *	target é o caractere que deve ser substituído por substitute,
 *	substitute é o caractere que substituirá o caractere alvo.
 *
 *	Nesse caso, utilizamos o PONTO como alvo e o finalizador de strings (\0), como substituto.
 */
void trade_char(char *str, char target, char substitute){
	for(uint8_t i=0; str[i]!='\0'; i++) // Procura pelo caractere alvo.
		if(str[i] == target){ // Se o caractere alvo for encontrado...
			str[i] = substitute; // Substitui pelo caractere sustituto
			return;
		}
}

/** Função adapt_PWM_freq
 * Como já foi citado, é necessário reajustar a frequência do timer sempre que
 * houver uma mudança de pontos de seno ou uma mudança na frequência da onda
 * fundamental (a senoide).
 * Essa função é encarregada disso.
 *
 * Parâmetros:
 * *htim estrutura do timer utilizado para gerar os pulsos PWM.
 * total_points número de pontos de seno a serem gerados.
 * freq frequência da onda fundamental utilizada.
 */
void adapt_PWM_freq(TIM_HandleTypeDef *htim, uint16_t total_points, uint16_t freq){
	freq*=2;
	uint16_t PSC=CLOCK_FREQ/1000000;
	uint16_t ARR = CLOCK_FREQ/(PSC*freq*total_points);
	if(ARR>65530){
		PSC = 65530;
		ARR = (CLOCK_FREQ/(PSC*freq));
	}else if(ARR<1){
		PSC = 1;
		ARR = (CLOCK_FREQ/(PSC*freq));
	}

	__HAL_TIM_SET_PRESCALER(htim, PSC-1);
	__HAL_TIM_SET_AUTORELOAD(htim, ARR-1);
}

/** Função sin_gen
 *  Essa é a função vital para o projeto.
 *  Ela é encarregada de gerar os valores seno e armazená-los na memória.
 *  Depois de armazenar os valores de seno na memória, ela indica onde está
 *  o início do bloco de memória utilizado, para que o ponteiro *points citado anteriormente
 *  possa percorrer o bloco de memória e mudar os valores de ciclod e trabalho do pulso PWM.
 *
 *  Parâmetros:
 *  total_points número de pontos de seno
 *
 *  Retorno:
 *  retorna o endereço inicial do bloco de memória onde estão os valores de seno.
 *
 *  Método: para evitar a utilização de um vetor de tipo float ou double (pois ocuparia espaço
 *  demais), utilizamos o número definido como FACTOR (10.000) que nos permite armazenar os
 *  valores de seno sem utilizar casas decimais. Basta-nos, ao mudarmos o ciclo de trabalho,
 *  apenas dividir o número da multiplicação por FACTOR novamente.
 *
 *	Exemplo:
 *	Em valor calculado de seno é de 0.0035.
 *
 *	Então, para evitar de armazená-lo em formato float ou double, oq eu ocuparia de 8 a 16 bytes,
 *	armazenamo-lo com uma multiplicação por 10000, o que apenas nos custa 4 bytes de memória e
 *	não altera sua integridade.
 *
 *	O número "viraria" 35.
 */
uint16_t* sin_gen(uint16_t total_points){
	uint16_t *out = malloc(total_points*sizeof(uint16_t));

	for(int i=0; i<total_points; i++){
		out[i] = FACTOR*((double)sin(i*PI/total_points));
	}
	return out;
}
/**********************************************************************
 	 	 	 	 	 	 	 CALLBACKS
 **********************************************************************/

/** Função de interrupção da recepção serial
 * Esse método é responsável pela configuração da onda, tanto pelo número de pontos de seno
 * como pela mudança do valor da frequência da onda fundamental.
 * Para entender a função corretamente, pule para a PARTE 1, e, depois, vá para a PARTE 2.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART2)
	{
		// ========= PARTE 2 - CONFIGURAÇÂO ========== //
		/* Caso o modo de CONFIGURAÇÃO TENHA SIDO ACIONADO, a pessoa terá de digitar
		 * ou o valor de frequência da onda fundamental, caso tenha escolhido a opção
		 * 2, ou o número de pontos por período, caso tenha selecioado a opção 1.
		 *
		 * Para as duas opções, os processos são simmilares:
		 *  1. Transformar os caracteres em valores numéricos
		 *  2. Verificar se o número está dentro dos limites permitidos pelas definições
		 *  iniciais
		 *  3. Adaptar a frequência do PWM de acordo com o valor digitado, e gerar novos
		 *  pontos de seno (caso a pessoa tenha escolhido a primeira opção)
		 *  4. Comunicar que a configuração foi efetuada
		 *  5. Desligar o modo configuração
		 *  6. Zerar a primeira opção do vetor de recepção de informações.
		 */

		if(config_mode && serial_receive[0] == '1')
		{
			// processo número 1.
			trade_char((char*)&serial_receive[1],'.', '\0');
			points_period = atoi((char*)&serial_receive[1]);

			// processo número 2.
			if(points_period>MAX_POINTS) points_period = MAX_POINTS;

			else if(points_period<MIN_POINTS) points_period = MIN_POINTS;

			// processo número 3.
			points = sin_gen(points_period);
			// processo número 3.
			adapt_PWM_freq(&htim8, points_period, sin_freq);

			// processo número 4.
			strcpy(msg, "Feito.\n\n\rMudar pontos por periodo (1)\n\rMudar frequencia da modulante (2)\n\r");
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)&msg[0], strlen(msg));

			// processo número 5.
			serial_receive[0]=0;
			config_mode=0;
		} // end if

		if(config_mode && serial_receive[0] == '2')
		{
			// processo número 1.
			trade_char((char*)&serial_receive[1],'.', '\0');
			sin_freq = atoi((char*)&serial_receive[1]);
			// processo número 2.
			if(sin_freq>MAX_FREQ) sin_freq = MAX_FREQ;

			else if(sin_freq<MIN_FREQ) sin_freq = MIN_FREQ;

			// processo número 3.
			adapt_PWM_freq(&htim8, points_period, sin_freq);

			// processo número 4.
			strcpy(msg, "Feito.\n\n\rMudar pontos por periodo (1)\n\rMudar frequencia da modulante (2)\n\r");
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)&msg[0], strlen(msg));

			// processo número 5.
			serial_receive[0]=0;
			config_mode=0;
		}// end if

		// ========= FIM DA PARTE 2  ========== //

		// ========= PARTE 1 - SELEÇÃO DA OPÇÃO DESEJADA ======== //
		/* Quando recebe o primeiro caractere da serial, verifica qual das opções foi
		 * escolhida.
		 *
		 * > Se for a opção 1, fala que o número de pontos vai de 50 a 1000.
		 * > Se for a 2, fala que as frequências variam de MAX_FREQ a MIN_FREQ.
		 *
		 * Após QUALQUER UMA DAS OPÇÕES, ACIONA O MODO DE CONFIGURAÇÃO.
		 * Veja a PARTE 3.
		 */
		switch (serial_receive[0]){
		case '1':
			strcpy(msg, "Determine o numero de pontos por periodo (de 50 a 1000 pontos):\n\r");
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)&msg[0], strlen(msg));

			config_mode=1;
			break;

		case '2':
			strcpy(msg, "Determine a frequencia da onda modulante (de 50 a 70Hz):\n\r");
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)&msg[0], strlen(msg));

			config_mode=1;
			break;
		} // END switch
		// ========= FIM PARTE 1 ======== //

		// ======== PARTE 3 - DETERMINAÇÃO DO QUE SERÁ RECEBIDDO NA PRÓXIMA COMUNICAÇÃO ==== //
		/* Caso a pessoa tenha escolhido uma das opções, receberá o valor
		 * que ela colocará na próxima comunicação (lembre-se, o valor deve ser terminado com PONTO.
		 *
		 * Caso a pessoa já tenha digitado o valor, reinicia a comunicação, permitindo que a pessoa
		 * escolha novamente uma das duas opções:
		 * -> Mudar pontos por periodo (1)
		 * -> Mudar frequencia da modulante (2)
		 *
		 *	Leia a parte 2.
		 */
		if(config_mode)
			HAL_UART_Receive_IT(&huart2, &serial_receive[1], 4);
		else
			HAL_UART_Receive_IT(&huart2, &serial_receive[0], 1);

		// ========= FIM PARTE 3 ======== //
	}
}

/** Função de interrupção por finalização de pulso PWM
 * Nessa última função, é onde toda a "mágica" acontece.
 * Sempre que o PWM finalizar o pulso, um novo Pulse é calculado por meio do
 * valor senoidal.
 * Para que um novo valor senoidal seja calculado, basta-nos utilizar o bloco de
 * memória com os valores de seno já armazenados e percorrer esse bloco.
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){

	if(htim->Instance==TIM8 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 && CH1_Works){


		if(i>(points_period-1)){ // Assim que o seno acabar, é hora do outro canal trabalhar.
			i=0; // Reinicia o índice para o bloco poder ser usado no outro canal.
			CH1_Works=0; // O outro canal trabalha, esse canal é impedido, por consequência.
		}


		// Faz o cálculo do próximo valor de pulse a ser configurado no periférico do TIMER.
		uint32_t compare = (uint32_t) ((htim->Init.Period)*(double)(points[i]/FACTOR));


		/* Configura o valor de compare para que o próximo pulso tenha sua largura modificada
		 * de acordo com o comportamento de uma senoide.
		 */
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1, compare);
		i++; // Aumenta o índice do bloco e "busca" o próximo valor de seno.
	}
	if(htim->Instance==TIM8 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2 && !(CH1_Works)){
		if(i>(points_period-1)){
			i=0;
			CH1_Works=1;
		}
		uint32_t compare = (uint32_t) ((htim->Init.Period)*(double)(points[i]/FACTOR));
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2, compare);
		i++;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
