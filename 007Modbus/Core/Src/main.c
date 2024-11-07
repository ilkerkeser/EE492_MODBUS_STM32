/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "wizchip_conf.h"
#include "socket.h"
#include "loopback.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_OF_REGS 125
#define NUM_OF_COILS  20
#define MBAP_SIZE 7


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define elemansayisi_array(arr)   (sizeof(arr)/sizeof(arr[0]))
#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

uint16_t REG[NUM_OF_REGS];

/* USER CODE BEGIN PV */
// MAC adresi (denetim masasındaki fiziksel adres)
uint8_t mac[6] = {0xB0, 0x34, 0xCB, 0x3E, 0x31, 0xDB};

// IP adresi (benzersiz bir adres)
uint8_t ip[4] = {192, 168, 0, 10};

// Ağ maskesi ve varsayılan ağ geçidi
uint8_t sn[4] = {255, 255, 255, 0};
uint8_t gw[4] = {192, 168, 0, 1};

uint8_t receive_message[12]; //Receive message

const char transmit_message[] = "Hello from W5500 server!";


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
/*Modbus TCP/IP functions*/
uint8_t *READ_COILS(uint8_t function_code,uint8_t start_address, uint8_t quantity_of_inputs);
uint8_t *READ_DISCRETE_INPUT(uint8_t function_code, uint8_t quantity_of_inputs);
uint8_t *READ_H_REGS( uint8_t function_code, uint8_t start_address, uint8_t quantity_of_inputs);
uint8_t *READ_IN_REGS(uint8_t function_code, uint8_t start_address, uint8_t quantity_of_inputs);
uint8_t *WRITE_SINGLE_COIL(uint8_t function_code,uint8_t output_address,uint16_t output_value);
uint8_t *WRITE_SINGLE_REG(uint8_t function_code,uint8_t start_address,uint8_t output_value);
/*Modbus TCP/IP functions end*/

/*Other Function */
unsigned int arrayToHex(int a[], int length);
uint8_t hexToDecimal(uint8_t hex);
char* decimalToHex(unsigned int decimal);
uint8_t *Dec2Hex(uint16_t c);
uint8_t *COMBINE_MBAP_PDU(uint8_t *MBAP, uint8_t *PDU,int size_MBAP,int size_PDU);
/*Other Function End*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay(void){
	for(uint32_t i=0;i<250000;i++);
}
void cs_sel() {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET); //CS LOW
}

void cs_desel() {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET); //CS HIGH
}

void W5500_ReadBuff(uint8_t *buffer, uint16_t len){
	HAL_SPI_Receive(&hspi1, buffer, len, HAL_MAX_DELAY);
}

void W5500_WriteBuff(uint8_t *buffer, uint16_t len){
	HAL_SPI_Transmit(&hspi1, buffer, len, HAL_MAX_DELAY);
}


uint8_t W5500_ReadByte(void) {
    uint8_t byte;
    W5500_ReadBuff(&byte, sizeof(byte));
    return byte;
}

void W5500_WriteByte(uint8_t byte) {
    W5500_WriteBuff(&byte, sizeof(byte));
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  REG[100] = GPIO_PIN_SET; //Dijital çıkış için kullanılacak
  REG[101] = GPIO_PIN_RESET; //Dijital çıkış için kullanılacak



  wizchip_init(0, 0);
  reg_wizchip_cs_cbfunc(cs_sel, cs_desel);
  reg_wizchip_spi_cbfunc(W5500_ReadByte,W5500_WriteByte);

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // Ağ bilgilerini yapılandırma
  wiz_NetInfo netInfo = {
    .mac = {mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]},
    .ip = {ip[0], ip[1], ip[2], ip[3]},
    .sn = {sn[0], sn[1], sn[2], sn[3]},
    .gw = {gw[0], gw[1], gw[2], gw[3]}
  };

  wizchip_setnetinfo(&netInfo);

  // Ayarlanan ağ bilgilerini kontrol edin
  wizchip_getnetinfo(&netInfo);

  // Soket başlatma
  uint8_t socketNumber = 0;
  socket(socketNumber, Sn_MR_TCP, 502, 0);

  if (listen(socketNumber) == SOCK_OK){
	  printf("Listening on port 502\n");
  }
  else{
	  printf("Not connected\n");
  }

  /* USER CODE END 2 */
  //send(socketNumber, (uint8_t *)transmit_message, sizeof(transmit_message));||=> bu bizim göndereceğimiz mesaj formatı

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    /* USER CODE END WHILE */
	 uint8_t status = getSn_SR(socketNumber);
	 if(status == SOCK_ESTABLISHED){
		 //recv(socketNumber, (uint8_t *)receive_message, elemansayisi_array(receive_message));
		recv(socketNumber, (uint8_t *)receive_message, 12);
		unsigned short transaction_id = (receive_message[0] << 8) | receive_message[1];
		unsigned short protocol_id = (receive_message[2] << 8) | receive_message[3];
		unsigned short t_length = (receive_message[4]<<8) | receive_message[5];

		uint8_t unit_id = receive_message[6];
		uint8_t function_code = receive_message[7];
	    uint8_t MBAP[7];

	    MBAP[0] = highByte(transaction_id);
	    MBAP[1] = lowByte(transaction_id);
	    MBAP[2] = highByte(protocol_id);
	    MBAP[3] = lowByte(protocol_id);

	    /*MBAP[4] = highByte(response_t_length_f3_f4); \
	                                                    --------> Bu iki byte sadece Function3 ve Function4 için geçerli,
	                                                    --------> o yüzden bu iki byte ilgili fonksiyonlarda tanımlanmışlardır.
	    MBAP[5] = lowByte(response_t_length_f3_f4);  */
	    MBAP[6] = unit_id;

	    if(unit_id ==1){
	    	if(function_code == 1){
	            unsigned short start_address = (receive_message[8] << 8)| receive_message[9];    //Coils de Output Address olarak alınacak.
	            unsigned short quantity_of_inputs = (receive_message[10] << 8)| receive_message[11];
	            uint8_t response_pdu_length;
	            uint8_t byte_count;
	            //MBAP mesaj uzunluğunu bulmak için quantity of inputs sayısına bakılmalıdır.
	            //Fonksiyondaki mantık burada da yapılmıştır.
	            if(quantity_of_inputs >8){
	              byte_count = quantity_of_inputs /8;
	              if(quantity_of_inputs %8 != 0){
	                byte_count++;
	              }
	            }else{
	              byte_count = 1;
	            }
	            //function code(1 byte) + byte count (1 byte)
	            response_pdu_length = byte_count + 3;
	            MBAP[4] = highByte(response_pdu_length);
	            MBAP[5] = lowByte(response_pdu_length);
	            //Size RES_PDU boyutunu tanımla
	            int size_RES_PDU = byte_count + 2;

	            //Fonksiyonu çağır ve geçici olarak PDU temp pointera ata (dönen değerlerin başlangıç adresi)

	            uint8_t *RES_PDU_TEMP = READ_COILS(function_code,start_address,quantity_of_inputs);
	            //PDU arrayini olustur
	            uint8_t RES_PDU_1[size_RES_PDU];

	            //Temp pointerindaki değerleri PDU_1 e al
	            for(int i=0;i<size_RES_PDU;i++){
	              RES_PDU_1[i] = *(RES_PDU_TEMP +i);
	            }

	            //Temp PDU pointeri boşalt
	            free(RES_PDU_TEMP);

	            for(int i =0;i<size_RES_PDU;i++){
	            	printf("/x%x ",RES_PDU_1[i]);
	            }

	            printf("\n");


	            //Mesaj uzunluğunu belirle
	            int size_all = elemansayisi_array(MBAP) + elemansayisi_array(RES_PDU_1);


	            /*printf("Size all: %d\n",size_all);
	             printf("Size MBAP: %d\n",elemansayisi_array(MBAP));
	             printf("Size PDU: %d\n",elemansayisi_array(RES_PDU_1))*/

	            //Tek framede birleştirdiktan sonra geçici olarak bit temp pointera ata
	            uint8_t *COMPLETE_FRAME_TEMP = COMBINE_MBAP_PDU(MBAP,RES_PDU_1,elemansayisi_array(MBAP),elemansayisi_array(RES_PDU_1));

	            //Complete frame oluştur
	            uint8_t COMPLETE_FRAME[size_all];

	            //Tempdeki değerleri Complete Frame e al
	            for(int i=0;i<size_all;i++){
	              COMPLETE_FRAME[i] = *(COMPLETE_FRAME_TEMP+i);
	            }

	            //Temp pointeri boşalt
	            free(COMPLETE_FRAME_TEMP);

	            printf("COMPLETE FRAME: \n");

	            for(int i=0;i<size_all;i++){
	            	printf("/x%x ",COMPLETE_FRAME[i]);
	            }

	            printf("\n");

	            send(socketNumber, (uint8_t *)COMPLETE_FRAME, elemansayisi_array(COMPLETE_FRAME));

	    	}else if(function_code == 2){
	    		//unsigned short start_address = (receive_message[8]<<8) | receive_message[9];
	    		unsigned short quantity_of_inputs = (receive_message[10] << 8) | receive_message[11];
	    		uint8_t response_pdu_length;
	    		uint8_t byte_count;

	    		if(quantity_of_inputs >8){
	    			byte_count = quantity_of_inputs / 8;
	    			if(quantity_of_inputs % 8 != 0){
	    				byte_count++;
	    			}
	    		}else{
	    			byte_count =1;
	    		}

	    		//function code(1 byte) + byte count (1 byte)
	    		response_pdu_length = byte_count+3;

	    		MBAP[4] = highByte(response_pdu_length);
	    		MBAP[5] = lowByte(response_pdu_length);

	            //Size RES_PDU boyutunu tanımla
	            int size_RES_PDU = byte_count + 2;

	            uint8_t *RES_PDU_TEMP = READ_DISCRETE_INPUT(function_code,quantity_of_inputs);

	            //PDU arrayini olustur
	            uint8_t RES_PDU_2[size_RES_PDU];


	            //Temp pointerindaki değerleri PDU_1 e al
	            for(int i=0;i<size_RES_PDU;i++){
	              RES_PDU_2[i] = *(RES_PDU_TEMP +i);
	            }

	            //Temp PDU pointeri boşalt
	            free(RES_PDU_TEMP);

	            for(int i=0;i<size_RES_PDU;i++){
	            	printf("/x%x ",RES_PDU_2[i]);
	            }

	            printf("\n");

	            //Mesaj uzunluğunu belirle
	            int size_all = elemansayisi_array(MBAP) + elemansayisi_array(RES_PDU_2);

	            printf("Size all: %d\n",size_all);
	            printf("Size MBAP = %d\n",elemansayisi_array(MBAP));
	            printf("Size PDU = %d\n",elemansayisi_array(RES_PDU_2));

	            //Tek framede birleştirdiktan sonra geçici olarak bit temp pointera ata
	            uint8_t *COMPLETE_FRAME_TEMP = COMBINE_MBAP_PDU(MBAP,RES_PDU_2,elemansayisi_array(MBAP),elemansayisi_array(RES_PDU_2));

	            //Complete frame oluştur
	            uint8_t COMPLETE_FRAME[size_all];

	            //Tempdeki değerleri Complete Frame e al
				for(int i=0;i<size_all;i++){
				  COMPLETE_FRAME[i] = *(COMPLETE_FRAME_TEMP+i);
				}
				printf("\n");

				//Temp pointeri boşalt
				free(COMPLETE_FRAME_TEMP);

				printf("-----COMPLETE FRAME-------------\n");

				for(int i = 0;i<size_all;i++){
					printf("/x%x ",COMPLETE_FRAME[i]);
				}

				printf("\n");

	            send(socketNumber, (uint8_t *)COMPLETE_FRAME, elemansayisi_array(COMPLETE_FRAME));//end function 2
	    	}else if(function_code == 3){
	            unsigned short start_address = (receive_message[8] << 8)| receive_message[9];               //Coils de Output Address olarak alınacak.
	            unsigned short quantity_of_inputs = (receive_message[10] << 8)| receive_message[11];
	            unsigned short response_t_length_f3_f4 = 1 + 2 + 2*quantity_of_inputs;
	            MBAP[4] = highByte(response_t_length_f3_f4);
	            MBAP[5] = lowByte(response_t_length_f3_f4);
	            int size_RES_PDU = 2*quantity_of_inputs + 1 + 1;
	            //Fonksiyonu çağır ve geçici olarak PDU temp pointera ata
	            uint8_t *RES_PDU_TEMP = READ_H_REGS(function_code, start_address, quantity_of_inputs);
	            //PDU arrayini oluştur.
	            uint8_t RES_PDU_3[size_RES_PDU];

	            for(int i=0;i<size_RES_PDU;i++){
	                RES_PDU_3[i] = *(RES_PDU_TEMP +i);
	            }

	            //Temp pointeri boşalt
	            free(RES_PDU_TEMP);

	            //Response mesaj uzunluğunu belirle
	            int size_all = elemansayisi_array(MBAP) + elemansayisi_array(RES_PDU_3);

	            printf("Size all: %d\n",size_all);
	            printf("Size MBAP: %d\n",elemansayisi_array(MBAP));
	            printf("Size PDU: %d\n",elemansayisi_array(RES_PDU_3));

	            //Tek framede birleştirdiktan sonra geçici olarak bit temp pointera ata
	            uint8_t *COMPLETE_FRAME_TEMP = COMBINE_MBAP_PDU(MBAP,RES_PDU_3,elemansayisi_array(MBAP),elemansayisi_array(RES_PDU_3));

	            //Complete frame oluştur
	            uint8_t COMPLETE_FRAME[size_all];

	            //Pointerdaki verileri Complete frame ata
	            for(int i=0;i<(elemansayisi_array(MBAP) + elemansayisi_array(RES_PDU_3));i++){
	                COMPLETE_FRAME[i] = *(COMPLETE_FRAME_TEMP +i);
	            }

	            for(int i =0;i<size_all;i++){
	            	printf("/x%x ",COMPLETE_FRAME[i]);
	            }

	            printf("\n");

	            //Temp pointeri boşalt
	            free(COMPLETE_FRAME_TEMP);

	            //Response mesajı gönder
	            send(socketNumber, (uint8_t*)COMPLETE_FRAME, elemansayisi_array(COMPLETE_FRAME)); //end function3
	    	}else if(function_code == 4){
	            unsigned short start_address = (receive_message[8] << 8)| receive_message[9];               //Coils de Output Address olarak alınacak.
	            unsigned short quantity_of_inputs = (receive_message[10] << 8)| receive_message[11];
	            unsigned short response_t_length_f3_f4 = 1 + 2 + 2*quantity_of_inputs;
	            MBAP[4] = highByte(response_t_length_f3_f4);
	            MBAP[5] = lowByte(response_t_length_f3_f4);
	            //RES_PDU boyutu tanımla
	            int size_RES_PDU = 2*quantity_of_inputs + 1 + 1;
	            //Fonksiyonu çağır ve geçici olarak PDU temp pointera ata (dönen değerlerin başlangıç adresi)
	            uint8_t *RES_PDU_TEMP = READ_IN_REGS(function_code, start_address, quantity_of_inputs);
	            //PDU arrayini oluştur.
	            uint8_t RES_PDU_4[size_RES_PDU];

	            //Temp pointerdaki değerleri PDU_4 e al
	            for(int i=0;i<size_RES_PDU;i++){
	                RES_PDU_4[i] = *(RES_PDU_TEMP +i);
	            }

	            //Temp PDU pointeri boşalt
	            free(RES_PDU_TEMP);

	            //Response mesaj uzunluğunu belirle
	            int size_all = elemansayisi_array(MBAP) + elemansayisi_array(RES_PDU_4);

	            printf("Size all: %d\n",size_all);
	            printf("Size MBAP: %d\n",elemansayisi_array(MBAP));
	            printf("Size PDU: %d\n",elemansayisi_array(RES_PDU_4));


	            //Tek framede birleştirdiktan sonra geçici olarak bit temp pointera ata
	            uint8_t *COMPLETE_FRAME_TEMP = COMBINE_MBAP_PDU(MBAP,RES_PDU_4,elemansayisi_array(MBAP),elemansayisi_array(RES_PDU_4));

	            //Complete frame oluştur
	            uint8_t COMPLETE_FRAME[size_all];


	            //Tempdeki değerleri Complete Frame'e al
	            for(int i=0;i<size_all;i++){
	              COMPLETE_FRAME[i] = *(COMPLETE_FRAME_TEMP+i);
	            }


	            printf("COMPLETE FRAME  \n");
	            for(int i=0;i<size_all;i++){
	            	printf("/x%x ",COMPLETE_FRAME[i]);
	            }

	            printf("\n");

	            //Temp pointeri boşalt
	            free(COMPLETE_FRAME_TEMP);
	            //Response mesajı gönder
	            send(socketNumber, (uint8_t*)COMPLETE_FRAME, elemansayisi_array(COMPLETE_FRAME)); //end function
	    	}else if(function_code == 5){
	            unsigned short start_address = (receive_message[8] << 8)| receive_message[9];               //Coils de Output Address olarak alınacak.
	            unsigned short quantity_of_inputs = (receive_message[10] << 8)| receive_message[11];
	            //Bu fonksiyonda Requestin bazı byte larında mesaj tipi değişiyor.
	            //Response message uzunluğu Request message uzunluğu ile aynı olacak. 12 byte
	            unsigned short response_pdu_length = 6;
	            MBAP[4] = highByte(response_pdu_length);
	            MBAP[5] = lowByte(response_pdu_length);
	            //RES_PDU boyutunu tanımla
	            int size_RES_PDU = 5;
	            //Fonksiyonu çağır ve geçici olarak PDU temp pointera ata
	            uint8_t *RES_PDU_TEMP = WRITE_SINGLE_COIL(function_code,start_address,quantity_of_inputs);
	            //PDU arrayini oluştur.
	            uint8_t RES_PDU_5[size_RES_PDU];

	            //Temp pointerdaki değerleri RES_PDU_5 e al
	            for(int i =0;i<size_RES_PDU;i++){
	              RES_PDU_5[i] = *(RES_PDU_TEMP+i);
	            }

	            //Temp pointeri boşalt
	            free(RES_PDU_TEMP);

	            int size_all = elemansayisi_array(MBAP) + elemansayisi_array(RES_PDU_5);

	            printf("Size all: %d\n",size_all);
				printf("Size MBAP: %d\n",elemansayisi_array(MBAP));
				printf("Size PDU: %d\n",elemansayisi_array(RES_PDU_5));

		        //Tek framede birleştirdiktan sonra geçici olarak bit temp pointera ata
		        uint8_t *COMPLETE_FRAME_TEMP = COMBINE_MBAP_PDU(MBAP,RES_PDU_5,elemansayisi_array(MBAP),elemansayisi_array(RES_PDU_5));

		          //Complete frame oluştur
		        uint8_t COMPLETE_FRAME[size_all];

		        //Pointerdaki verileri Complete frame ata
		        for(int i=0;i<(elemansayisi_array(MBAP) + elemansayisi_array(RES_PDU_5));i++){
		            COMPLETE_FRAME[i] = *(COMPLETE_FRAME_TEMP +i);
		        }

	            printf("COMPLETE FRAME  \n");
	            for(int i=0;i<size_all;i++){
	            	printf("/x%x ",COMPLETE_FRAME[i]);
	            }

	            printf("\n");

	            //Temp pointeri boşalt
	            free(COMPLETE_FRAME_TEMP);

	            //Response mesajı gönder
	            send(socketNumber, (uint8_t*)COMPLETE_FRAME, elemansayisi_array(COMPLETE_FRAME)); //end function 5
	    	}else if(function_code == 6){
	            unsigned short start_address = (receive_message[8] << 8)| receive_message[9];               //Coils de Output Address olarak alınacak.
	            unsigned short quantity_of_inputs = (receive_message[10] << 8)| receive_message[11];
	            //Bu fonksiyonda Requestin bazı byte larında mesaj tipi değişiyor.
	            //Response message uzunluğu Request message uzunluğu ile aynı olacak. 12 byte
	            unsigned short response_pdu_length = 6;
	            MBAP[4] = highByte(response_pdu_length);
	            MBAP[5] = lowByte(response_pdu_length);
	            //RES_PDU boyutunu tanımla
	            int size_RES_PDU = 5;

	            uint8_t *RES_PDU_TEMP = WRITE_SINGLE_REG(function_code,start_address,quantity_of_inputs);

	            //PDU arrayini oluştur
	            uint8_t RES_PDU_6[size_RES_PDU];

	            //Temp pointerdaki değerleri RES_PDU_5 e al
	            for(int i =0;i<size_RES_PDU;i++){
	              RES_PDU_6[i] = *(RES_PDU_TEMP+i);
	            }

	            //Temp pointeri boşalt
	            free(RES_PDU_TEMP);

	            int size_all = elemansayisi_array(MBAP) + elemansayisi_array(RES_PDU_6);

	            printf("Size all: %d\n",size_all);
				printf("Size MBAP: %d\n",elemansayisi_array(MBAP));
				printf("Size PDU: %d\n",elemansayisi_array(RES_PDU_6));

		        //Tek framede birleştirdiktan sonra geçici olarak bit temp pointera ata
		        uint8_t *COMPLETE_FRAME_TEMP = COMBINE_MBAP_PDU(MBAP,RES_PDU_6,elemansayisi_array(MBAP),elemansayisi_array(RES_PDU_6));

		        //Complete frame oluştur
		        uint8_t COMPLETE_FRAME[size_all];


		        //Pointerdaki verileri Complete frame ata
		        for(int i=0;i<size_all;i++){
		            COMPLETE_FRAME[i] = *(COMPLETE_FRAME_TEMP +i);
		        }

	            printf("COMPLETE FRAME  \n");
	            for(int i=0;i<size_all;i++){
	            	printf("/x%x ",COMPLETE_FRAME[i]);
	            }

	            printf("\n");

	            //Temp pointeri boşalt
	            free(COMPLETE_FRAME_TEMP);

	            //Response mesajı gönder
	            send(socketNumber, (uint8_t*)COMPLETE_FRAME, elemansayisi_array(COMPLETE_FRAME));// end function6
	    	}
	    }

	 }
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_Pin_GPIO_Port, CS_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_Pin_Pin */
  GPIO_InitStruct.Pin = CS_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_Pin_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*MODBUS TCP/IP Functions*//*Modbus TCP/IP functions from 1 to 6*/
//01 (0x01) Read Coils
uint8_t *READ_COILS(uint8_t function_code,uint8_t start_address, uint8_t quantity_of_inputs){
  //RES PDU'nun uzunluğu quantity of inputsun değerine göre değişiyor.
  REG[102] = GPIO_PIN_RESET;
  REG[103] = GPIO_PIN_SET;
  int length_res_pdu;
  int byte_count;
  if(quantity_of_inputs>8){
    //Burada response uzunluğunun belirlenmesi için quantity of inputs sayısının 8 e bölünmesi gerekir.
    //Tam kısmı bizim byte sayımızı oluşturur. Eğer 8 e tam bölünmüyorsa artık bitler için ekstra bir byte
    //atanması gerekir.
    byte_count = quantity_of_inputs /8;
    if(quantity_of_inputs %8 != 0){
      byte_count++;
    }
  }else{
    byte_count=1;
  }
  //response length = 1 byte(function code) + 1 byte(byte count) + output status byte
  length_res_pdu = byte_count +2;

  uint8_t *RES_PDU_1 = malloc(length_res_pdu*sizeof(uint8_t));

  RES_PDU_1[0] = function_code; //Function code
  RES_PDU_1[1] = byte_count; //byte count

  int mosfet1_status = REG[102];
  int mosfet2_status = REG[103];

  //Röleleri sür
  HAL_GPIO_WritePin(Role1_GPIO_Port, Role1_Pin, REG[102]);
  HAL_GPIO_WritePin(Role2_GPIO_Port, Role2_Pin, REG[103]);

  //Burada sadece iki tane röleyi süreceğinden start adress kullanmaya gerek kalmamıştır.
  //Eğer fazla sayıda bir dijital input okunacaksa geçici bir arraye push fonksiyonuyla status
  //değerler atılıp PDU registerina yazılır ardından temp arraydeki değerler resetlenerek
  //yeni değerlerin girişine hazır edilir.
  int status_role[8] = {0,0,0,0,0,0,mosfet2_status,mosfet1_status};

  int result_array_to_hex = arrayToHex(status_role,elemansayisi_array(status_role));
  printf("Array to hex: %d\n",result_array_to_hex);
  char *hexString = decimalToHex(result_array_to_hex);
  int con_hexString = atoi(hexString); //char olarak uint8 tipinde bir arraye atamayız. O yüzden char stringi int tipine dönüştürdüm.
  printf("Hex format: %d\n",con_hexString);

  //Sadece iki adet mosfet girişi olduğundan sadece üç elemanlı bir RESPONSE PDU arrayi oluşturulmuştur.
  RES_PDU_1[2] = (uint8_t)con_hexString;

  printf("RES_PDU_1[2] = %d",RES_PDU_1[2]);
  return RES_PDU_1;
}

//02 (0x02) Read Discrete Input
uint8_t *READ_DISCRETE_INPUT(uint8_t function_code, uint8_t quantity_of_inputs){
  //RES PDU'nun uzunluğu quantity of inputsun değerine göre değişiyor.
  int length_res_pdu;
  int byte_count;
  REG[102] = HAL_GPIO_ReadPin(MOSFET1_GPIO_Port, MOSFET1_Pin);
  REG[103] = HAL_GPIO_ReadPin(MOSFET2_GPIO_Port, MOSFET2_Pin);
  if(quantity_of_inputs>8){
    //Burada response uzunluğunun belirlenmesi için quantity of inputs sayısının 8 e bölünmesi gerekir.
    //Tam kısmı bizim byte sayımızı oluşturur. Eğer 8 e tam bölünmüyorsa artık bitler için ekstra bir byte
    //atanması gerekir.
    byte_count = quantity_of_inputs /8;
    if(quantity_of_inputs %8 != 0){
      byte_count++;
    }
  }else{
    byte_count=1;
  }
  //response length = 1 byte(function code) + 1 byte(byte count) + output status byte
  length_res_pdu = byte_count +2;

  uint8_t *RES_PDU_2 = malloc(length_res_pdu*sizeof(uint8_t));

  RES_PDU_2[0] = function_code;
  RES_PDU_2[1] = byte_count;

  int role1_status = REG[102];
  int role2_status = REG[103];

  //Burada sadece iki tane röle girişi okunacağından start adress kullanmaya gerek kalmamıştır.
  //Eğer fazla sayıda bir dijital input okunacaksa geçici bir arraye push fonksiyonuyla status
  //değerler atılıp PDU registerina yazılır ardından temp arraydeki değerler resetlenerek
  //yeni değerlerin girişine hazır edilir.
  int status_role[8] = {0,0,0,0,0,0,role2_status,role1_status};

  int result_array_to_hex = arrayToHex(status_role,elemansayisi_array(status_role));
  printf("Array to hex: %d\n",result_array_to_hex);
  char *hexString = decimalToHex(result_array_to_hex);
  int con_hexString = atoi(hexString); //char olarak uint8 tipinde bir arraye atamayız. O yüzden char stringi int tipine dönüştürdüm.
  printf("Hex format: %d\n",con_hexString);
  //Sadece iki adet röle girişi olduğundan sadece üç elemanlı bir RESPONSE PDU arrayi oluşturulmuştur.

  RES_PDU_2[2] = (uint8_t)con_hexString;
  printf("RES_PDU_2[2] = %d\n",RES_PDU_2[2]);
  return RES_PDU_2;
}

//03 (0x03) Read Holding Registers
uint8_t *READ_H_REGS( uint8_t function_code, uint8_t start_address, uint8_t quantity_of_inputs)
{
  // RES_PDU nun uzunluğu 2*quantity_of_inputs (1byte*2) + byte_count(1 byte) + function_code(1byte)
  int length_res_pdu = 2*quantity_of_inputs + 1 + 1;
  REG[100] = 355;
  REG[101] = 356;
  //Adjustment to memory dynamically
  uint8_t *RES_PDU_3 = malloc(length_res_pdu*sizeof(uint8_t));

  uint8_t byte_count = 2*quantity_of_inputs;
  RES_PDU_3[0] = function_code;
  RES_PDU_3[1] = byte_count;

  //Burada Arduino'nun Analog Output çıkışlarını okuyacak değerler yer alacak
  for(int i = 0; i<quantity_of_inputs; i++)
  {
    RES_PDU_3[(2*i)+2] = highByte(REG[start_address + i]);
    RES_PDU_3[(2*i)+3] = lowByte(REG[start_address + i]);
  }
  printf("RES_PDU \n");
  for (int i = 0; i < length_res_pdu; i++)
  {
    printf("/x%x ",RES_PDU_3[i]);
  }
  return RES_PDU_3;
}

//04 (0x04) Read Input Registers (analogRead() fonksiyonuyla analog voltaj girişlerini okuyabiliyoruz)
uint8_t *READ_IN_REGS(uint8_t function_code, uint8_t start_address, uint8_t quantity_of_inputs){
  // RES_PDU nun uzunluğu 2*quantity_of_inputs (1byte*2) + byte_count(1 byte) + function_code(1byte)
  int length_res_pdu = 2*quantity_of_inputs + 1 + 1;

  //Adjustment to memory dynamically
  uint8_t *RES_PDU_4 = malloc(length_res_pdu*sizeof(uint8_t));

  uint8_t byte_count = 2*quantity_of_inputs;
  RES_PDU_4[0] = function_code;
  RES_PDU_4[1] = byte_count;


  //Burada STM32 de 3 tane Analog Input (ADC) giriş değerlerini okuyan (kaç tane ise) değerler yer alacak.
  //Burada yapılması gereken bir map fonksiyonu oluşturmak
  //Eğer birden fazla ise switch case yapısı ile for döngüsü yapılmalı yada if-else if condition şeklinde
  //Burada örnek olarak map fonksiyonundan çıkan değeri REG[100]-REG[101] değerlerine atamamız lazım.
  //Şimdilik sadece değer olarak atanmalı sonrası için değişiklikler yapılmalı

  REG[100] = 64;
  REG[101] = 65;

  for(int i = 0; i<quantity_of_inputs; i++)
  {
    RES_PDU_4[(2*i)+2] = highByte(REG[start_address + i]); //analogRead fonksiyonunu analog giriş verdiğimizde koymalıyız
    RES_PDU_4[(2*i)+3] = lowByte(REG[start_address + i]);  //analogRead fonksiyonunu analog giriş verdiğimizde koymalıyız
  }
  printf("RES_PDU \n");
  for (int i = 0; i < length_res_pdu; i++)
  {
    printf("/x%x ",RES_PDU_4[i]);
  }
  return RES_PDU_4;
}


//05 (0x5) Write Single Coil
uint8_t *WRITE_SINGLE_COIL(uint8_t function_code,uint8_t output_address,uint16_t output_value){
    //RES PDU nun uzunluğu 5 byte = 1 byte (Function code) + 2byte (Output Address) + 2 byte (Output value)
    int length_res_pdu = 5;
    uint8_t *output_address_high_low_byte = Dec2Hex(output_address);
    uint8_t *output_value_high_low_byte = Dec2Hex(output_value);
    //Adjustment to memory dynamically
    uint8_t *RES_PDU_5 = malloc(length_res_pdu*sizeof(uint8_t));

    RES_PDU_5[0] = function_code;

    //Burada bir dijital bir pin çıktısı adresi atanması lazım.
    for(int i=0;i<2;i++){
      RES_PDU_5[i+1] = *(output_address_high_low_byte + i);
    }
    //Eğer output value 0xFF00 (aslında 0xFF ama high byte FF low byte 00 olarak ayarlanmış) ise
    //GPIO_Write fonksiyonu üzerinden olarak dijital pin aktif edilmelidir. Bu fonksiyona yazılmalıdır
    for(int i=0;i<2;i++){
      RES_PDU_5[i+3] = *(output_value_high_low_byte+i);
    }

    printf("RES_PDU \n");
    for(int i=0;i<length_res_pdu;i++){
        printf("/x%x \n",RES_PDU_5[i]);
    }

    return RES_PDU_5;
}

//06 (0x06) Write Single Register
//This command is used to record one value of the analog output AO.
uint8_t *WRITE_SINGLE_REG(uint8_t function_code,uint8_t start_address,uint8_t output_value){
  //RES PDU'nun uzunluğu 5byte = 1byte (function code) + 2byte(Register Byte High/Low) + 2byte(High/Low Byte meaning)
  int length_res_pdu = 5;
  uint8_t *output_address_high_low_byte = Dec2Hex(start_address);
  uint8_t *output_value_high_low_byte = Dec2Hex(output_value);

  uint8_t *RES_PDU_6 = malloc(length_res_pdu*sizeof(uint8_t));

  RES_PDU_6[0] = function_code;

  //Burada bir dijital bir pin çıktısı adresi atanması lazım.
  for(int i=0;i<2;i++){
    RES_PDU_6[i+1] = *(output_address_high_low_byte + i);
  }

  for(int i=0;i<2;i++){
    RES_PDU_6[i+3] = *(output_value_high_low_byte + i);
  }

  /*//Örnek olarak analog çıkışı sürmek için low- high olarak ayırdıktan sonra bu değerlere göre çıkış sürülebilir, eğer değer 5V tan daha büyükse
  int output_value_low = hexToDecimal(lowByte(output_value));
  int output_value_high = hexToDecimal(highByte(output_value));*/

  //Duruma göre output değeriyle çıkış sürülebilir. Mesela 5V çıkış alacaksak direkt output değerini sürebilir.
  //DAC ataması yapıldı sadece değeri sürmek kalıyor.

  //PDU yu bastır
  printf("RES_PDU: \n");
  for(int i=0;i<length_res_pdu;i++){
    printf("/x%x \n",RES_PDU_6[i]);
  }
  return RES_PDU_6;
}



//Bir 8 elemanlı arrayi (0 ve 1 lerden oluşan) hexadecimale çeviriyor.
unsigned int arrayToHex(int a[], int length) {
    unsigned int hexValue = 0;
    for (int i = 0; i < length; i++) {
        hexValue = (hexValue << 1) | a[i];
    }
    return hexValue;
}

//Hexadecimal to decimal converts
uint8_t hexToDecimal(uint8_t hex) {
    uint8_t decimal = 0;

    if (hex >= '0' && hex <= '9') {
        decimal = hex - '0';
    } else if (hex >= 'A' && hex <= 'F') {
        decimal = hex - 'A' + 10;
    } else if (hex >= 'a' && hex <= 'f') {
        decimal = hex - 'a' + 10;
    }

    return decimal;
}


//Decimal bir değeri Hex sayısına çevirmek için (arrayToHex den gelen sayıyı hex formatına çevirir)
char* decimalToHex(unsigned int decimal) {
    char* hexValue = (char*)malloc(sizeof(char) * 10); // Bellekte alan ayır, maksimum 10 karakterlik hex değeri
    if(hexValue == NULL) {
        printf("Bellek tahsisi basarisiz!\n");
        exit(1);
    }
    sprintf(hexValue, "%X", decimal); // Ondalık sayıyı hexadecimal olarak dönüştür
    return hexValue;
}

//Decimal to High-Low Byte Hexidecimal value
uint8_t *Dec2Hex(uint16_t c){
    uint8_t high_byte = (c>>8) & 0xFF;
    uint8_t low_byte = c & 0xFF;

    uint8_t *result = malloc(2*sizeof(uint8_t));
    result[0] = high_byte;
    result[1] = low_byte;

    return result;
}

uint8_t *COMBINE_MBAP_PDU(uint8_t *MBAP, uint8_t *PDU,int size_MBAP,int size_PDU)
{
  int size_all = size_MBAP + size_PDU;
  uint8_t *COMPLETE_FRAME = malloc(size_all * sizeof(uint8_t));

  for(int i = 0; i<size_MBAP; i++)
  {
    COMPLETE_FRAME[i] = MBAP[i];
  }
  for (int i = 0; i<size_PDU; i++)
  {
    COMPLETE_FRAME[i + size_MBAP] = PDU[i];
  }
  printf("COMPLETE_FRAME: \n");
  for (int i = 0; i < size_all; i++)
  {
    printf("/x%x ",*(COMPLETE_FRAME +i));
  }
  printf("\n");
  return COMPLETE_FRAME;
}

//Hercule TCP client için denemeler
//00 01 00 00 00 06 01 01 00 01 00 02 ====> Fonksiyon 1
//00 01 00 00 00 06 01 02 00 64 00 02 ====> Fonksiyon 2
//00 01 00 00 00 06 01 03 00 64 00 02 ====> Fonksiyon 3
//00 01 00 00 00 06 01 04 00 64 00 02 ====> Fonksiyon 4
//00 01 00 00 00 06 01 05 00 01 FF 00 ====> Fonksiyon 5
//00 01 00 00 00 06 01 06 00 64 00 05 ====> Fonksiyon 6


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
