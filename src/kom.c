/*=======================================================================================*
 * @file    kom.c
 * @author  Marcin
 * @version 1.0
 * @date    22 maj 2018
 * @brief   
 *======================================================================================*/

/**
 * @addtogroup kom.c Description
 * @{
 * @brief  
 */

/*======================================================================================*/
/*                       ####### PREPROCESSOR DIRECTIVES #######                        */
/*======================================================================================*/
/*-------------------------------- INCLUDE DIRECTIVES ----------------------------------*/
#include <stdio.h>
#include <stdlib.h>
//#include <stm32f30x_conf.h>
#include "stm32f4xx_conf.h"
#include "delay.h"
#include "debugkom.h"
#include "timer.h"
#include "usart.h"
#include "kom.h"
/*----------------------------- LOCAL OBJECT-LIKE MACROS -------------------------------*/
#define KOM_SEND_BUF_SIZE 255
#define KOM_RCV_BUF_SIZE 255
#define KOM_PREFIX 0x01
#define KOM_ID_DATA 0x44
#define KOM_ID_CFG 0x46
#define KOM_ID_CMD 0x43
#define KOM_NAK 0x15
#define KOM_AK	0x06
#define KOM_BELL 0x07
#define KOM_END_OF_HISTORY 0x04


#define KOM_FRAME_TIMEOUT_MS 1000 // 20ms - timeout for frame receiving

/*---------------------------- LOCAL FUNCTION-LIKE MACROS ------------------------------*/

/*======================================================================================*/
/*                      ####### LOCAL TYPE DECLARATIONS #######                         */
/*======================================================================================*/
/*---------------------------- ALL TYPE DECLARATIONS -----------------------------------*/

/*-------------------------------- OTHER TYPEDEFS --------------------------------------*/

/*------------------------------------- ENUMS ------------------------------------------*/
typedef enum{KOM_SEND_HIST_CFG, KOM_SEND_HIST_DATA, INIT, KOM_SEND_HIST, KOM_SEND_RECORD, KOM_IDLE,KOM_HEADER,KOM_RCV,KOM_APP, KOM_SEND_CFG} kom_state_T;

typedef enum{KOM_RESP_OK,KOM_RESP_NOK} kom_resp_T;
/*------------------------------- STRUCT AND UNIONS ------------------------------------*/

/*======================================================================================*/
/*                    ####### LOCAL FUNCTIONS PROTOTYPES #######                        */
/*======================================================================================*/
static kom_state_T Init(void);
static kom_state_T KOM_Idle(void);
static kom_state_T KOM_Header(void);
static kom_state_T KOM_Rcv(void);
static kom_state_T KOM_App(void);
static kom_resp_T KOM_CheckCRC(void);
static kom_resp_T KOM_ParseData(void);
static kom_state_T KOM_Send_Cfg(void);
static kom_state_T KOM_Send_Hist(void);
static kom_state_T KOM_Send_Record(void);
static kom_state_T KOM_Send_Hist_Cfg(void);
static kom_state_T KOM_Send_Hist_Data(void);

void KOM_USART_Send(volatile char *c);
void KOM_SendChar(char c);
void KOM_BufClr(void);
void KOM_SendAK(void);
void KOM_SendNAK(void);

void KOM_SendBufClr(void);


/*======================================================================================*/
/*                         ####### OBJECT DEFINITIONS #######                           */
/*======================================================================================*/
/*--------------------------------- EXPORTED OBJECTS -----------------------------------*/

/*---------------------------------- LOCAL OBJECTS -------------------------------------*/
kom_state_T kom_state;
//kom_record_data_T meas_data, *wsk = &meas_data;

volatile uint8_t kom_send_buf[KOM_SEND_BUF_SIZE];
volatile uint8_t kom_send_buf_cnt = 0;
volatile uint8_t kom_send_buf_rdy = 0;

volatile uint8_t kom_rcv_buf[KOM_RCV_BUF_SIZE];
volatile uint8_t kom_frame_cnt = 0;
volatile uint8_t kom_frame_len = 0;
volatile uint8_t bell_flag = 0;
volatile uint8_t end_of_history_flag = 0;
volatile uint8_t cfg_changed_flag = 0;
volatile uint8_t new_record_flag = 0;

volatile uint8_t hist_cfg_rdy = 0;
volatile uint8_t hist_data_rdy = 0;
volatile uint8_t all_hist_data_sent = 0;


volatile uint8_t timeout_cnt = 0;

uint8_t kom_timer;
/*======================================================================================*/
/*                  ####### EXPORTED FUNCTIONS DEFINITIONS #######                      */
/*======================================================================================*/

void KOM_Init(void)
{
	Timer_Register(&kom_timer,KOM_FRAME_TIMEOUT_MS,timerOpt_AUTOSTOP);
//	Timer_Stop(&kom_timer);
	Timer_Reset(&kom_timer);
	kom_state = KOM_IDLE;
	GPIO_SetBits(GPIOD, GPIO_Pin_14);
}
void KOM_Main(void)
{
	switch(kom_state){

	case KOM_IDLE:
		kom_state = KOM_Idle();
		break;

	case KOM_SEND_RECORD:
		kom_state = KOM_Send_Record();
		break;

	case KOM_SEND_HIST:
		kom_state = KOM_Send_Hist();
		break;

	case KOM_SEND_HIST_CFG:
		kom_state = KOM_Send_Hist_Cfg();
		break;

	case KOM_SEND_HIST_DATA:
		kom_state = KOM_Send_Hist_Data();
		break;

	case INIT:
		kom_state = Init();
		break;

	case KOM_HEADER:
		kom_state = KOM_Header();
		break;

	case KOM_RCV:
		kom_state = KOM_Rcv();
		break;

	case KOM_APP:
		kom_state = KOM_App();
		break;

	case KOM_SEND_CFG:
		kom_state = KOM_Send_Cfg();
		break;
	default:
		break;
	}

}

/*======================================================================================*/
/*                   ####### LOCAL FUNCTIONS DEFINITIONS #######                        */
/*======================================================================================*/
static kom_state_T KOM_Idle(void)
{
	PC_Debug("Jestem w IDLE\n\r");
	GPIO_SetBits(GPIOD, GPIO_Pin_15);
	//symulacja ustawienia flagi, która informuje o nowym recordzie pomiarowym do wyslania
	if( GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1 ){
		new_record_flag = 1;
		return KOM_SEND_RECORD;
	}
	//

//	if( all_hist_data_sent == 0){
		if(kom_frame_cnt >= 1){
			if(kom_rcv_buf[0] == KOM_BELL){
				KOM_SendAK();
				KOM_BufClr();

				//modul pomairowy ustawia te flagi kiedy bedzie gotowy na wysylanie danych historycznych
				hist_cfg_rdy = 1;
				hist_data_rdy = 1;
				all_hist_data_sent = 0;
				//
				return KOM_SEND_HIST;
			}
			else{
				return KOM_HEADER;
			}
		}
//	}

//	if(kom_frame_cnt >= 1){
//		return KOM_HEADER;
//	}

	return KOM_IDLE;
}

static kom_state_T KOM_Send_Hist(void){
	PC_Debug("Jestem w SEND_HIST\n\r");
	if( hist_cfg_rdy == 1){
		return KOM_SEND_HIST_CFG;
	}

	if( hist_data_rdy == 1){
		return KOM_SEND_HIST_DATA;
	}

	if( all_hist_data_sent == 1){
		return KOM_IDLE;
	}

	if( hist_cfg_rdy == 0 && hist_data_rdy == 0){
		//wyslij hist end i poczekaj na ak

		PC_Debug("Wysylane END OF HISTORY\n\r");

		KOM_SendBufClr(); // czyszczenie bufora nadawczego

		//przygotowana ramka z End Of History CMD
		//\0x01\0x02\0x01\0x02\0x03\0x04\0x43\0x04
		kom_send_buf[0] = KOM_PREFIX; //Prefix
		kom_send_buf[1] = 0x02; //DataLength
		kom_send_buf[2] = 0x01; //CRC1
		kom_send_buf[3] = 0x02; //CRC2
		kom_send_buf[4] = 0x03; //CRC3
		kom_send_buf[5] = 0x04; //CRC4
		kom_send_buf[6] = KOM_ID_CMD; //Command ID
		kom_send_buf[7] = 0x04; //CONFIG

		//Wysylanie konfiguracji
		KOM_USART_Send(kom_send_buf);
		//Uruchomienia timera (póŸniej przyda siê inicjalizacja, by ustawiony by³ odpowiedni timeout
		Timer_Reset(&kom_timer);
		while( Timer_Check(&kom_timer) == 0 ){
					if( kom_rcv_buf[0] == KOM_AK){
						PC_Debug("Modul pomiarowy poprawnie odebral END OF HISOTYR\n\r");
						Timer_Stop(&kom_timer);
						KOM_BufClr();
						all_hist_data_sent = 1;
//						hist_cfg_rdy = 0;
						timeout_cnt = 0;
						return KOM_SEND_HIST;
					}
					else{
						KOM_BufClr();
					}
		}
		timeout_cnt++;
		if( timeout_cnt == 6){
			PC_Debug("Timeout\n\r");
			PC_Debug("Display niepoprawnie odebral end of history\n\r");
			Timer_Stop(&kom_timer);
			KOM_SendBufClr();
//			all_hist_data_sent = 1;
//			hist_cfg_rdy = 0;
			timeout_cnt = 0;
			return KOM_IDLE;
		}
		return KOM_SEND_HIST;
		}
}

static kom_state_T  KOM_Send_Hist_Cfg(void)
	{
	PC_Debug("Jestem w SEND_HIST_CFG\n\r");

	KOM_SendBufClr(); // czyszczenie bufora nadawczego

	//przygotowana ramka z konfiguracja do wyslania
	kom_send_buf[0] = KOM_PREFIX; //Prefix
	kom_send_buf[1] = 0x02; //DataLength
	kom_send_buf[2] = 0x01; //CRC1
	kom_send_buf[3] = 0x02; //CRC2
	kom_send_buf[4] = 0x03; //CRC3
	kom_send_buf[5] = 0x04; //CRC4
	kom_send_buf[6] = KOM_ID_CFG; //Config ID
	kom_send_buf[7] = 0x04; //CONFIG

	//Wysylanie konfiguracji
	KOM_USART_Send(kom_send_buf);
	//Uruchomienia timera (póŸniej przyda siê inicjalizacja, by ustawiony by³ odpowiedni timeout
	Timer_Reset(&kom_timer);
	while( Timer_Check(&kom_timer) == 0 ){
				if( kom_rcv_buf[0] == KOM_AK){
					PC_Debug("Modul pomiarowy poprawnie odebral konfiguracje\n\r");
					Timer_Stop(&kom_timer);
					KOM_BufClr();
					hist_cfg_rdy = 0;
					timeout_cnt = 0;
					return KOM_SEND_HIST;
				}
				else{
					KOM_BufClr();
				}
	}
	timeout_cnt++;
	if( timeout_cnt == 6){
		PC_Debug("Timeout\n\r");
		Timer_Stop(&kom_timer);
		KOM_SendBufClr();
		hist_cfg_rdy = 0;
		timeout_cnt = 0;
		return KOM_SEND_HIST;
	}
	return KOM_SEND_HIST;
	}

static kom_state_T  KOM_Send_Hist_Data(void)
	{
	PC_Debug("Jestem w SEND_HIST_DATA\n\r");

	KOM_SendBufClr(); // czyszczenie bufora nadawczego

	//przygotowana ramka z dan¹ do wyslania
//	\0x01\0x06\0x01\0x02\0x03\0x04 \0x44\ 0x4A\0x41\0x50\0x4B\0x4F
	kom_send_buf[0] = KOM_PREFIX; //Prefix
	kom_send_buf[1] = 0x06; //DataLength
	kom_send_buf[2] = 0x01; //CRC1
	kom_send_buf[3] = 0x02; //CRC2
	kom_send_buf[4] = 0x03; //CRC3
	kom_send_buf[5] = 0x04; //CRC4
	kom_send_buf[6] = KOM_ID_DATA; //Data ID
	kom_send_buf[7] = 'J'; //Data "J"
	kom_send_buf[8] = 'A'; //Data "A"
	kom_send_buf[9] = 'P'; //Data "P"
	kom_send_buf[10] = 'K'; //Data "K"
	kom_send_buf[11] = 'O'; //Data "O"

	//Wysylanie danych
	KOM_USART_Send(kom_send_buf);
	//Uruchomienia timera (póŸniej przyda siê inicjalizacja, by ustawiony by³ odpowiedni timeout
	Timer_Reset(&kom_timer);
	while( Timer_Check(&kom_timer) == 0 ){
				if( kom_rcv_buf[0] == KOM_AK){
					PC_Debug("Modul pomiarowy poprawnie odebral dane\n\r");
					Timer_Stop(&kom_timer);
					KOM_BufClr();
					hist_data_rdy = 0;
					timeout_cnt = 0;
					return KOM_SEND_HIST;
				}
				else{
					KOM_BufClr();
				}
	}
	timeout_cnt++;
	if( timeout_cnt == 6){
		PC_Debug("Timeout\n\r");
		Timer_Stop(&kom_timer);
		KOM_SendBufClr();
		hist_data_rdy = 0;
		timeout_cnt = 0;
		return KOM_SEND_HIST;
	}
	return KOM_SEND_HIST;
	}

static kom_state_T KOM_Send_Record(void){
	PC_Debug("Jestem w SEND_RECORD\n\r");
	PC_Debug("Wyslano RECORD\n\r");

	KOM_SendBufClr(); // czyszczenie bufora nadawczego

	//przygotowana ramka z dan¹ do wyslania
//	\0x01\0x06\0x01\0x02\0x03\0x04 \0x44\ 0x4A\0x41\0x50\0x4B\0x4F
	kom_send_buf[0] = KOM_PREFIX; //Prefix
	kom_send_buf[1] = 0x06; //DataLength
	kom_send_buf[2] = 0x01; //CRC1
	kom_send_buf[3] = 0x02; //CRC2
	kom_send_buf[4] = 0x03; //CRC3
	kom_send_buf[5] = 0x04; //CRC4
	kom_send_buf[6] = KOM_ID_DATA; //Data ID
	kom_send_buf[7] = 'C'; //Data "C"
	kom_send_buf[8] = 'H'; //Data "H"
	kom_send_buf[9] = 'L'; //Data "L"
	kom_send_buf[10] = 'E'; //Data "E"
	kom_send_buf[11] = 'B'; //Data "B"

	KOM_USART_Send(kom_send_buf);
	KOM_BufClr();

	//Uruchomienia timera (póŸniej przyda siê inicjalizacja, by ustawiony by³ odpowiedni timeout
		Timer_Reset(&kom_timer);
		while( Timer_Check(&kom_timer) == 0 ){
					if( kom_rcv_buf[0] == KOM_AK){
						PC_Debug("Modul pomiarowy poprawnie odebral nowy RECORD\n\r");
						Timer_Stop(&kom_timer);
						KOM_BufClr();
//						hist_data_rdy = 0;
						timeout_cnt = 0;
						return KOM_IDLE;
					}
					else{
						KOM_BufClr();
					}
		}
		timeout_cnt++;
		if( timeout_cnt == 6){
			PC_Debug("Timeout\n\r");
			Timer_Stop(&kom_timer);
			KOM_SendBufClr();
//			hist_data_rdy = 0;
			timeout_cnt = 0;
			return KOM_IDLE;
		}
	return KOM_SEND_RECORD;
//	return KOM_IDLE;
}

static kom_state_T KOM_Send_Cfg(void){
	//wejscie w ten stan powinno byc uzaleznione od ustawienia flagi cfg_chander_flag na "1"
	//tutaj powinno zajsc wyslanie danych konfiguracyjnych do modulu pomiarowego
	//potem powinno nast¹pic zerowanie flagi i powrot do stanu IDLE
	PC_Debug("Wyslano CONFIG do Wyswietlacza\n\r");
	//funkcja wysy³ania danych konfiguracyjnych();
	Timer_Reset(&kom_timer);
	while( Timer_Check(&kom_timer) == 0 && cfg_changed_flag == 1){
		if( kom_rcv_buf[0] == KOM_AK){
			PC_Debug("Modul pomiarowy poprawnie odebral konfiguracje\n\r");
			Timer_Stop(&kom_timer);
			KOM_BufClr();
			cfg_changed_flag = 0;
			timeout_cnt = 0;
			return KOM_IDLE;
		}
	}
	timeout_cnt++;
	if( timeout_cnt == 6){
		PC_Debug("Timeout\n\r");
		Timer_Stop(&kom_timer);
		KOM_SendBufClr();
		cfg_changed_flag = 0;
		timeout_cnt = 0;
		return KOM_IDLE;
	}
	return KOM_SEND_CFG;
}

static kom_state_T Init(void){
	if( end_of_history_flag == 1){
		KOM_BufClr();
		return KOM_IDLE;
	}

	while(1){
	PC_Debug("Jestem w INIT\n\r");
	}

	if(kom_frame_cnt >= 1){
		return KOM_HEADER;
	}
	if( end_of_history_flag == 0){
		return INIT;
	}
	return KOM_IDLE;
}

static kom_state_T KOM_Header(void)
{
	PC_Debug("Jestem w HEADER\n\r");
	if(kom_rcv_buf[0] == KOM_PREFIX){
		kom_frame_len = kom_rcv_buf[1];
		Timer_Reset(&kom_timer);
		return KOM_RCV;
	}
	KOM_BufClr();
	KOM_SendNAK();
	return KOM_IDLE;
}

static kom_state_T KOM_Rcv(void)
{
	PC_Debug("Jestem w RECEIVE\n\r");
	if(Timer_Check(&kom_timer) == 1 || GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1){
		PC_Debug("Timeout\n\r");
		KOM_BufClr();
		KOM_SendNAK();

//		if( end_of_history_flag == 0){
//			return KOM_IDLE;
////			return INIT;
//		}
		return KOM_IDLE;
	}
	else if(kom_frame_cnt-6 == kom_frame_len){
		if(KOM_CheckCRC() == KOM_RESP_OK){
			return KOM_APP;
		}
		else{
			KOM_BufClr();
			KOM_SendNAK();

//			if( end_of_history_flag == 0){
//				return INIT;
//			}
			return KOM_IDLE;
		}
	}
	return KOM_RCV;
}

static kom_state_T KOM_App(void)
{
	PC_Debug("Jestem w APP\n\r");
	if(KOM_ParseData() == KOM_RESP_OK){
		KOM_SendAK();
		GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
	}
	else{
		KOM_SendNAK();
	}
	KOM_BufClr();

//	if( end_of_history_flag == 0){
//		return INIT;
//	}
	return KOM_IDLE;
}

static kom_resp_T KOM_CheckCRC(void)
{
	PC_Debug("Jestem w Check CRC\n\r");
	if(kom_rcv_buf[2] == 0x01 && kom_rcv_buf[3] == 0x02 && kom_rcv_buf[4] == 0x03 && kom_rcv_buf[5] == 0x04 ){
		return KOM_RESP_OK;
	}
	return KOM_RESP_NOK;
}

static kom_resp_T KOM_ParseData(void)
{
	kom_record_data_T data;

	switch(kom_rcv_buf[6]){
	case KOM_ID_DATA: // DATA
		{
			PC_Debug("Odebrano DANE: ");
			PC_Debug(kom_rcv_buf);
//			for(uint8_t i=7; i<kom_frame_len+6;i++){
//				KOM_SendChar(kom_rcv_buf[i]);
//			}
			uint8_t index = 7;
		    data.timestamp = (kom_rcv_buf[index] << 8) | kom_rcv_buf[index+1];
		    data.moist = (kom_rcv_buf[index+2] << 8) | kom_rcv_buf[index+3];
		    data.flow = (kom_rcv_buf[index+4] << 8) | kom_rcv_buf[index+5];
		    data.temp_z = (kom_rcv_buf[index+6] << 8) | kom_rcv_buf[index+7];
		    data.temp_in = (kom_rcv_buf[index+8] << 8) | kom_rcv_buf[index+9];
		    data.temp_out = (kom_rcv_buf[index+10] << 8) | kom_rcv_buf[index+11];
			data.hum_in = kom_rcv_buf[index+12];
			data.hum_out = kom_rcv_buf[index+13];

			PC_Debug("\n\rData Parsed\n\r");
			GPIO_SetBits(GPIOD, GPIO_Pin_13);
			GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
			break;
		}
	case KOM_ID_CFG: // CONFIG
	{
		PC_Debug("Odebrano CONFIG: ");
		PC_Debug("CFG\n\r");
		break;
	}
	case KOM_ID_CMD: // COMMAND
		PC_Debug("Odebrano COMMAND: ");

		//end of history checking
		if( kom_rcv_buf[7] == KOM_END_OF_HISTORY){
			PC_Debug("END OF HISTORY\n\r");
			end_of_history_flag = 1;
		}
		break;
	default:
		return KOM_RESP_NOK;
		break;
	}
	return KOM_RESP_OK;
}

void USART6_IRQHandler(void){
	if (USART_GetITStatus(USART6, USART_IT_RXNE) == SET){
		if (USART_GetFlagStatus(USART6, USART_FLAG_RXNE)){
			if(kom_frame_cnt < KOM_RCV_BUF_SIZE-1){
				kom_rcv_buf[kom_frame_cnt++] = USART_ReceiveData(USART6);
			}
		}
	}
	USART_ClearITPendingBit(USART6, USART_IT_RXNE);
}
/**
 * @} end of group kom.c
 */

void KOM_BufClr(void)
{
		for(uint16_t i=0;i<KOM_RCV_BUF_SIZE;i++){
			kom_rcv_buf[i]=0;
		}
		kom_frame_cnt=0;
		USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
}

void KOM_SendBufClr(void)
{
		for(uint16_t i=0;i<KOM_SEND_BUF_SIZE;i++){
			kom_send_buf[i]=0;
		}
//		kom_frame_cnt=0;
//		USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
}

void KOM_SendAK(void)
{
	KOM_SendChar(KOM_AK);
}
void KOM_SendNAK(void)
{
	KOM_SendChar(KOM_NAK);
}

void KOM_USART_Send(volatile char *c)
	{
     //Petla dziala do puki bedzie jakis znak do wyslania
     while(*c)
     {
        //Sprawdza czy rejestr danych zostal oprózniony
        while( !(USART6->SR & 0x00000040) );
        //Przeslij dane,
        USART_SendData(USART6, *c);
        *c++;
     }
}

void KOM_SendChar(char c)
	{
     //Sprawdza czy bufor nadawczy jest pusty
     while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
     USART_SendData(USART6, c);
	}
