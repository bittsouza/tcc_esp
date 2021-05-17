/* CAN Obd2 scanner

   Author: Gabriel Bittencourt de Souza
   Git_Hub: bittsouza
   
*/

/*
 * The following code send requests from ECU using CAN BUS. The data qnted is sent by ESP32 from ECU and the
 * data receiver is processed to get Human Readable information from the Vehicle. 
 * The current information acquired is:
 * 1) RPM - Revolutions per minute from the motor
 * 2) SPD - Speed of the vehicle in Km/h
 * 3) INT - Intake temperature in ºC
 * 4) TPS - Throttle position in %
 * 5) FUL - Fuel in the tank in %
 * 6) ODO - Odometer in Km
 * 7) LBD - Actual trnsmission LBDr
 * 8) RTM - Run time engine since start
 * 
 * Definition of DATA[]
 * Frame DATA[0]
 * Frame DATA[1]
 * Frame DATA[2] == PID ANSWER
 * Frame DATA[3] == A
 * Frame DATA[4] == B
 * Frame DATA[5] == C
 * 
 * All PID's, Baud rate and ID request tested in VW Polo 2018.
 * 
 * For more information of the obd2 message and PID's access:
 * 
 * https://www.csselectronics.com/screen/page/obd-ii-pid-examples/language/en
 * https://en.wikipedia.org/wiki/OBD-II_PIDs
 */

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/can.h"
#include "pids.hvv"

// /* --------------------- Definitions and static variables ------------------ */
// //System Configuration
// #define EXAMPLE_TAG                     "CAN Listen Only"
// #define NO_OF_ITERS                     50          //Number of interations
// #define RX_TASK_PRIO                    9           //Priority of the main task
// #define TX_GPIO_NUM                     17          //Tx pin ESP32
// #define RX_GPIO_NUM                     16          //Rx pin ESP32
// #define CAN_MAX_DATA_LEN                8           //< Maximum number of data bytes in a CAN2.0B frame

// //Can configuration
// #define ID_HELLO_ECU                    0x7DF       //ID to request data for ECU
// #define ID_ANSWER_ECU                   0x7E8       //ID answer from ECU with the data

// #define SERVICE_MODE_CURRENT            01        //Current information from ECU
// #define SERVICE_MODE_FREEZE             02        //Freze frame data information from ECU
// #define SERVICE_MODE_ERR_CODES          03        //Request trouble codes

// //PIDs requested
// #define ID_ENGINE_RPM                   0x0C // RPM - Revolutions per minute from the motor
// #define ID_ENGINE_SPD                   0x0D // SPD - Intake temperature in ºC
// #define ID_ENGINE_INT                   0x0F // INT - Intake temperature in ºC
// #define ID_ENGINE_TPS                   0x11 // TPS - Throttle position in %
// #define ID_ENGINE_FUL                   0x2F // FUL - Fuel in the tank in %
// #define ID_ENGINE_ODO                   0x31 // ODO - Total distance traveled in Km
// #define ID_ENGINE_LBD                   0x44 // LBD - Lambda sensor
// #define ID_ENGINE_RTM                   0x1F // RTM - Run time since the engine start
// #define ID_ENGINE_ETH                   0x52 // ETH - Ethanol %

void obd_id_class(can_message_t rx_msg);

int TIMING_CONFIG = 0;
int Contador_TX = 0;

//Config to no expect AKC
static const can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

//Config bit timing to 500Kbits of rate
static const can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();

//Set TX queue length to 0 due to listen only mode
static const can_general_config_t g_config = {.mode = CAN_MODE_NO_ACK,
                                              .tx_io = TX_GPIO_NUM, .rx_io = RX_GPIO_NUM,
                                              .clkout_io = CAN_IO_UNUSED, .bus_off_io = CAN_IO_UNUSED,
                                              .tx_queue_len = 5, .rx_queue_len = 5,
                                              .alerts_enabled = CAN_ALERT_NONE,
                                              .clkout_divider = 0};


// //RPM
// static const can_message_t start_message_RPM = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
//                                             .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_RPM,55,55,55,55,55}};                                              
// //speed
// static const can_message_t start_message_SPD = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
//                                             .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_SPD,01,55,55,55,55}};                                              
// //Intake Temp
// static const can_message_t start_message_INT = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
//                                             .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_INT,01,55,55,55,55}};                                              
// //Throttle position
// static const can_message_t start_message_TPS = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
//                                             .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_TPS,01,55,55,55,55}};                                              
// //Fuel Level
// static const can_message_t start_message_FUL = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
//                                             .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_FUL,01,55,55,55,55}};                                              

// //Odometer
// static const can_message_t start_message_ODO = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
//                                             .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_ODO,55,55,55,55,55}};                                              
//Lambda sensor
static const can_message_t start_message_LBD = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
                                           .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_LBD,01,55,55,55,55}};                                              
// //Run time engine
// static const can_message_t start_message_RTM = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
//                                             .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_RTM,55,55,55,55,55}};                                              
// //Ethanol %
// static const can_message_t start_message_ETH = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
//                                             .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_ETH,55,55,55,55,55}};                                              




static SemaphoreHandle_t rx_sem;
esp_err_t error_rx_can;
esp_err_t error_tx_can;

can_message_t rx_msg;

// Data acquired and processed.
uint16_t RPM;
uint16_t SPD;
uint16_t INT;
uint16_t TPS;
uint16_t FUL;
uint16_t ODO;
uint16_t LBD_A;
uint16_t LBD_B;
uint16_t LBD;
uint16_t RTM;
uint16_t ETH;

/* --------------------------- Tasks and Functions -------------------------- */

static void can_receive_task(void *arg)
{
   
    
    uint32_t iterations = 0;
    

    while (iterations < NO_OF_ITERS) {

        system("cls");
        
        vTaskDelay(5/portTICK_PERIOD_MS);
        // error_tx_can =  can_transmit(&start_message_TPS, portMAX_DELAY);
        // obd_id_class(rx_msg);
        // vTaskDelay(5/portTICK_PERIOD_MS);
        // error_tx_can =  can_transmit(&start_message_INT, portMAX_DELAY);
        // obd_id_class(rx_msg);
        // vTaskDelay(5/portTICK_PERIOD_MS);
        // error_tx_can =  can_transmit(&start_message_FUL, portMAX_DELAY);
        // obd_id_class(rx_msg);
        // vTaskDelay(5/portTICK_PERIOD_MS);
        // error_tx_can =  can_transmit(&start_message_SPD, portMAX_DELAY);
        // obd_id_class(rx_msg);
        // vTaskDelay(5/portTICK_PERIOD_MS);
        // error_tx_can =  can_transmit(&start_message_RPM, portMAX_DELAY);
        // obd_id_class(rx_msg);
        // vTaskDelay(5/portTICK_PERIOD_MS);
        // error_tx_can =  can_transmit(&start_message_ODO, portMAX_DELAY);
        // obd_id_class(rx_msg);
        // vTaskDelay(5/portTICK_PERIOD_MS);
        error_tx_can =  can_transmit(&start_message_LBD, portMAX_DELAY);
        obd_id_class(rx_msg);
        vTaskDelay(5/portTICK_PERIOD_MS);
        // error_tx_can =  can_transmit(&start_message_RTM, portMAX_DELAY);
        // obd_id_class(rx_msg);
        // vTaskDelay(5/portTICK_PERIOD_MS);
        // error_tx_can =  can_transmit(&start_message_ETH, portMAX_DELAY);
        // obd_id_class(rx_msg);
        // vTaskDelay(5/portTICK_PERIOD_MS);


        if(error_tx_can == ESP_OK){
            //ESP_LOGI(EXAMPLE_TAG, "Transmitted start command");
            Contador_TX=1;
        } 
        else{
            ESP_LOGI(EXAMPLE_TAG, "ERRO_TX");
        }
        
    }

    vTaskDelete(NULL);
}

void obd_id_class(can_message_t rx_msg){
uint32_t cont = 0;
    error_rx_can = can_receive(&rx_msg, portMAX_DELAY);
    
        if(error_rx_can == ESP_OK){
            if(rx_msg.identifier == 401604624) {
                cont ++;
            } else{

            
                switch (rx_msg.data[2])
                {   
                    
                    case ID_ENGINE_RPM:
                        RPM = (((rx_msg.data[3]*256)+rx_msg.data[4])/4); //RPM=(((A*256)+B)/4)
    //                    ESP_LOGI(EXAMPLE_TAG, "RPM: %d", RPM);
                    break;  

                    case ID_ENGINE_SPD:
                        SPD = (rx_msg.data[3]);
  //                      ESP_LOGI(EXAMPLE_TAG, "SPEED: %d KPH", SPD);
                    break;

                    case ID_ENGINE_INT:
                        INT = ((rx_msg.data[3])-40);
//                        ESP_LOGI(EXAMPLE_TAG, "INTAKE: %d ºC", INT);
                    break;

                    case ID_ENGINE_TPS: 
                        TPS = ((rx_msg.data[3]*100)/255);
                        ESP_LOGI(EXAMPLE_TAG, "TPS: %d %%", TPS);
                    break;
            
                    case ID_ENGINE_FUL:
                        FUL = ((rx_msg.data[3]*100)/255);
  //                      ESP_LOGI(EXAMPLE_TAG, "FUEL LEVEL: %d %%", FUL);
                    break;
                    
                    case ID_ENGINE_ODO:
                        ODO = (3660+((256*rx_msg.data[3])+rx_msg.data[4]));
//                        ESP_LOGI(EXAMPLE_TAG, "ODO: %d Km", ODO);                     
                    break;

                    case ID_ENGINE_LBD:
                       // LBD = ((2/655536)*((256*rx_msg.data[3])+(rx_msg.data[4])));
                       LBD_A = (rx_msg.data[3]);
                        ESP_LOGI(EXAMPLE_TAG, "code1: %d", LBD_A); //190 - BE //128 - 80 //254 //                      
                       LBD_B = (rx_msg.data[4]);
                        ESP_LOGI(EXAMPLE_TAG, "code1: %d", LBD_B); //190 - BE //128 - 80 //254 //      
                        LBD =  ((2/655536)*(256*(LBD_A)+(LBD_B)));
                    break;

                    case ID_ENGINE_RTM:
                        RTM = ((rx_msg.data[3]*256)+rx_msg.data[4]);
/*                        ESP_LOGI(EXAMPLE_TAG, "RUNTIME: %d", RTM);*/
                    break;

                    case ID_ENGINE_ETH:
                        RTM = ((100/255)*rx_msg.data[3]);
//                        ESP_LOGI(EXAMPLE_TAG, "ETHANOL: %d", ETH);
                    break;
                }
            }
        }
		else {     

            ESP_LOGI(EXAMPLE_TAG, "ERRO");

		}   
            
        
}

void app_main()
{
    rx_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(can_receive_task, "CAN_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
   

    //Install and start CAN driver
    ESP_ERROR_CHECK(can_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");
    ESP_ERROR_CHECK(can_start());
    ESP_LOGI(EXAMPLE_TAG, "Driver started");

    vTaskDelay(portMAX_DELAY);

    //Stop and uninstall CAN driver
    ESP_ERROR_CHECK(can_stop());
    ESP_LOGI(EXAMPLE_TAG, "Driver stopped");
    ESP_ERROR_CHECK(can_driver_uninstall());
    ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

    //Cleanup
    vSemaphoreDelete(rx_sem);
}

