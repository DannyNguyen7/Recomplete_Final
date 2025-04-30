#include <dummy.h>

#include "driver/gpio.h"
#include "driver/twai.h"
#include "BluetoothSerial.h"
#include <string>
#include <stdlib.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

#include "VirtuinoCM.h"
VirtuinoCM virtuino;               
#define V_memory_count 6          // the size of V memory. You can change it to a number <=255)
float V[V_memory_count];           // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.


#define RECEIVE_CAN_ID      0x100
#define TRANS_CAN_ID        0x101
#define V_MIN               0.497f
#define V_MAX               3
#define ADC_MAX             4095
//-------------------------------------
#define _Base_Freq          10   // Hz
//-------------------------------------
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
#define _Sys_CLK            80000000  // Hz
#define _BaseTimer_CLK      1000000  // Hz
#define _BaseTimer_PRE      (_Sys_CLK / _BaseTimer_CLK) 
#define _BaseTimer_TOP      (_BaseTimer_CLK / _Base_Freq)

hw_timer_t * BaseTimer = NULL;

twai_message_t rx_message, tx_message;

bool Gui_flag = false;

void IRAM_ATTR onBaseTimer() 
{
  portENTER_CRITICAL_ISR(&timerMux);
  Gui_flag = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

float adc_to_voltage(uint32_t adc_value) {
    return (float)adc_value * 3.3 / ADC_MAX; // Chuyển đổi giá trị ADC sang điện áp
}

float calculate_position(uint16_t adc_value) {
    float voltage = adc_to_voltage(adc_value);
    float position = (voltage - V_MIN) / (V_MAX - V_MIN) * 100;
    if (position < 0) position = 0;
    if (position > 100) position = 100;
    return position;
}

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  SerialBT.begin("ESP32SEVCON"); //Bluetooth device name   // enable bluetooth
  virtuino.begin(onReceived,onRequested,256);  // enable virtuino

  BaseTimer = timerBegin(0, _BaseTimer_PRE, true); 
  timerAttachInterrupt(BaseTimer, &onBaseTimer, true);
  timerAlarmWrite(BaseTimer, _BaseTimer_TOP, true);
  timerAlarmEnable(BaseTimer);  

  disableCore0WDT(); 
  disableCore1WDT(); 

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_4, TWAI_MODE_NORMAL);
  // TX CAN Controller D5 - TX pin module CAN Transceiver CAN transmit data input
  // RX CAN Controller D4 - RX pin module CAN Transceiver CAN receive data output
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  //Khởi tạo giá trị ban đầu cho TX_Frame
  tx_message.identifier       = TRANS_CAN_ID;
  tx_message.data_length_code = 8;
  tx_message.data[0]          = 0;      //desire TPS
  tx_message.data[1]          = 0;      // KI
  tx_message.data[2]          = 0;      // KP
  tx_message.data[3]          = 0;      // KD

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
  {
    Serial.print("Driver installed\n");
  } 
  else 
  {
    Serial.print("Failed to install driver\n");
    return;
  }
  
  if (twai_start() == ESP_OK) 
  {
    Serial.print("Driver started\n");
  } 
  else 
  {
    Serial.print("Failed to start driver\n");
    return;
  }
}
//varialble for app
float actual_TPS, desire_TPS, K_p, K_i, K_d;
int last_button_state = 0;
void loop() 
{
  virtuinoRun();    //Truyền dữ liệu qua bluetooth
  if(twai_receive(&rx_message, pdMS_TO_TICKS(1)) == ESP_OK)
  {
    Serial.println();
    if(rx_message.identifier == RECEIVE_CAN_ID)
    {
      // nhận giá trị vào biến
      Serial.print("ID: 0x");
      Serial.print(rx_message.identifier, HEX);

      Serial.print("Data: ");
      for (int i = 0; i < rx_message.data_length_code; i++) {
      Serial.print(rx_message.data[i], HEX);
      Serial.print(" ");
      }
      Serial.println();
      uint16_t TPS1 = (rx_message.data[1] << 8) | rx_message.data[0];
      actual_TPS = calculate_position(TPS1);
      V[0] = actual_TPS;
    }
  }
  if(V[2] != last_button_state)//Đọc được nút nhấn từ app V[2]
  {
    //chốt giá trị
    //Khúc này chốt các dữ liệu ki kp và kd và dersire_TPS để truyền ra CAN
    //
    
    tx_message.data[1] = V[3];
    tx_message.data[2] = V[4];
    tx_message.data[3] = V[5];
    
    K_i =         (float)V[3]*10/256;
    K_p =         (float)V[4]*10/256;
    K_d =         (float)V[5]*10/256;
    last_button_state = V[2];
    
    Serial.print(actual_TPS); Serial.print("\t");
    Serial.print(K_i); Serial.print("\t");
    Serial.print(K_p); Serial.print("\t");
    Serial.print(K_d); Serial.println("\t");
  }
  if(Gui_flag) //done
  {
    //xóa cờ, cập nhật dữ liệu actual
    V[0] = actual_TPS;   
    // Serial.print(desire_TPS); Serial.println("\t");
    desire_TPS =  V[1];
    tx_message.data[0] = V[1];
    if(twai_transmit(&tx_message, pdMS_TO_TICKS(10)) == ESP_OK)
    {
      Serial.println(" Truyen OK");
    }
    portENTER_CRITICAL_ISR(&timerMux);
    Gui_flag = false;
    portEXIT_CRITICAL_ISR(&timerMux);
  }
}
//============================================================== onCommandReceived
//==============================================================
/* This function is called every time Virtuino app sends a request to server to change a Pin value
 * The 'variableType' can be a character like V, T, O  V=Virtual pin  T=Text Pin    O=PWM Pin 
 * The 'variableIndex' is the pin number index of Virtuino app
 * The 'valueAsText' is the value that has sent from the app   */
 void onReceived(char variableType, uint8_t variableIndex, String valueAsText){     
    if (variableType=='V'){
        float value = valueAsText.toFloat();        // convert the value to float. The valueAsText have to be numerical
        if (variableIndex<V_memory_count) V[variableIndex]=value;              // copy the received value to arduino V memory array
    }
}

//==============================================================
/* This function is called every time Virtuino app requests to read a pin value*/
String onRequested(char variableType, uint8_t variableIndex){     
    if (variableType=='V') {
    if (variableIndex<V_memory_count) return  String(V[variableIndex]);   // return the value of the arduino V memory array
  }
  return "";
}
//==============================================================
void virtuinoRun(){
  while (SerialBT.available()) {
      char tempChar=SerialBT.read();
      if (tempChar==CM_START_CHAR) {               // a new command is starting...
            virtuino.readBuffer=CM_START_CHAR;     // copy the new command to the virtuino readBuffer
            virtuino.readBuffer+=SerialBT.readStringUntil(CM_END_CHAR);
            virtuino.readBuffer+=CM_END_CHAR;
            //if (debug) Serial.println("\nCommand= "+virtuino.readBuffer);
            String* response= virtuino.getResponse();    // get the text that has to be sent to Virtuino as reply. The library will check the inptuBuffer and it will create the response text
            //if (debug) Serial.println("Response : "+*response);
            SerialBT.print(*response);
            break; 
        }
  }
}


 //============================================================== vDelay
  void vDelay(int delayInMillis){long t=millis()+delayInMillis;while (millis()<t) virtuinoRun();}