/************************************************************************************
-----------------------ASSIGNMENT 3 BY GODFREY INYAMA--------------------------------
---------------------PRESENTED ON THE 7TH OF APRIL 2022-----------------------------
---------------------SUBMITTED ON THE 11TH OF APRIL 2022-----------------------------
                    AIM:DEMONSTRATION OF FREERTOS
This code is developed to show how a freertos system works, by performing certain tasks using queues, struct, and a mutex semaphore.
Task 1: Output of Signal B from Assignment 1
Task 2: Monitoring of one digital input
Task 3: Measuring the frequency of a 3.3v square wave signal
Task 4: Reading one analogue input
Task 5: Computing the average of the last four analogue signals
Task 6: Executing _asm_ _ volatile_ ("nop");
Task 7: Performing  if (average_analogue_in > half of maximum range for 
analogue input):
    error_code = 1
else:
    error_code = 0
Task 8: Vizualing the error with an LED
Task 9: Logging the following every 5sec
Task 10: Printing task 9 only when pushbutton is HIGH
State of the digital input (pushbutton / switch);
Frequency value (Hz, as an integer);
Filtered analogue input.
**************************************************************************************/

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 1
#else
#define ARDUINO_RUNNING_CORE 0
#endif

#define LED 21                             //Task1 WatchDog Signal from Signal B in Assignment1 which is 50us
#define error_LED 5                        // LED to vizualize the error_code from Task 7 in Task 8
#define digital_input 4                    //Task2 Digital Input Monitor using one switch
#define squarewave_reader 35               //Task3 3.3v square wave for reading frequency
#define analogue_reader 34                 //Task4 Analogue Input reader using a potentiometer

float analogue_input_task4[4] = {0,0,0,0}; //array to store 4 cocurrent values from task 4 in a map
float task5_average = 0;                   //initializing task5_average value as LOW

int error_code = 0;                        //initializing error value as LOW
int monitor_task2 = 0;                     //initializing task5_average value as false
int Counter = 0;                           // Counter to record the number of cycle went through by the Cyclic Executive
int logvaluetask9 = 0;

//======================================================//
//-----------variables for Task 3-----------------------//
//======================================================//
int freq_flag = 0; //the flag to indicate if the signal is low then went high
int task3_frequency = 0; //the value of the frequency in Hz
int frequency_count = 0; //The number of pulses over the time period
int unfiltered_frequency;
int unfiltered_frequency_old;
unsigned long start_timeF;
unsigned long currentTime;
int task4_average =0;


//Rates (time) for each task performance
const int Time_Task1 = 14;                     //actual value = 14.45
const int Time_Task2 = 200; 
const int Time_Task3 = 1000;
const int Time_Task4 = 42;                     //actual value = 41.67
const int Time_Task5 = 42;                     //actual value = 41.67
const int Time_Task6 = 100;
const int Time_Task7 = 33;                     //actual value = 33.33
const int Time_Task8 = 33;                     //actual value = 33.33 
const int Time_Task9 = 5000;
const int B = 50;                              //time of HIGH from Assignment 1

float task5_av;

//===============================================================//
//------------Function models to be called in the loop-----------// 
//===============================================================//
void task1( void *pvParameters );
void task2( void *pvParameters );
void task3( void *pvParameters );
void task4( void *pvParameters );
void task5( void *pvParameters );
void task6( void *pvParameters );
void task7( void *pvParameters );
void task8( void *pvParameters );
void task9( void *pvParameters );


static QueueHandle_t queue;                      //queue initilization
static SemaphoreHandle_t mutex;                  //type of semaphore in use

//global struct to store task2, task3, and task 5 
 typedef struct s_data {                     
 byte  monitor_task2            : 1;
 unsigned int task3_frequency   : 10;
 unsigned int task5_av          : 12;
};

s_data datas;

unsigned int       queuerec1 =          0;    //1st analogue value from queue
unsigned int       queuerec2 =          0;    //2nd analogue value from queue
unsigned int       queuerec3 =          0;    //3rd analogue value from queue
unsigned int       queuerec4 =          0;    //4th analogue value from queue

void setup() {
// initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

// queue setup with 4 as the stack size
  queue = xQueueCreate( 4, sizeof( float ) );

  mutex = xSemaphoreCreateMutex();
  
//=========== Ten tasks setup to run independently===========//
  xTaskCreatePinnedToCore(
    task1
    ,  "Task 1"                   // A name just for humans
    ,  4096                       // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1                          // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL
    ,  ARDUINO_RUNNING_CORE);     // Tasks running on the first core with the other for the CPU
    
  xTaskCreatePinnedToCore(
    task2
    ,  "Task 2"   
    ,  4096  
    ,  NULL
    ,  1  
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    task3
    ,  "Task 3"   
    ,  4096  
    ,  NULL
    ,  1  
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    task4
    ,  "Task 4"   
    ,  4096  
    ,  NULL
    ,  6  
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    task5
    ,  "Task 5"   
    ,  4096  
    ,  NULL
    ,  1  
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    task6
    ,  "Task 6"   
    ,  4096  
    ,  NULL
    ,  1  
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    task7
    ,  "Task 7"   
    ,  4096  
    ,  NULL
    ,  1  
    ,  NULL
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    task8
    ,  "Task 8"  
    ,  4096  
    ,  NULL
    ,  1  
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    task9
    ,  "Task 9 & 10"   
    ,  4096 
    ,  NULL
    ,  1 
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

}

void loop(){}


//=================================//
//-------function for task 1------//
//================================//
void task1(void *pvParameters) {
  (void) pvParameters;

  // initialize digital LED on pin 21 as an output.
  pinMode(LED, OUTPUT);
  
  for (;;)  //loop forever
{
    digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
    delayMicroseconds(B);      // one tick delay (50us) in between reads for stability
    digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay(Time_Task1);    // one tick delay (14ms) in between reads for stability
  }
}
 

//=================================//
//-------function for task 2------//
//================================//
void task2(void *pvParameters) {
  (void) pvParameters;
  for (;;)
  {
  xSemaphoreTake(mutex, portMAX_DELAY); // using a semaphore to protect the data from the structure ( wait function of the semaphore)
  if (digitalRead (digital_input)){
    datas.monitor_task2 = 1;
  }
  else{
    datas.monitor_task2 = 0;
  }
  xSemaphoreGive(mutex);               // end of the use of the semaphore (signal functio of the semaphore)
  vTaskDelay(Time_Task2 / portTICK_PERIOD_MS);  // one tick delay (Time_Task2) in between reads for stability
  }
}


//=================================//
//-------function for task 3------//
//================================//
void task3(void *pvParameters) {
  (void) pvParameters;
  for (;;)
  {
  unfiltered_frequency=digitalRead(squarewave_reader);
  unfiltered_frequency_old= unfiltered_frequency;
  frequency_count=0;    
 

  start_timeF= micros(); //define the start time of the clock
  currentTime=micros();
  xSemaphoreTake(mutex, portMAX_DELAY);
    while ((currentTime-start_timeF) < 40000){                 //40000uS is the ideal time for detecting frequency that keeps errors within a tolerance of 2.5%
       
      unfiltered_frequency=digitalRead(squarewave_reader);
      if (unfiltered_frequency_old != unfiltered_frequency){  //add one to the value of the frequency counter if frequncy input was high but was low.
        frequency_count++;
        unfiltered_frequency_old=unfiltered_frequency;}
     
      currentTime=micros();
        
    } 
    
    if (micros()>= start_timeF +40000){                       //increasing 40000uS to 1sec
      
        datas.task3_frequency =frequency_count*25/2;          //actual frequency read from the frequency generator 
        xSemaphoreGive(mutex);
      }
      
      vTaskDelay(Time_Task3 / portTICK_PERIOD_MS);  // one tick delay (Time_Task3) in between reads for stability
  }
  
}


//=================================//
//-------function for task 4------//
//================================//
void task4(void *pvParameters) {
  (void) pvParameters;
  for (;;)
  {

  for(int i=1; i<4; i++){
    analogue_input_task4[i] = analogue_input_task4[1];
  }
  analogue_input_task4[1] = analogRead (analogue_reader);    //reading one analogue value
  task4_average = analogue_input_task4[1];
  xQueueSend(queue,&task4_average,1);                        // send the value inside the queue
  vTaskDelay(Time_Task4 / portTICK_PERIOD_MS);               // one tick delay (Time_Task4) in between reads for stability
  }
}


//=================================//
//-------function for task 5------//
//================================//
void task5(void *pvParameters) {
  (void) pvParameters;
  int queuerec1;
  int queuerec2;
  int queuerec3;
  int queuerec4;
  for(;;){
//=============================================================================//
//=========setting the values of the analogue values based on last value=======//
//=============================================================================//
    queuerec4 = queuerec3; 
    queuerec3 = queuerec2;
    queuerec2 = queuerec1;
    xQueueReceive(queue,&queuerec1,portMAX_DELAY); // reception of the value in the queue to protect the data
    
    xSemaphoreTake(mutex,portMAX_DELAY);
    datas.task5_av = (queuerec1 + queuerec2 + queuerec3 + queuerec4)/4;
    xSemaphoreGive(mutex);

vTaskDelay(Time_Task5 / portTICK_PERIOD_MS);  // one tick delay (Time_Task5) in between reads for stability
  }

}

//=================================//
//-------function for task 6------//
//================================//
void task6(void *pvParameters) {
  (void) pvParameters;
  for (;;)
  {
  //Serial.println ("Task 6");
  for(int i=0; i<1000; i++){
    __asm__ __volatile__ ("nop");
  }
  vTaskDelay(Time_Task6 / portTICK_PERIOD_MS);  // one tick delay (Time_Task6) in between reads for stability
}
}


//=================================//
//-------function for task 7------//
//================================//
void task7(void *pvParameters) {
  (void) pvParameters;
  for (;;)
  {

  int max_range = 4096;                //maximum range when varied on the potentiometer
  int half_max = max_range / 2;        //half of maximum range calculated
  xSemaphoreTake(mutex,portMAX_DELAY); // protection of the average_analog_value 
  if(datas.task5_av > half_max){        //if analogue average value calculated is greater than half the maximum range show error code by Turning ON LED else Turn OFF LED
    error_code = 1;
  }
  else{
    error_code = 0;
  }
  xSemaphoreGive(mutex);
  vTaskDelay(Time_Task7 / portTICK_PERIOD_MS);  // one tick delay (Time_Task7) in between reads for stability
}
}


//=================================//
//-------function for task 8------//
//================================//
void task8(void *pvParameters) {
  (void) pvParameters;
  
  pinMode(error_LED, OUTPUT);
  for (;;)
  {
  //Serial.println ("Task 8");
 
  if(error_code){
    digitalWrite(error_LED,HIGH);     //turned on because task5_average > half_max
  }
  else
   {
   digitalWrite(error_LED,LOW);       //turned on because task5_average < half_max
  }
  vTaskDelay(Time_Task8 / portTICK_PERIOD_MS);  // one tick delay (Time_Task8) in between reads for stability
}
}


//============================================//
//-------function for task 9 and task 10------//
//============================================//
void task9(void *pvParameters) { 
  (void) pvParameters;
  for (;;)
  {
  //========================================================================================================================// 
  //---printing out the values of digital button state, frequency value, and filtered analogue input after every 5seconds---//
  //-------------------------------------when button state from task 2 is HIGH----------------------------------------------//
  //========================================================================================================================//

  if(digitalRead (digital_input) ){ 
  xSemaphoreTake(mutex, portMAX_DELAY); //protection of shared data
  Serial.println((String)datas.monitor_task2+ "," + (String) datas.task3_frequency + "Hz," + (String)datas.task5_av);
  xSemaphoreGive(mutex);
  }
  vTaskDelay(Time_Task9 / portTICK_PERIOD_MS);  // one tick delay (Time_Task9) in between reads for stability
}
}
