
#include <Arduino.h>

//OneButton btnReload(36, true);
#define LOG_ENABLE 
#define USR_ENABLE 
#define RELOAD_DOG_PIN 3
#define BOARD_M100_CONFIG

#ifdef BOARD_M100_CONFIG
  #define RX1PIN GPIO_NUM_33
  #define TX1PIN GPIO_NUM_32
  #define RELOAD_DOG_PIN 3
#endif

#include <Preferences.h>
Preferences preferences;

#include <BluetoothSerial.h>
BluetoothSerial SerialBT;

QueueHandle_t queue;
const int queueLen = 10;

enum COMMAND_STATE {
  EXECUTING = 0,
  SUCCESS = 88,
  ERROR = 44
};

typedef struct {
  unsigned int state;
  //char node[10];
  //String fun[8];
  //unsigned char crc;
  char msg[255];
} Command;



Command curr_command;

bool unpack_command(Command *command);
String pack_command(Command *command);
void reload_dog(void) ;
void xTask_watchdog(void *xTask1);
void xTask_handle(void *xTask1);

void setup() {

  preferences.begin("app", false);
    
   xTaskCreate(
    xTask_watchdog,   /* 任务函数 看门狗*/
    "xTask_watchdog", /* 名称 */
    4096,             /* 堆栈大小. */
    NULL,             /* 参数输入传递给任务的*/
    2,                /* 任务的优先级*/
    NULL);            /* 任务所在核心 */
   
    //创建消息队列
    queue = xQueueCreate(queueLen, sizeof(Command));

    //打开串口
    Serial.begin(115200);
    

    Serial1.begin(115200, SERIAL_8N1, RX1PIN, TX1PIN);

   
    String mac = preferences.getString("MAC","EEEE");
    if(mac=="EEEE"){
        
        SerialBT.begin("KP-12");
        mac = SerialBT.getBtAddressString(); 
        mac.replace(":","");
        preferences.putString("MAC", mac);
    }
    
    SerialBT.begin("KP-"+mac);
        
    
    SerialBT.setPin("241212");

    
    

    //初始化本机IO
    delay(1000);
 
    //usr_io_instance.log_on(1);
    //usr_io_instance.print();

    xTaskCreate(
    xTask_handle,    /* 任务函数 处理上位机消息 */
    "xTask_handle",  /* 名称 */
    4096,             /* 堆栈大小. */
    NULL,             /* 参数输入传递给任务的*/
    2,                /* 任务的优先级*/
    NULL); 

    Serial.println("Run...");
}

String readPacket(){
   String line;
   int time = 0;
   int readTimeOut = 1000;
   while(time<readTimeOut){
      while (SerialBT.available())
      {
          char ch = SerialBT.read(); 
          line.concat(ch);
          if(ch == '#'){
              return line;
          }
      }
      delay(10);
      time+=10;
   }
   return line;
}

void loop() {

   
   String line  = readPacket();
   line.trim();
   if(line.length()<1){ //空数据包，不处理
      return;
   }
   
    
    //转成指令结构体
  Command cmd;
  cmd.state = COMMAND_STATE::EXECUTING;
  line.toCharArray(cmd.msg, sizeof(cmd.msg));  

  //进行拆包
  bool unpack = unpack_command(&cmd);
  
  if(unpack){ //协议校验成功
      
      if(String(cmd.msg)==String(curr_command.msg)){ //重复接收数据，根据指令执行状态返回响应数据包
          String result = pack_command(&curr_command);
          SerialBT.println(result);  
      }else{

        //返回响应数据包
        String result = pack_command(&cmd);
        SerialBT.println(result);  
        
        //放入执行队列
        Serial.println("xQueueSend...");
        xQueueSend(queue, &cmd, portMAX_DELAY);
      }

  }else{  //协议校验失败
      
      SerialBT.println("E001"); 
  }



}


void xTask_watchdog(void *xTask1) {

  while (1) {
    reload_dog();
    delay(200);
  }

}

void reload_dog(void) {

  pinMode(RELOAD_DOG_PIN, OUTPUT);
  digitalWrite(RELOAD_DOG_PIN, 0);
  delay(10);
  digitalWrite(RELOAD_DOG_PIN, 1);
  delay(25);
  digitalWrite(RELOAD_DOG_PIN, 0);
  delay(10);
}

bool unpack_command(Command *command){
  String line = String(command->msg);
  line.trim();

  Serial.println("unpack_command:");
  Serial.println(line);

  return line.length() >0 && line.lastIndexOf('#')>0;
}

String pack_command(Command *command){
  
  String line = String(command->msg);
  line.trim();
  switch (command->state)
  {
    case COMMAND_STATE::SUCCESS :
      line.replace("#","!"); //成功
      break;
    case COMMAND_STATE::EXECUTING :
      line.replace("#","~"); //执行中
      break;  
    default:
      line.replace("#","?"); //失败
      break;
  }
  return line;
}
//接收上位机消息
void xTask_receive(void *xTask1) 
{
  
   

}

//执行上位机消息
void xTask_handle(void *xTask1){
  
  Command cmd;
  while (1) {
    if (xQueueReceive(queue, &cmd, portMAX_DELAY)) {
      
      curr_command = cmd;
      curr_command.state = COMMAND_STATE::EXECUTING;

      Serial.println("xQueueReceive...");

      vTaskDelay(3000 / portTICK_PERIOD_MS);

      curr_command.state = COMMAND_STATE::SUCCESS;
      //SerialBT.println(cmd.msg);

    }
  }

}

void xTask_modbus(void *xTask1) 
{
  

} 
