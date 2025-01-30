
#include <Arduino.h>
#include <logger.h>
#include <utils.h>
#include <main.h>

#define LOG_ENABLE 1
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

//主机IO口操作
#include <ArduinoModbus.h>
#include <USR_IO.h>
USR_IO usr_io_instance;

//当前等待执行的命令队列
QueueHandle_t queue;
const int queueLen = 3;

//当前正在执行的命令
static Command curr_command;

void setup() {

   //创建消息队列
   queue = xQueueCreate(queueLen, sizeof(Command));
   
   //初始化EEMROM
   preferences.begin("app", false);
    
   xTaskCreate(
    xTask_watchdog,   /* 任务函数 看门狗*/
    "xTask_watchdog", /* 名称 */
    4096,             /* 堆栈大小. */
    NULL,             /* 参数输入传递给任务的*/
    2,                /* 任务的优先级*/
    NULL);            /* 任务所在核心 */
   
    

    //打开串口
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, RX1PIN, TX1PIN);

    //创建BT链接
    String mac = preferences.getString("MAC","EEEE");
    if(mac=="EEEE"){
        SerialBT.begin("KP-000000");
        mac = SerialBT.getBtAddressString(); 
        mac.replace(":","");
        preferences.putString("MAC", mac);
    }
    SerialBT.begin("KP-"+mac);
    SerialBT.setPin("241212");

    //初始化本机IO  
    usr_io_instance.init();
    delay(500);

    xTaskCreate(
      xTask_handle,    /* 任务函数 处理上位机消息 */
      "xTask_handle",  /* 名称 */
      4096,             /* 堆栈大小. */
      NULL,             /* 参数输入传递给任务的*/
      10,                /* 任务的优先级*/
      NULL); 

    //初始化日志记录
    LOGGER::begin(&SerialBT);
    LOGGER::enable(true);
    LOGGER::println("Run...");
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
        LOGGER::println("xQueueSend...");

        xQueueSend(queue, &cmd, portMAX_DELAY);
      }

  }else{  //协议校验失败
      
      SerialBT.println("E001"); 
  }

}


void xTask_watchdog(void *xTask1) {
  pinMode(RELOAD_DOG_PIN, OUTPUT);
  while (1) {

    digitalWrite(RELOAD_DOG_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    digitalWrite(RELOAD_DOG_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));

  }

}


bool unpack_command(Command *command){
  String line = String(command->msg);
  line.trim();

  LOGGER::print("unpack:");
  LOGGER::println(line);
  
  //获取节点名称
  String node = line.substring(0,line.indexOf('@'));
  if(node.length()>0 && node.length()<=5){
    LOGGER::print("node:");
    LOGGER::print(node+",");
    node.toCharArray(command->node, sizeof(command->node));
  }

  //获取功能码
  String func = line.substring(line.indexOf('@')+1,line.indexOf('@')+6);
  if(func.length()>0 && func.length()<=6){
    LOGGER::print("func:");
    LOGGER::print(func+",");
    func.toCharArray(command->func, sizeof(command->func));
  }

  //获取校验码
  String crc = line.substring(line.indexOf('&')+1,line.indexOf('&')+9);
  if(crc.length()>0 && crc.length()<=8){
    
    char buffer[50];
    crc.toCharArray(buffer, sizeof(buffer)); 

    // 使用 strtol 将十六进制字符串转换为长整数
    long result = strtol(buffer, NULL, 16);

    // 检查转换是否成功
    if (result == 0L || buffer[0] != '0') {
       return false;
    } 

    LOGGER::print("crc:");
    LOGGER::print(result);
    LOGGER::println("");

  }
  
  //计算校验码
  String str = line.substring(0,line.indexOf('&'));
  uint8_t data[str.length()];
  str.getBytes(data, sizeof(data));  
  uint16_t len = sizeof(data) / sizeof(data[0]);

  // 计算Modbus CRC16
  uint16_t modbusCRC =UTILS::modbus_crc16(data, len);
  
  LOGGER::print("cmp:");  
  LOGGER::print(str);  
  LOGGER::print(","); 
  LOGGER::print(modbusCRC); 
  LOGGER::println("");

  return line.length() >0 && line.lastIndexOf('#')>0;
}

String pack_command(Command *command){
  
  String line = String(command->msg);
  line.trim();
  switch (command->state)
  {
    case COMMAND_STATE::EXECUTING :
      line.replace("#","~"); //执行中
      break;  
    case COMMAND_STATE::SUCCESS :
      line.replace("#","!"); //成功
      break;
    default:
      line.replace("#","?"); //失败
      String res = String(command->res);//错误码  
      res.trim();
      if(res.length() >0){
          line+=res;
      }
      break;
  }
  return line;
}

//执行上位机消息
void xTask_handle(void *xTask1){

  Command cmd;
  while (1) {
    if (xQueueReceive(queue, &cmd, portMAX_DELAY)) {
      LOGGER::println("xQueueReceive...");
      curr_command = cmd;
      curr_command.state = COMMAND_STATE::EXECUTING;
      String func = String(cmd.func);
      bool success = false;
      if(func.startsWith("fd")){
          success = handle_do(&curr_command);
      }else if(func.startsWith("f0000")){
          usr_io_instance.do_set(1,1,0); 
      }

      if(success){
        curr_command.state = COMMAND_STATE::SUCCESS;
      }else{
        curr_command.state = COMMAND_STATE::ERROR;
      }
      
    }
  }

}

void do_set(int gpio,char level){
  if(gpio<10){
      usr_io_instance.do_set(1,gpio, level); 
  }else if(gpio<20){
      usr_io_instance.do_set(2,gpio-10, level); 
  }else if(gpio<30){
      usr_io_instance.do_set(3,gpio-30, level); 
  }
}

bool handle_do(Command *cmd){
  String func = String(cmd->func);
  String msg = String(cmd->msg);
  String sub = func.substring(2,func.length());

  int gpio = sub.toInt();
  String t = msg.substring(msg.indexOf("t:")+2,msg.indexOf("&"));

  LOGGER::println("t:"+t);
  //LOGGER::println("gpio:"+gpio);
  

  unsigned long startTime = millis();
  
  do_set(gpio,1);

  if(t.length()>0){ // 延迟
     float f = t.toFloat();
     const TickType_t xFrequency = pdMS_TO_TICKS(f ); // 将 ms 转换为系统时钟节拍数
     TickType_t xLastWakeTime = xTaskGetTickCount(); 
     vTaskDelayUntil(&xLastWakeTime, xFrequency); 
     do_set(gpio,0);
  }
  unsigned long elapsedTime = millis() - startTime;
  //LOGGER::println(""+elapsedTime);

  return true;
}



bool handle_granule(Command *cmd){
  String func = String(cmd->func);
  String sub = func.substring(2,func.length());

  return true;
}

