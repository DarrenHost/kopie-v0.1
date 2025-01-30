
#include <Arduino.h>
#include <main.h>
#include <logger.h>
//OneButton btnReload(36, true);
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

#include <ArduinoModbus.h>

#include <BluetoothSerial.h>
BluetoothSerial SerialBT;


// #include <ArduinoModbus.h>
#include <USR_IO.h>
USR_IO usr_io_instance;

QueueHandle_t queue;
const int queueLen = 3;



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
        
        Serial.println("EEPROM");

        SerialBT.begin("KP-12");
        mac = SerialBT.getBtAddressString(); 
        mac.replace(":","");
        preferences.putString("MAC", mac);
    }
    
    SerialBT.begin("KP-"+mac);
        
    
    SerialBT.setPin("241212");

    usr_io_instance.init();
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

  LOGGER::print("unpack_command:");
  LOGGER::println(line);
  
  String node = line.substring(0,line.indexOf('@'));
  if(node.length()>0 && node.length()<=5){
    LOGGER::print("node:");
    LOGGER::print(node+",");
    node.toCharArray(command->node, sizeof(command->node));
  }

  String func = line.substring(line.indexOf('@')+1,line.indexOf('@')+6);
  if(func.length()>0 && func.length()<=6){
    LOGGER::print("func:");
    LOGGER::print(func+",");
    func.toCharArray(command->func, sizeof(command->func));
  }

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
    //LOGGER::print("crc:");
    //LOGGER::println(result);

  }
  
  String str = line.substring(0,line.indexOf('&'));
  uint8_t data[str.length()];
  str.getBytes(data, sizeof(data));  

  //uint8_t data[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x06};
  uint16_t len = sizeof(data) / sizeof(data[0]);

  // 计算Modbus CRC16
  uint16_t modbusCRC = modbus_crc16(data, len);
  
  Serial.print("modbusCRC:");  
  Serial.print(str);  
  Serial.print(","); 
  Serial.println(modbusCRC,HEX);

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
      if(func.startsWith("fd")){
          handle_do(&curr_command);
      }else if(func.startsWith("f0000")){
          usr_io_instance.do_set(1,1,0); 
      }

      //logln("xQueueReceive...");

      //vTaskDelay(3000 / portTICK_PERIOD_MS);

      curr_command.state = COMMAND_STATE::SUCCESS;
      //SerialBT.println(cmd.msg);

    }
  }

}

bool handle_do(Command *cmd){
  String func = String(cmd->func);
  String sub = func.substring(2,func.length());

  int gpio = sub.toInt();
  LOGGER::println("sub:"+sub);
  //LOGGER::println("gpio:"+gpio);

  usr_io_instance.do_set(1,gpio, 1); 
  
  return true;
}

bool handle_io(Command *cmd){
  
  return true;
}

// CRC16校验函数
uint16_t modbus_crc16(uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF; // 初始值
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8; // 将数据字节与CRC值进行XOR操作
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) { // 检查最高位是否为1
        crc = (crc << 1) ^ 0xA001; // 如果是1，进行多项式运算
      } else {
        crc = crc << 1; // 如果不是，则左移
      }
    }
  }
  return crc; // 返回最终的CRC值
}

void xTask_modbus(void *xTask1) 
{
  

} 


