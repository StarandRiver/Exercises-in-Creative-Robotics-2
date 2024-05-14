#include<Dynamixel2Arduino.h>

//マイコンごとの設定
//今回は手元にあるOpenCM・OpenRB・Dynamixel Shieldを使用できるように設定
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL Serial
  #define BT_SERIAL Serial2
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2;
  const unsigned long SOFT_BUADRATE = 9600; // ソフトウェアシリアルのボーレート
#elif defined(ARDUINO_OpenCM904)
  #define DXL_SERIAL Serial3       // OpenCM9.04の拡張ボードのDynamixelとのシリアル通信の設定
  #define USB_SERIAL Serial
  #define BT_SERIAL Serial2
    const int DXL_DIR_PIN = 22;
  const bool SENSOR_IS_AVAILABLE = false;
#elif defined(ARDUINO_OpenCR)
  #define DXL_SERIAL Serial3
  #define USB_SERIAL Serial
  #define BT_SERIAL Serial2
  const int DXL_DIR_PIN = -1;
  const bool SENSOR_IS_AVAILABLE = true;
#endif

/*======使用するモーターを選択======*/
#define DXL_AX12A
//#define DXL_XC430
/*=================================*/

  const uint8_t DXL_CNT = 5;                 // モータの数
  const uint16_t DXL_INIT_VELOCITY = 100;     // モータの初期速度
  const uint16_t DXL_INIT_ACCELERATION = 100; // モータの初期加速度
  const unsigned long USB_BUADRATE = 57600;   // USBのボーレート
  const unsigned long BT_BUADRATE = 57600;    // Bluetoothデバイスのボーレート

#if defined(DXL_AX12A)
  const float DXL_PROTOCOL_VERSION = 1.0; // モータの通信プロトコル
  const uint16_t DXL_MAX_POSITION_VALUE = 850; // モータの最大値
  const uint16_t DXL_MIN_POSITION_VALUE = 150; // モータの最小値
  const unsigned long DXL_BUADRATE = 57600;  // Dynamixelのボーレート
#elif defined(DXL_XC430)
  const float DXL_PROTOCOL_VERSION = 2.0; // モータの通信プロトコル
  const uint16_t DXL_MAX_POSITION_VALUE = 850; // モータの最大値
  const uint16_t DXL_MIN_POSITION_VALUE = 150; // モータの最小値
  const unsigned long DXL_BUADRATE = 1000000;  // Dynamixelのボーレート
#endif

uint16_t g_dxl_present_velocities[DXL_CNT+1]; // Dynamixelの速度(モータの数+1)
uint16_t g_dxl_present_accelerations[DXL_CNT+1]; // Dynamixelの加速度(モータの数+1)
uint16_t g_dxl_pos[DXL_CNT+1];                    // Dynamixelの目標位置(モータの数+1)
bool g_dxl_is_connected[DXL_CNT+1];             // Dynamixelが接続されているかどうかの判別(モータの数+1)

  bool g_torque_is_on = false; // Dynamixelがトルクオンかどうか
  String g_read_line = "";     // シリアル通信で受け取るコマンド
  char g_cmd_word = '\0';      // コマンドのキーワード
  uint16_t g_cmd_args[128];    // コマンドの引数

  // Dynamixel2Arduinoクラスのインスタンス化
  Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

  using namespace ControlTableItem;

  /*コマンドの読み取り部*/
  int8_t readCommand()
  {
    int read_line_length = g_read_line.length();
    if (3 <= read_line_length && g_read_line.charAt(0) == '[' && g_read_line.charAt(read_line_length - 1) == ']')
    {
      g_cmd_word = g_read_line.charAt(1);
      int8_t arg_max_index = -1;
      int8_t elm_begin_index = 3;
      int8_t elm_end_index = 3;

      // カンマ区切りごとに数字を取り出す
      // カンマ，数字以外が含まれていた場合，エラーとして扱い-1を返す
      for (int line_i = 3; line_i < read_line_length; line_i++)
      {
        elm_end_index++;
        if (g_read_line.charAt(line_i) == ',' || line_i == read_line_length - 1)
        {
          arg_max_index++;
          g_cmd_args[arg_max_index] = g_read_line.substring(elm_begin_index, elm_end_index).toInt();
          elm_begin_index = elm_end_index;
        }
        else if (g_read_line.charAt(line_i) < '0' && '9' < g_read_line.charAt(line_i))
        {
          g_cmd_word = '\0';
          return -1;
        }
      }
      return arg_max_index;
    }
    g_cmd_word = '\0';
    return -1;
}

/*Dynamixelの速度，加速度の設定*/
void set_accel_velocity(){
  for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
  {
    if (!g_dxl_is_connected[dxl_i])
    {
      continue;
    }
    if (1.5 <= DXL_PROTOCOL_VERSION)
    {
      dxl.writeControlTableItem(PROFILE_VELOCITY, dxl_i, g_dxl_present_velocities[dxl_i]);
      dxl.writeControlTableItem(PROFILE_ACCELERATION, dxl_i, g_dxl_present_accelerations[dxl_i]);
    }
    else
    {
      dxl.writeControlTableItem(MOVING_SPEED, dxl_i, g_dxl_present_velocities[dxl_i]);
    }
  }
  return ;
}
/*モーションの配置*/
void forward()
{
  int forward_poster_cnt = 3;                        // 動作の数を入力
  int forward_poster[forward_poster_cnt][DXL_CNT] = {// モータ角度
                                                     {512, 512, 512, 512, 512},
                                                     {256, 256, 256, 256, 256},
                                                     {128, 128, 128, 128, 128}};
  int forward_poster_delay[forward_poster_cnt] = {200, 200, 200}; // 動作間の時間

  // トルクONの時の動作(速度・加速度の設定->コマンドの入力の有無の判断->動作)
  if (g_torque_is_on)
  {
    set_accel_velocity();
    while (0)
    {
      for (int pos_i = 1; pos_i <= forward_poster_cnt; pos_i++)
      {
        if (0 < USB_SERIAL.available() || 0 < BT_SERIAL.available())
        { // シリアル入力がある場合は動作を停止
          return;
        }
        else
        { 
          for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
          {
            g_dxl_pos[dxl_i] = forward_poster[pos_i][dxl_i];
            dxl.setGoalPosition(dxl_i, forward_poster[pos_i][dxl_i]);
          }
          delay(forward_poster_delay[pos_i]);
        }
      }
    }
  }
  else
  {
    for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
    {
      if (!g_dxl_is_connected[dxl_i])
      {
        continue;
      }
      g_dxl_pos[dxl_i] = constrain(uint16_t(dxl.getPresentPosition(dxl_i)), DXL_MIN_POSITION_VALUE, DXL_MAX_POSITION_VALUE);
    }
  }
  g_cmd_word = '\0';
  delay(5);
}

void back()
{
  int back_poster_cnt = 3;                     // 動作の数を入力
  int back_poster[back_poster_cnt][DXL_CNT] = {// モータ角度
                                               {512, 512, 512, 512, 512},
                                               {256, 256, 256, 256, 256},
                                               {128, 128, 128, 128, 128}};
  int back_poster_delay[back_poster_cnt] = {200, 200, 200}; // 動作間の時間

  // トルクONの時の動作(速度・加速度の設定->コマンドの入力の有無の判断->動作)
  if (g_torque_is_on)
  {
    set_accel_velocity();
    while (0)
    {
      for (int pos_i = 1; pos_i <= back_poster_cnt; pos_i++)
      {
        if (0 < USB_SERIAL.available() || 0 < BT_SERIAL.available())
        { // シリアル入力がある場合は動作を停止
          return;
        }
        else
        { 
          for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
          {
            g_dxl_pos[dxl_i] = back_poster[pos_i][dxl_i];
            dxl.setGoalPosition(dxl_i, back_poster[pos_i][dxl_i]);
          }
          delay(back_poster_delay[pos_i]);
        }
      }
    }
  }
  else
  {
    for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
    {
      if (!g_dxl_is_connected[dxl_i])
      {
        continue;
      }
      g_dxl_pos[dxl_i] = constrain(uint16_t(dxl.getPresentPosition(dxl_i)), DXL_MIN_POSITION_VALUE, DXL_MAX_POSITION_VALUE);
    }
  }
  g_cmd_word = '\0';
  delay(5);
}

void left()
{
  int left_poster_cnt = 3;                     // 動作の数を入力
  int left_poster[left_poster_cnt][DXL_CNT] = {// モータ角度
                                               {512, 512, 512, 512, 512},
                                               {256, 256, 256, 256, 256},
                                               {128, 128, 128, 128, 128}};
  int left_poster_delay[left_poster_cnt] = {200, 200, 200}; // 動作間の時間

  // トルクONの時の動作(速度・加速度の設定->コマンドの入力の有無の判断->動作)
  if (g_torque_is_on)
  {
    set_accel_velocity();
    while (0)
    {
      for (int pos_i = 1; pos_i <= left_poster_cnt; pos_i++)
      {
        if (0 < USB_SERIAL.available() || 0 < BT_SERIAL.available())
        { // シリアル入力がある場合は動作を停止
          return;
        }
        else
        { //
          for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
          {
            g_dxl_pos[dxl_i] = left_poster[pos_i][dxl_i];
            dxl.setGoalPosition(dxl_i, left_poster[pos_i][dxl_i]);
          }
          delay(left_poster_delay[pos_i]);
        }
      }
    }
  }
  else
  {
    for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
    {
      if (!g_dxl_is_connected[dxl_i])
      {
        continue;
      }
      g_dxl_pos[dxl_i] = constrain(uint16_t(dxl.getPresentPosition(dxl_i)), DXL_MIN_POSITION_VALUE, DXL_MAX_POSITION_VALUE);
    }
  }
  g_cmd_word = '\0';
  delay(5);
}

void right()
{
  int right_poster_cnt = 3;                      // 動作の数を入力
  int right_poster[right_poster_cnt][DXL_CNT] = {// モータ角度
                                                 {512, 512, 512, 512, 512},
                                                 {256, 256, 256, 256, 256},
                                                 {128, 128, 128, 128, 128}};
  int right_poster_delay[right_poster_cnt] = {200, 200, 200}; // 動作間の時間

  // トルクONの時の動作(速度・加速度の設定->コマンドの入力の有無の判断->動作)
  if (g_torque_is_on)
  {
    set_accel_velocity();
    while (0)
    {
      for (int pos_i = 1; pos_i <= right_poster_cnt; pos_i++)
      {
        if (0 < USB_SERIAL.available() || 0 < BT_SERIAL.available())
        { // シリアル入力がある場合は動作を停止
          return;
        }
        else
        { //
          for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
          {
            g_dxl_pos[dxl_i] = right_poster[pos_i][dxl_i];
            dxl.setGoalPosition(dxl_i, right_poster[pos_i][dxl_i]);
          }
          delay(right_poster_delay[pos_i]);
        }
      }
    }
  }
  else
  {
    for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
    {
      if (!g_dxl_is_connected[dxl_i])
      {
        continue;
      }
      g_dxl_pos[dxl_i] = constrain(uint16_t(dxl.getPresentPosition(dxl_i)), DXL_MIN_POSITION_VALUE, DXL_MAX_POSITION_VALUE);
    }
  }
  g_cmd_word = '\0';
  delay(5);
}

void leftturn()
{
  int leftturn_poster_cnt = 3;                         // 動作の数を入力
  int leftturn_poster[leftturn_poster_cnt][DXL_CNT] = {// モータ角度
                                                       {512, 512, 512, 512, 512},
                                                       {256, 256, 256, 256, 256},
                                                       {128, 128, 128, 128, 128}};
  int leftturn_poster_delay[leftturn_poster_cnt] = {200, 200, 200}; // 動作間の時間

  // トルクONの時の動作(速度・加速度の設定->コマンドの入力の有無の判断->動作)
  if (g_torque_is_on)
  {
    set_accel_velocity();
    while (0)
    {
      for (int pos_i = 1; pos_i <= leftturn_poster_cnt; pos_i++)
      {
        if (0 < USB_SERIAL.available() || 0 < BT_SERIAL.available())
        { // シリアル入力がある場合は動作を停止
          return;
        }
        else
        { //
          for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
          {
            g_dxl_pos[dxl_i] = leftturn_poster[pos_i][dxl_i];
            dxl.setGoalPosition(dxl_i, leftturn_poster[pos_i][dxl_i]);
          }
          delay(leftturn_poster_delay[pos_i]);
        }
      }
    }
  }
  else
  {
    for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
    {
      if (!g_dxl_is_connected[dxl_i])
      {
        continue;
      }
      g_dxl_pos[dxl_i] = constrain(uint16_t(dxl.getPresentPosition(dxl_i)), DXL_MIN_POSITION_VALUE, DXL_MAX_POSITION_VALUE);
    }
  }
  g_cmd_word = '\0';
  delay(5);
}

void rightturn()
{
  int rightturn_poster_cnt = 3;                          // 動作の数を入力
  int rightturn_poster[rightturn_poster_cnt][DXL_CNT] = {// モータ角度
                                                         {512, 512, 512, 512, 512},
                                                         {256, 256, 256, 256, 256},
                                                         {128, 128, 128, 128, 128}};
  int rightturn_poster_delay[rightturn_poster_cnt] = {200, 200, 200}; // 動作間の時間

  // トルクONの時の動作(速度・加速度の設定->コマンドの入力の有無の判断->動作)
  if (g_torque_is_on)
  {
    set_accel_velocity();
    while (0)
    {
      for (int pos_i = 1; pos_i <= rightturn_poster_cnt; pos_i++)
      {
        if (0 < USB_SERIAL.available() || 0 < BT_SERIAL.available())
        { // シリアル入力がある場合は動作を停止
          return;
        }
        else
        { //
          for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
          {
            g_dxl_pos[dxl_i] = rightturn_poster[pos_i][dxl_i];
            dxl.setGoalPosition(dxl_i, rightturn_poster[pos_i][dxl_i]);
          }
          delay(rightturn_poster_delay[pos_i]);
        }
      }
    }
  }
  else
  {
    for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
    {
      if (!g_dxl_is_connected[dxl_i])
      {
        continue;
      }
      g_dxl_pos[dxl_i] = constrain(uint16_t(dxl.getPresentPosition(dxl_i)), DXL_MIN_POSITION_VALUE, DXL_MAX_POSITION_VALUE);
    }
  }
  g_cmd_word = '\0';
  delay(5);
}

void setup()
{
for (uint16_t i = 0; i <= DXL_CNT; i++){  //初期位置・初期加速度・初期速度を各モータの要素に代入
  g_dxl_present_velocities[i] = DXL_INIT_VELOCITY;
  g_dxl_present_accelerations[i] = DXL_INIT_ACCELERATION;
  g_dxl_pos[i] = 512;
  g_dxl_is_connected[i] = false;
}
  #if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
    DEBUG_SERIAL.begin(SOFT_BUADRATE);
    BT_SERIAL.begin(BT_BUADRATE);
  #elif defined(ARDUINO_OpenCM904) || defined(ARDUINO_OpenCR)
    USB_SERIAL.begin(USB_BUADRATE);
    BT_SERIAL.begin(BT_BUADRATE);
  #endif

  // 誤作動防止のためDynamixelが一つ以上認識されるまで待機する
  uint8_t dxl_id = 1;
  while (!dxl.ping(dxl_id))
  {
    dxl_id++;
    dxl_id %= DXL_CNT + 1;
    delay(5);
  }
  delay(100);

  // 接続されているDynamixelを確認する
  for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
  {
    if (dxl.ping(dxl_i))
    {
      g_dxl_is_connected[dxl_i] = true;
    }
    else
    {
      g_dxl_is_connected[dxl_i] = false;
    }
  }
  // 接続されたDynamixelに対して初期設定を行い現在角度の表示する
  if (USB_SERIAL)
  {
    USB_SERIAL.println("----- DXL_DEFAULT_POSITION_VALUE ------");
  }
  for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
  {
    if (!g_dxl_is_connected[dxl_i])
    {
      continue;
    }
    dxl.torqueOff(dxl_i);
    dxl.setOperatingMode(dxl_i, OP_POSITION);
    dxl.torqueOn(dxl_i);
    if (1.5 <= DXL_PROTOCOL_VERSION)
    {
      dxl.writeControlTableItem(PROFILE_VELOCITY, dxl_i, DXL_INIT_VELOCITY);
      dxl.writeControlTableItem(PROFILE_ACCELERATION, dxl_i, DXL_INIT_ACCELERATION);
    }
    else
    {
      dxl.writeControlTableItem(MOVING_SPEED, dxl_i, DXL_INIT_VELOCITY);
    }

    g_dxl_pos[dxl_i] = constrain(uint16_t(dxl.getPresentPosition(dxl_i)), DXL_MIN_POSITION_VALUE, DXL_MAX_POSITION_VALUE);
    if (USB_SERIAL)
    {
      USB_SERIAL.print("[ID:");
      USB_SERIAL.print(dxl_i);
      USB_SERIAL.print(" Pos:");
      USB_SERIAL.print(g_dxl_pos[dxl_i]);
      USB_SERIAL.println("]");
    }
  }
  g_torque_is_on = true;
  if (USB_SERIAL)
  {
    USB_SERIAL.println("---------------------------------------");
  }
}

void loop(){
  // シリアル通信によりコマンドの取得を行う
  int8_t arg_max_index = -1;
  #if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
    if (0 < BT_SERIAL.available())
    {
      g_read_line = BT_SERIAL.readStringUntil('\n');
      arg_max_index = readCommand();
    }
    else if (0 < DEBUG_SERIAL.available())
    {
      g_read_line = DEBUG_SERIAL.readStringUntil('\n');
      arg_max_index = readCommand();
    }
  #elif defined(ARDUINO_OpenCM904) || defined(ARDUINO_OpenRB)
    if (0 < USB_SERIAL.available())
    {
      g_read_line = USB_SERIAL.readStringUntil('\n');
      arg_max_index = readCommand();
    }
    else if (0 < BT_SERIAL.available())
    {
      g_read_line = BT_SERIAL.readStringUntil('\n');
      arg_max_index = readCommand();
    }
  #endif
}