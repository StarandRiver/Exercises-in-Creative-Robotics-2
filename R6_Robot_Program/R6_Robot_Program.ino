#include<Dynamixel2Arduino.h>

//マイコンごとの設定
//今回は手元にあるOpenCM・OpenRB・Dynamixel Shieldを使用できるように設定
#if defined(ARDUINO_OpenCM904)
  #define DXL_SERIAL Serial3       // OpenCM9.04の拡張ボードのDynamixelとのシリアル通信の設定
  #define USB_SERIAL Serial
  #define BT_SERIAL Serial2
    const int DXL_DIR_PIN = 22;
  const bool SENSOR_IS_AVAILABLE = false;
#elif defined(ARDUINO_OpenRB)
  #define DXL_SERIAL Serial1
  #define USB_SERIAL Serial
  #define BT_SERIAL Serial2
  const int DXL_DIR_PIN = -1;
  const bool SENSOR_IS_AVAILABLE = true;
#endif

/*======使用するモーターを選択======*/
//#define DXL_AX12A
#define DXL_XC430
/*=================================*/

  const uint8_t DXL_CNT = 14;                 // モータの数
  const uint16_t DXL_INIT_VELOCITY = 300;     // モータの初期速度
  const uint16_t DXL_INIT_ACCELERATION =300; // モータの初期加速度
  const unsigned long USB_BUADRATE = 57600;   // USBのボーレート
  const unsigned long BT_BUADRATE = 57600;    // Bluetoothデバイスのボーレート

#if defined(DXL_AX12A)
  const float DXL_PROTOCOL_VERSION = 1.0; // モータの通信プロトコル
  const uint16_t DXL_MAX_POSITION_VALUE = 1024; // モータの最大値
  const uint16_t DXL_MIN_POSITION_VALUE = 0; // モータの最小値
  const unsigned long DXL_BUADRATE = 57600;  // Dynamixelのボーレート
#elif defined(DXL_XC430)
  const float DXL_PROTOCOL_VERSION = 2.0; // モータの通信プロトコル
  const uint16_t DXL_MAX_POSITION_VALUE = 4096; // モータの最大値
  const uint16_t DXL_MIN_POSITION_VALUE = 0; // モータの最小値
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
                                                     {3153,844,3403,3144,3255,673,3056,2689,1001,1033,2639,907,1610,1587},
                                                     {3085,1599,3217,3076,2486,674,3094,2681,1219,1038,2664,1272,1617,1593},
                                                     {3084,1595,3403,3065,2421,673,3078,2624,687,1035,2668,693,2097,2068}};
  int forward_poster_delay[forward_poster_cnt] = {100, 100, 100}; // 動作間の時間

  // トルクONの時の動作(速度・加速度の設定->コマンドの入力の有無の判断->動作)
  if (g_torque_is_on)
  {
    set_accel_velocity();
    while (1)
    {
      for (int pos_i = 0; pos_i < forward_poster_cnt; pos_i++)
      {
        if (0 < USB_SERIAL.available() || 0 < BT_SERIAL.available())
        { // シリアル入力がある場合は動作を停止
          return;
        }
        else
        { 
          for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
          {
            g_dxl_pos[dxl_i] = forward_poster[pos_i][dxl_i-1];
            dxl.setGoalPosition(dxl_i, forward_poster[pos_i][dxl_i-1]);
          }
        }
        delay(forward_poster_delay[pos_i]);
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
      g_dxl_pos[dxl_i] = constrain(uint16_t(dxl.getPresentPosition(dxl_i-1)), DXL_MIN_POSITION_VALUE, DXL_MAX_POSITION_VALUE);
    }
  }
  g_cmd_word = '\0';
  delay(5);
}

void back()
{
  int back_poster_cnt = 3;                        // 動作の数を入力
  int back_poster[back_poster_cnt][DXL_CNT] = {// モータ角度
                                                     {1002,2037,1852,2955,1931,1935,3070,1366,3405,1004,1357,3412,1700,1674},
                                                     {1006,2053,1208,2982,2034,1235,3039,1954,2004,1005,1867,2189,1702,1677},
                                                     {1002,2164,1885,2983,2069,2123,3039,1973,2002,1006,1846,2221,2029,2002}};
  int back_poster_delay[back_poster_cnt] = {200, 200, 200}; // 動作間の時間

  // トルクONの時の動作(速度・加速度の設定->コマンドの入力の有無の判断->動作)
  if (g_torque_is_on)
  {
    set_accel_velocity();
    while (1)
    {
      for (int pos_i = 0; pos_i < back_poster_cnt; pos_i++)
      {
        if (0 < USB_SERIAL.available() || 0 < BT_SERIAL.available())
        { // シリアル入力がある場合は動作を停止
          return;
        }
        else
        { 
          for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
          {
            g_dxl_pos[dxl_i] = back_poster[pos_i][dxl_i-1];
            dxl.setGoalPosition(dxl_i, back_poster[pos_i][dxl_i-1]);
          }
        }
        delay(back_poster_delay[pos_i]);
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
      g_dxl_pos[dxl_i] = constrain(uint16_t(dxl.getPresentPosition(dxl_i-1)), DXL_MIN_POSITION_VALUE, DXL_MAX_POSITION_VALUE);
    }
  }
  g_cmd_word = '\0';
  delay(5);
}


void left()
{
  int left_poster_cnt = 4;                        // 動作の数を入力
  int left_poster[left_poster_cnt][DXL_CNT] = {// モータ角度
                                                     {511,519,187,200,311},
                                                     {511,861,318,430,311},
                                                     {489,864,331,361,699},
                                                     {489,655,169,155,699}};
  int left_poster_delay[left_poster_cnt] = {500, 500, 500, 500}; // 動作間の時間

  // トルクONの時の動作(速度・加速度の設定->コマンドの入力の有無の判断->動作)
  if (g_torque_is_on)
  {
    set_accel_velocity();
    while (1)
    {
      for (int pos_i = 0; pos_i < left_poster_cnt; pos_i++)
      {
        if (0 < USB_SERIAL.available() || 0 < BT_SERIAL.available())
        { // シリアル入力がある場合は動作を停止
          return;
        }
        else
        { 
          for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
          {
            g_dxl_pos[dxl_i] = left_poster[pos_i][dxl_i-1];
            dxl.setGoalPosition(dxl_i, left_poster[pos_i][dxl_i-1]);
          }
        }
        delay(left_poster_delay[pos_i]);
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
      g_dxl_pos[dxl_i] = constrain(uint16_t(dxl.getPresentPosition(dxl_i-1)), DXL_MIN_POSITION_VALUE, DXL_MAX_POSITION_VALUE);
    }
  }
  g_cmd_word = '\0';
  delay(5);
}


void right()
{
  int right_poster_cnt = 4;                        // 動作の数を入力
  int right_poster[right_poster_cnt][DXL_CNT] = {// モータ角度
                                                     {511,519,187,200,311},
                                                     {511,861,318,430,311},
                                                     {489,864,331,361,699},
                                                     {489,655,169,155,699}};
  int right_poster_delay[right_poster_cnt] = {500, 500, 500, 500}; // 動作間の時間

  // トルクONの時の動作(速度・加速度の設定->コマンドの入力の有無の判断->動作)
  if (g_torque_is_on)
  {
    set_accel_velocity();
    while (1)
    {
      for (int pos_i = 0; pos_i < right_poster_cnt; pos_i++)
      {
        if (0 < USB_SERIAL.available() || 0 < BT_SERIAL.available())
        { // シリアル入力がある場合は動作を停止
          return;
        }
        else
        { 
          for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
          {
            g_dxl_pos[dxl_i] = right_poster[pos_i][dxl_i-1];
            dxl.setGoalPosition(dxl_i, right_poster[pos_i][dxl_i-1]);
          }
        }
        delay(right_poster_delay[pos_i]);
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
      g_dxl_pos[dxl_i] = constrain(uint16_t(dxl.getPresentPosition(dxl_i-1)), DXL_MIN_POSITION_VALUE, DXL_MAX_POSITION_VALUE);
    }
  }
  g_cmd_word = '\0';
  delay(5);
}


void leftturn()
{
  int leftturn_poster_cnt = 4;                        // 動作の数を入力
  int leftturn_poster[leftturn_poster_cnt][DXL_CNT] = { // モータ角度
    {511, 519, 187, 200, 311},
    {511, 861, 318, 430, 311},
    {489, 864, 331, 361, 699},
#include <Dynamixel2Arduino.h>

// マイコンごとの設定
// 今回は手元にあるOpenCM・OpenRB・Dynamixel Shieldを使用できるように設定
#if defined(ARDUINO_OpenCM904)
#define DXL_SERIAL Serial3 // OpenCM9.04の拡張ボードのDynamixelとのシリアル通信の設定
#define USB_SERIAL Serial
#define BT_SERIAL Serial2
    const int DXL_DIR_PIN = 22;
  const bool SENSOR_IS_AVAILABLE = false;
#elif defined(ARDUINO_OpenRB)
#define DXL_SERIAL Serial1
#define USB_SERIAL Serial
#define BT_SERIAL Serial2
    const int DXL_DIR_PIN = -1;
// const bool SENSOR_IS_AVAILABLE = true;
#endif

/*======使用するモーターを選択======*/
// #define DXL_AX12A
#define DXL_XC430
  /*=================================*/

  const uint8_t DXL_CNT = 14;                 // モータの数
  const uint16_t DXL_INIT_VELOCITY = 400;     // モータの初期速度
  const uint16_t DXL_INIT_ACCELERATION = 400; // モータの初期加速度
  const unsigned long USB_BUADRATE = 57600;   // USBのボーレート
  const unsigned long BT_BUADRATE = 57600;    // Bluetoothデバイスのボーレート

#if defined(DXL_AX12A)
  const float DXL_PROTOCOL_VERSION = 1.0;       // モータの通信プロトコル
  const uint16_t DXL_MAX_POSITION_VALUE = 1024; // モータの最大値
  const uint16_t DXL_MIN_POSITION_VALUE = 0;    // モータの最小値
  const unsigned long DXL_BUADRATE = 57600;     // Dynamixelのボーレート
#elif defined(DXL_XC430)
  const float DXL_PROTOCOL_VERSION = 2.0;       // モータの通信プロトコル
  const uint16_t DXL_MAX_POSITION_VALUE = 4096; // モータの最大値
  const uint16_t DXL_MIN_POSITION_VALUE = 0;    // モータの最小値
  const unsigned long DXL_BUADRATE = 1000000;   // Dynamixelのボーレート
#endif

  uint16_t g_dxl_present_velocities[DXL_CNT + 1];    // Dynamixelの速度(モータの数+1)
  uint16_t g_dxl_present_accelerations[DXL_CNT + 1]; // Dynamixelの加速度(モータの数+1)
  uint16_t g_dxl_pos[DXL_CNT + 1];                   // Dynamixelの目標位置(モータの数+1)
  bool g_dxl_is_connected[DXL_CNT + 1];              // Dynamixelが接続されているかどうかの判別(モータの数+1)

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
  void set_accel_velocity()
  {
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
    return;
  }
  /*モーションの配置*/
  void forward()
  {
    int forward_poster_cnt = 3;                        // 動作の数を入力
    int forward_poster[forward_poster_cnt][DXL_CNT] = {// モータ角度
                                                       {1024, 2449, 835, 3072, 2449, 835, 1024, 1866, 1162, 3072, 1866, 1162, 2340, 2340},
                                                       {1024, 2196, 1096, 3072, 2196, 1096, 1024, 3159, 1227, 3072, 3159, 1227, 1623, 1623},
                                                       {1024, 2196, 1096, 3072, 2196, 1096, 1024, 3159, 1227, 3072, 3159, 1227, 2148, 2148}};
    int forward_poster_delay[forward_poster_cnt] = {200, 200, 200}; // 動作間の時間

    // トルクONの時の動作(速度・加速度の設定->コマンドの入力の有無の判断->動作)
    if (g_torque_is_on)
    {
      set_accel_velocity();
      while (1)
      {
        for (int pos_i = 0; pos_i < forward_poster_cnt; pos_i++)
        {
          if (0 < USB_SERIAL.available() || 0 < BT_SERIAL.available())
          { // シリアル入力がある場合は動作を停止
            return;
          }
          else
          {
            for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
            {
              g_dxl_pos[dxl_i] = forward_poster[pos_i][dxl_i - 1];
              dxl.setGoalPosition(dxl_i, forward_poster[pos_i][dxl_i - 1]);
            }
          }
          delay(forward_poster_delay[pos_i]);
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
        g_dxl_pos[dxl_i] = constrain(uint16_t(dxl.getPresentPosition(dxl_i - 1)), DXL_MIN_POSITION_VALUE, DXL_MAX_POSITION_VALUE);
      }
    }
    g_cmd_word = '\0';
    delay(5);
  }

  void back()
  {
    int back_poster_cnt = 3;                     // 動作の数を入力
    int back_poster[back_poster_cnt][DXL_CNT] = {// モータ角度
                                                 {1024, 2230, 2934, 3072, 2230, 2934, 1024, 1647, 3261, 3072, 1647, 3261, 2340, 2340},
                                                 {1024, 937, 2869, 3072, 937, 2869, 1024, 1900, 3001, 3072, 1900, 3001, 1623, 1623},
                                                 {1024, 937, 2869, 3072, 937, 2869, 1024, 1900, 3001, 3072, 1900, 3001, 2048, 2048}};
    int back_poster_delay[back_poster_cnt] = {200, 200, 200}; // 動作間の時間

    // トルクONの時の動作(速度・加速度の設定->コマンドの入力の有無の判断->動作)
    if (g_torque_is_on)
    {
      set_accel_velocity();
      while (1)
      {
        for (int pos_i = 0; pos_i < back_poster_cnt; pos_i++)
        {
          if (0 < USB_SERIAL.available() || 0 < BT_SERIAL.available())
          { // シリアル入力がある場合は動作を停止
            return;
          }
          else
          {
            for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
            {
              g_dxl_pos[dxl_i] = back_poster[pos_i][dxl_i - 1];
              dxl.setGoalPosition(dxl_i, back_poster[pos_i][dxl_i - 1]);
            }
          }
          delay(back_poster_delay[pos_i]);
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
        g_dxl_pos[dxl_i] = constrain(uint16_t(dxl.getPresentPosition(dxl_i - 1)), DXL_MIN_POSITION_VALUE, DXL_MAX_POSITION_VALUE);
      }
    }
    g_cmd_word = '\0';
    delay(5);
  }

  void left()
  {
    int left_poster_cnt = 3;                     // 動作の数を入力
    int left_poster[left_poster_cnt][DXL_CNT] = {// モータ角度
                                                 {600, 2449, 835, 2500, 2449, 835, 750, 1866, 1162, 2852, 1866, 1162, 2340, 2340},
                                                 {600, 2196, 1096, 2500, 2196, 1096, 750, 3159, 1227, 2852, 3159, 1227, 1623, 1623},
                                                 {600, 2196, 1096, 2500, 2196, 1096, 750, 3159, 1227, 2852, 3159, 1227, 2048, 2048}};
    int left_poster_delay[left_poster_cnt] = {200, 200, 200}; // 動作間の時間

    // トルクONの時の動作(速度・加速度の設定->コマンドの入力の有無の判断->動作)
    if (g_torque_is_on)
    {
      set_accel_velocity();
      while (1)
      {
        for (int pos_i = 0; pos_i < left_poster_cnt; pos_i++)
        {
          if (0 < USB_SERIAL.available() || 0 < BT_SERIAL.available())
          { // シリアル入力がある場合は動作を停止
            return;
          }
          else
          {
            for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
            {
              g_dxl_pos[dxl_i] = left_poster[pos_i][dxl_i - 1];
              dxl.setGoalPosition(dxl_i, left_poster[pos_i][dxl_i - 1]);
            }
          }
          delay(left_poster_delay[pos_i]);
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
        g_dxl_pos[dxl_i] = constrain(uint16_t(dxl.getPresentPosition(dxl_i - 1)), DXL_MIN_POSITION_VALUE, DXL_MAX_POSITION_VALUE);
      }
    }
    g_cmd_word = '\0';
    delay(5);
  }

  void right()
  {
    int right_poster_cnt = 3;                      // 動作の数を入力
    int right_poster[right_poster_cnt][DXL_CNT] = {// モータ角度

                                                   {3072, 1866, 1162, 1024, 1866, 1162, 1024, 1647, 3261, 3072, 1647, 3261, 2340, 2340},
                                                   {3072, 3159, 1227, 1024, 3159, 1227, 1024, 1900, 3001, 3072, 1900, 3001, 1623, 1623},
                                                   {3072, 3159, 1227, 1024, 3159, 1227, 1024, 1900, 3001, 3072, 1900, 3001, 2048, 2048}};
    int right_poster_delay[right_poster_cnt] = {200, 200, 200}; // 動作間の時間

    // トルクONの時の動作(速度・加速度の設定->コマンドの入力の有無の判断->動作)
    if (g_torque_is_on)
    {
      set_accel_velocity();
      while (1)
      {
        for (int pos_i = 0; pos_i < right_poster_cnt; pos_i++)
        {
          if (0 < USB_SERIAL.available() || 0 < BT_SERIAL.available())
          { // シリアル入力がある場合は動作を停止
            return;
          }
          else
          {
            for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
            {
              g_dxl_pos[dxl_i] = right_poster[pos_i][dxl_i - 1];
              dxl.setGoalPosition(dxl_i, right_poster[pos_i][dxl_i - 1]);
            }
          }
          delay(right_poster_delay[pos_i]);
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
        g_dxl_pos[dxl_i] = constrain(uint16_t(dxl.getPresentPosition(dxl_i - 1)), DXL_MIN_POSITION_VALUE, DXL_MAX_POSITION_VALUE);
      }
    }
    g_cmd_word = '\0';
    delay(5);
  }

  void leftturn()
  {
    int leftturn_poster_cnt = 6;                         // 動作の数を入力
    int leftturn_poster[leftturn_poster_cnt][DXL_CNT] = {// モータ角度
                                                         {1024, 3159, 1227, 3072, 3159, 1227, 1024, 1345, 2577, 3072, 1345, 2577, 1623, 1623},
                                                         {1024, 3159, 1227, 3072, 3330, 1227, 1024, 766, 2577, 3072, 1345, 2577, 1623, 1623},
                                                         {1024, 3159, 1227, 2672, 3330, 1227, 624, 766, 2577, 3072, 1345, 2577, 1623, 1623},
                                                         {1024, 3159, 1227, 2672, 3159, 1227, 624, 1345, 2577, 3072, 1345, 2577, 1623, 1623},
                                                         {1024, 3330, 1227, 2672, 3159, 1227, 624, 1345, 2577, 3072, 766, 2577, 1623, 1623},
                                                         {1024, 3330, 1227, 3072, 3159, 1227, 1024, 1345, 2577, 3072, 766, 2577, 1623, 1623}};
    int leftturn_poster_delay[leftturn_poster_cnt] = {100, 100, 100, 100, 100, 100}; // 動作間の時間

    // トルクONの時の動作(速度・加速度の設定->コマンドの入力の有無の判断->動作)
    if (g_torque_is_on)
    {
      set_accel_velocity();
      while (1)
      {
        for (int pos_i = 0; pos_i < leftturn_poster_cnt; pos_i++)
        {
          if (0 < USB_SERIAL.available() || 0 < BT_SERIAL.available())
          { // シリアル入力がある場合は動作を停止
            return;
          }
          else
          {
            for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
            {
              g_dxl_pos[dxl_i] = leftturn_poster[pos_i][dxl_i - 1];
              dxl.setGoalPosition(dxl_i, leftturn_poster[pos_i][dxl_i - 1]);
            }
          }
          delay(leftturn_poster_delay[pos_i]);
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
        g_dxl_pos[dxl_i] = constrain(uint16_t(dxl.getPresentPosition(dxl_i - 1)), DXL_MIN_POSITION_VALUE, DXL_MAX_POSITION_VALUE);
      }
    }
    g_cmd_word = '\0';
    delay(5);
  }

  void rightturn()
  {
    int rightturn_poster_cnt = 6;                          // 動作の数を入力
    int rightturn_poster[rightturn_poster_cnt][DXL_CNT] = {// モータ角度
                                                           {1024, 3159, 1227, 3072, 3159, 1227, 1024, 1345, 2577, 3072, 1345, 2577, 1623, 1623},
                                                           {1024, 3330, 1227, 3072, 3159, 1227, 1024, 1345, 2577, 3072, 766, 2577, 1623, 1623},
                                                           {1424, 3330, 1227, 3072, 3159, 1227, 1024, 1345, 2577, 3472, 766, 2577, 1623, 1623},
                                                           {1424, 3159, 1227, 3072, 3159, 1227, 1024, 1345, 2577, 3472, 1345, 2577, 1623, 1623},
                                                           {1424, 3159, 1227, 3072, 3330, 1227, 1024, 766, 2577, 3472, 1345, 2577, 1623, 1623},
                                                           {1024, 3159, 1227, 3072, 3330, 1227, 1024, 766, 2577, 3072, 1345, 2577, 1623, 1623}};
    int rightturn_poster_delay[rightturn_poster_cnt] = {100, 100, 100, 100, 100, 100}; // 動作間の時間

    // トルクONの時の動作(速度・加速度の設定->コマンドの入力の有無の判断->動作)
    if (g_torque_is_on)
    {
      set_accel_velocity();
      while (1)
      {
        for (int pos_i = 0; pos_i < rightturn_poster_cnt; pos_i++)
        {
          if (0 < USB_SERIAL.available() || 0 < BT_SERIAL.available())
          { // シリアル入力がある場合は動作を停止
            return;
          }
          else
          {
            for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
            {
              g_dxl_pos[dxl_i] = rightturn_poster[pos_i][dxl_i - 1];
              dxl.setGoalPosition(dxl_i, rightturn_poster[pos_i][dxl_i - 1]);
            }
          }
          delay(rightturn_poster_delay[pos_i]);
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

  void othermotion()
  {
    if (g_torque_is_on)
    {
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
      for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
      {
        if (!g_dxl_is_connected[dxl_i])
        {
          continue;
        }
        dxl.setGoalPosition(dxl_i, g_dxl_pos[dxl_i]);
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
    return;
  }

  void setup()
  {
    for (uint16_t i = 0; i <= DXL_CNT; i++)
    { // 初期位置・初期加速度・初期速度を各モータの要素に代入
      g_dxl_present_velocities[i] = DXL_INIT_VELOCITY;
      g_dxl_present_accelerations[i] = DXL_INIT_ACCELERATION;
#if defined(DXL_AX12A)
      g_dxl_pos[i] = 512;
#elif defined(DXL_XC430)
      g_dxl_pos[i] = 2048;
#endif
      g_dxl_is_connected[i] = false;
    }
    USB_SERIAL.begin(USB_BUADRATE);
    BT_SERIAL.begin(BT_BUADRATE);
    while (!USB_SERIAL && !BT_SERIAL)
    {
    }
    delay(2000);

    dxl.begin(DXL_BUADRATE);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

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

  void loop()
  {
    // シリアル通信によりコマンドの取得を行う
    int8_t arg_max_index = -1;
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
    /**
       取得したコマンドによって以下の操作を行う
       [i] -> 初期姿勢にする
       [f] -> トルクオフ
       [n] -> トルクオン
       [c] -> モータの速度の設定値を30にする
       [v] -> モータの速度の設定値を60にする
       [b] -> モータの速度の設定値を100にする
       以下は追加コマンド
       [a,POS1,POS2,..., POS_DXL_CNT] -> 全てのモータの位置制御（初期型のmコマンド）
       [v,VEL] -> 全てのモータの速度の設定値をVELにする
       [v,ID1,VEL1,ID2,VEL2,...] -> 指定したモータ複数の速度設定
       [k,ACC] -> 全てのモータの加速度の設定値をACCにする
       [k,ID1,ACC1,ID2,ACC2,...] -> 指定したモータ複数の加速度設定
       [p] -> モータの現在位置を取得する
    */
    switch (g_cmd_word)
    {
    case 'i':
      for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
      {
#if defined(DXL_AX12A)
        g_dxl_pos[dxl_i] = 512;
#elif defined(DXL_XC430)
        int normal_position[14] = {1024, 3159, 1227, 3072, 3159, 1227, 1024, 1345, 2577, 3072, 1345, 2577, 1623, 1623};
        g_dxl_pos[dxl_i] = normal_position[dxl_i - 1];
#endif
      }
      othermotion();
      break;
    case 'f': // g_cmd_args -> { }
      for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
      {
        if (!g_dxl_is_connected[dxl_i])
        {
          continue;
        }
        dxl.torqueOff(dxl_i);
        g_torque_is_on = false;
      }
      othermotion();
      break;
    case 'n': // g_cmd_args -> { }
      for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
      {
        if (!g_dxl_is_connected[dxl_i])
        {
          continue;
        }
        dxl.torqueOn(dxl_i);
        g_torque_is_on = true;
      }
      othermotion();
      break;
    case 'c': // g_cmd_args -> { }
      for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
      {
        g_dxl_present_velocities[dxl_i] = 30;
      }
      othermotion();
      break;
    case 'v': // g_cmd_args -> { ID1, VEL1, ID2, VEL2, ... }, { VEL } or { }
      if (1 <= arg_max_index)
      {
        for (int arg_i = 0; arg_i < arg_max_index; arg_i += 2)
        {
          if (0 < g_cmd_args[arg_i] && g_cmd_args[arg_i] <= DXL_CNT)
          {
            g_dxl_present_velocities[g_cmd_args[arg_i]] = g_cmd_args[arg_i + 1];
          }
        }
      }
      else if (0 == arg_max_index)
      {
        for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
        {
          g_dxl_present_velocities[dxl_i] = g_cmd_args[0];
        }
      }
      else
      {
        for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
        {
          g_dxl_present_velocities[dxl_i] = 60;
        }
      }
      othermotion();
      break;
    case 'b': // g_cmd_args -> { }
      for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
      {
        g_dxl_present_velocities[dxl_i] = 100;
      }
      othermotion();
      break;
    case 'w':
      forward();
      break;
    case 'a':
      left();
      break;
    case 's':
      back();
      break;
    case 'd':
      right();
      break;
    case 'q':
      leftturn();
      break;
    case 'e':
      rightturn();
      break;
    case 'p': // g_cmd_args -> { }
    {
      String dxl_pos_msg = "[";
      for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
      {
        if (g_dxl_is_connected[dxl_i])
        {
          dxl_pos_msg += String(uint16_t(dxl.getPresentPosition(dxl_i)));
        }
        else
        {
          dxl_pos_msg += "-1";
        }
        if (dxl_i < DXL_CNT)
        {
          dxl_pos_msg += ",";
        }
      }
      dxl_pos_msg += "]";
      USB_SERIAL.println(dxl_pos_msg);
      BT_SERIAL.println(dxl_pos_msg);
      USB_SERIAL.flush();
      BT_SERIAL.flush();
      break;
    }
    default:
      break;
    }
    g_cmd_word = '\0';
    delay(5);
  }
  {
    489, 655, 169, 155, 699
  }
};
  int leftturn_poster_delay[leftturn_poster_cnt] = {500, 500, 500, 500}; // 動作間の時間

  // トルクONの時の動作(速度・加速度の設定->コマンドの入力の有無の判断->動作)
  if (g_torque_is_on)
  {
    set_accel_velocity();
    while (1)
    {
      for (int pos_i = 0; pos_i < leftturn_poster_cnt; pos_i++)
      {
        if (0 < USB_SERIAL.available() || 0 < BT_SERIAL.available())
        { // シリアル入力がある場合は動作を停止
          return;
        }
        else
        { 
          for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
          {
            g_dxl_pos[dxl_i] = leftturn_poster[pos_i][dxl_i-1];
            dxl.setGoalPosition(dxl_i, leftturn_poster[pos_i][dxl_i-1]);
          }
        }
        delay(leftturn_poster_delay[pos_i]);
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
      g_dxl_pos[dxl_i] = constrain(uint16_t(dxl.getPresentPosition(dxl_i-1)), DXL_MIN_POSITION_VALUE, DXL_MAX_POSITION_VALUE);
    }
  }
  g_cmd_word = '\0';
  delay(5);
}


void rightturn()
{
  int rightturn_poster_cnt = 4;                        // 動作の数を入力
  int rightturn_poster[rightturn_poster_cnt][DXL_CNT] = {// モータ角度
                                                     {511,519,187,200,311},
                                                     {511,861,318,430,311},
                                                     {489,864,331,361,699},
                                                     {489,655,169,155,699}};
  int rightturn_poster_delay[rightturn_poster_cnt] = {500, 500, 500, 500}; // 動作間の時間

  // トルクONの時の動作(速度・加速度の設定->コマンドの入力の有無の判断->動作)
  if (g_torque_is_on)
  {
    set_accel_velocity();
    while (1)
    {
      for (int pos_i = 0; pos_i < rightturn_poster_cnt; pos_i++)
      {
        if (0 < USB_SERIAL.available() || 0 < BT_SERIAL.available())
        { // シリアル入力がある場合は動作を停止
          return;
        }
        else
        { 
          for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
          {
            g_dxl_pos[dxl_i] = rightturn_poster[pos_i][dxl_i-1];
            dxl.setGoalPosition(dxl_i, rightturn_poster[pos_i][dxl_i-1]);
          }
        }
        delay(rightturn_poster_delay[pos_i]);
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


void othermotion(){
  if (g_torque_is_on)
  {
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
    for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
    {
      if (!g_dxl_is_connected[dxl_i])
      {
        continue;
      }
      dxl.setGoalPosition(dxl_i, g_dxl_pos[dxl_i]);
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
  return;
}

void setup()
{
for (uint16_t i = 0; i <= DXL_CNT; i++){  //初期位置・初期加速度・初期速度を各モータの要素に代入
  g_dxl_present_velocities[i] = DXL_INIT_VELOCITY;
  g_dxl_present_accelerations[i] = DXL_INIT_ACCELERATION;
  #if defined(DXL_AX12A)
    g_dxl_pos[i] = 512;
  #elif defined(DXL_XC430)
    g_dxl_pos[i] = 2048;
  #endif
  g_dxl_is_connected[i] = false;
}
    USB_SERIAL.begin(USB_BUADRATE);
    BT_SERIAL.begin(BT_BUADRATE);
    while (!USB_SERIAL && !BT_SERIAL) {}
    delay(2000);

    dxl.begin(DXL_BUADRATE);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

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
    /**
     * 取得したコマンドによって以下の操作を行う
     * [i] -> 初期姿勢にする
     * [f] -> トルクオフ
     * [n] -> トルクオン
     * [c] -> モータの速度の設定値を30にする
     * [v] -> モータの速度の設定値を60にする
     * [b] -> モータの速度の設定値を100にする
     * 以下は追加コマンド
     * [a,POS1,POS2,..., POS_DXL_CNT] -> 全てのモータの位置制御（初期型のmコマンド）
     * [v,VEL] -> 全てのモータの速度の設定値をVELにする
     * [v,ID1,VEL1,ID2,VEL2,...] -> 指定したモータ複数の速度設定
     * [k,ACC] -> 全てのモータの加速度の設定値をACCにする
     * [k,ID1,ACC1,ID2,ACC2,...] -> 指定したモータ複数の加速度設定
     * [p] -> モータの現在位置を取得する
     */
    switch (g_cmd_word){
      case 'i':
        for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
        {
          #if defined(DXL_AX12A)
            g_dxl_pos[dxl_i] = 512;
            #elif defined(DXL_XC430)
            g_dxl_pos[dxl_i] = 2048;
            #endif
        }
        othermotion();
        break;
      case 'f': // g_cmd_args -> { }
        for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
        {
          if (!g_dxl_is_connected[dxl_i])
          {
            continue;
          }
          dxl.torqueOff(dxl_i);
          g_torque_is_on = false;
        }
        othermotion();
        break;
      case 'n': // g_cmd_args -> { }
        for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
        {
          if (!g_dxl_is_connected[dxl_i])
          {
            continue;
          }
          dxl.torqueOn(dxl_i);
          g_torque_is_on = true;
        }
        othermotion();
        break;
      case 'c': // g_cmd_args -> { }
        for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
        {
          g_dxl_present_velocities[dxl_i] = 30;
        }
        othermotion();
        break;
      case 'v': // g_cmd_args -> { ID1, VEL1, ID2, VEL2, ... }, { VEL } or { }
        if (1 <= arg_max_index)
        {
          for (int arg_i = 0; arg_i < arg_max_index; arg_i += 2)
          {
            if (0 < g_cmd_args[arg_i] && g_cmd_args[arg_i] <= DXL_CNT)
            {
              g_dxl_present_velocities[g_cmd_args[arg_i]] = g_cmd_args[arg_i + 1];
            }
          }
        }
        else if (0 == arg_max_index)
        {
          for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
          {
            g_dxl_present_velocities[dxl_i] = g_cmd_args[0];
          }
        }
        else
        {
          for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
          {
            g_dxl_present_velocities[dxl_i] = 60;
          }
        }
        othermotion();
        break;
      case 'b': // g_cmd_args -> { }
        for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
        {
          g_dxl_present_velocities[dxl_i] = 100;
        }
        othermotion();
        break;
      case 'w':
        forward();
        break;
      case 'a':
        left();
        break;
      case 's':
        back();
        break;
      case 'd':
        right();
        break;
      case 'q':
        leftturn();
        break;
      case 'e':
        rightturn();
        break;
      case 'p': // g_cmd_args -> { }
            {
                String dxl_pos_msg = "[";
                for (int dxl_i = 1; dxl_i <= DXL_CNT; dxl_i++)
                {
                    if (g_dxl_is_connected[dxl_i])
                    {
                        dxl_pos_msg += String(uint16_t(dxl.getPresentPosition(dxl_i)));
                    }
                    else
                    {
                        dxl_pos_msg += "-1";
                    }
                    if (dxl_i < DXL_CNT)
                    {
                        dxl_pos_msg += ",";
                    }
                }
                dxl_pos_msg += "]";
                USB_SERIAL.println(dxl_pos_msg);
                BT_SERIAL.println(dxl_pos_msg);
                USB_SERIAL.flush();
                BT_SERIAL.flush();
                break;
            }
        default:
            break;
    }
    g_cmd_word = '\0';
    delay(5);
}
