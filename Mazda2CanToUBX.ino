#define DEBUG_RXANY 0
#define BCAN 0
#include <SPI.h>
#include <mcp_can.h>
#include <SoftwareSerial.h>

// ソフトウェアシリアルの設定 (ピン5をRX、ピン7をTXとして使用)
SoftwareSerial receiver(5, 7); // RX, TX

// SPIのCSピン設定
// バージョン1.1以降はデフォルトでD9、v0.9bとv1.0はデフォルトでD10
const int SPI_CS_PIN = 9;

// 車速パルス用のピン
const int WT_PIN = 6;

// MCP_CANインスタンスの作成 (CSピンを指定)
MCP_CAN CAN0(SPI_CS_PIN);                                    // Set CS pin

// 変速モードフラグ (オートマかマニュアルか)
bool isAutomatic = true;

void setup()
{
    Serial.begin(57600);

    // マツダのB-CANとF-CANはどちらも500KBPS
    // 使用するCANボードは8MHzクロック
    int retryCount = 0;
    const int maxRetries = 10;
    while (CAN_OK != CAN0.begin(CAN_500KBPS, MCP_8MHz)) // CAN通信の初期化 (ボーレート500k)
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(1000); // 初期化失敗時は1秒待機して再試行
        retryCount++;
        if (retryCount >= maxRetries) {
            Serial.println("Maximum retry limit reached. Stopping initialization attempts.");
            break;
        }
    }
    if (retryCount < maxRetries) {
        Serial.println("CAN BUS Shield init ok!");
    }
    pinMode(WT_PIN, OUTPUT); // 車速パルス用ピンを出力に設定
    receiver.begin(115200);  // ソフトウェアシリアルの初期化 (115200ボーレート)
}

// UBX-ESF-MEAS 速度データのみ
unsigned char speedData[] = {
    0xB5, 0x62, 0x10, 0x02 // id
  , 0x0C, 0x00  // len
  , 0x00, 0x00, 0x00, 0x00 // time
  , 0x02, 0x08  // flag 1件 Ext1のTimemark
  , 0x00, 0x00  // id
  , 0x00, 0x00, 0x00, 0x0B // data speed
  , 0x04, 0x48}; // checksum

// UBX-ESF-MEAS リアTickのみ
unsigned char rearTickData[] = {
    0xB5, 0x62, 0x10, 0x02 // id
  , 0x18, 0x00  // len (4つの車輪データを含むためサイズを変更)
  , 0x00, 0x00, 0x00, 0x00 // time
  , 0x02, 0x10  // flag 4件 Ext1のTimemark
  , 0x00, 0x00  // id
  , 0x00, 0x00, 0x00, 0x08 // data FL (フロント左)
  , 0x00, 0x00, 0x00, 0x09 // data FR (フロント右)
  , 0x00, 0x00, 0x00, 0x0A // data RL (リア左)
  , 0x00, 0x00, 0x00, 0x0B // data RR (リア右)
  , 0x04, 0x48}; // checksum

// 変換係数 (マツダ用の変換値)
float unit = 637.0 * 4.0 / 60.0 / 60.0 * 10.0;
unsigned char fwd = 0x00; // シフト方向を示すフラグ
int cnt = 0;

void loop()
{
    unsigned char len = 0;
    unsigned char buf[64]; // Buffer size reduced to 64 based on expected data length

    // CANメッセージが利用可能か確認
    if (CAN_MSGAVAIL == CAN0.checkReceive())
    {
        // CANメッセージを読み込む (len: データ長, buf: データバッファ)
        CAN0.readMsgBuf(&len, buf);
        unsigned long canId = CAN0.getCanId();

        // 車速データの処理
        if (canId == 0x201) // Mazda2車のCAN IDに対応
        {
            unsigned long ms = millis(); // 現在のミリ秒を取得

            // 車速パルスの生成 (WT_PINをHIGHにして5ミリ秒後にLOWに戻す)
            digitalWrite(WT_PIN, HIGH);
            delay(5); // 5ミリ秒待機
            digitalWrite(WT_PIN, LOW);

            int ck_a = 0;
            int ck_b = 0;

            int len = 8 + (4 * 1);

            // 0.01km/hから1mm/sに変換してセット
            unsigned long kmph = buf[3] << 8; // 上位8ビットを取得
            kmph = kmph + buf[4]; // 下位8ビットを取得して合成

            float work = kmph * unit; // 変換係数を使用して速度を計算
            unsigned long sp = work;

            // 計算した速度をspeedDataに設定
            speedData[14] = byte(sp);
            speedData[15] = byte(sp >> 8);
            speedData[16] = byte(sp >> 16);

            // ローカル時間をspeedDataに設定
            speedData[6] = byte(ms);
            speedData[7] = byte(ms >> 8);
            speedData[8] = byte(ms >> 16);
            speedData[9] = byte(ms >> 24);

            // CLASS ~ ペイロードまでを対象にチェックサムを算出
            for (int i = 2; i < 6 + len; i++)
            {
                int hex2 = speedData[i];
                hex2 &= 0xFF;
                ck_a += hex2;
                ck_a &= 0xFF;
                ck_b += ck_a;
                ck_b &= 0xFF;
            }
            speedData[6 + len] = ck_a;
            speedData[6 + len + 1] = ck_b;

            // 速度データをレシーバに送信
            receiver.write(speedData, sizeof(speedData));
        }
        // シフト情報の取得（Mazda2用のID）
        else if (canId == 0x202)
        {
            // シフト情報を取得し、前進後退を示すフラグに設定
            fwd = ((buf[0] & 0x02) != 0) ? 0x40 : 0x00; // マスクを追加して安全にシフト方向を設定

            // オートマとマニュアルのモードを判断
            // 例えば、CAN ID 0x202のデータからオートマとマニュアルの判定を行う
            if ((buf[0] & 0x01) == 0x01) {
                isAutomatic = true; // オートマチックモード
            } else {
                isAutomatic = false; // マニュアルモード
            }

            Serial.print("Transmission Mode: ");
            if (isAutomatic) {
                Serial.println("Automatic");
            } else {
                Serial.println("Manual");
            }
        }
        // エンジン回転数の取得（Mazda2用のID）
        else if (canId == 0x210)
        {
            // エンジン回転数を取得
            unsigned long rpm = (buf[3] << 8) + buf[4];
            Serial.print("Engine RPM: ");
            Serial.println(rpm); // エンジン回転数をシリアルモニタに出力
        }
        // 4輪別パルスデータの取得（Mazda2用のID）
        else if (canId == 0x211)
        {
            unsigned long ms = millis(); // 現在のミリ秒を取得

            // 各車輪の速度データを取得して設定
            unsigned long fl = (buf[1] << 8) + buf[2]; // フロント左
            unsigned long fr = (buf[3] << 8) + buf[4]; // フロント右
            unsigned long rl = (buf[5] << 8) + buf[6]; // リア左
            unsigned long rr = (buf[7] << 8) + buf[8]; // リア右

            rearTickData[10] = byte(fl);
            rearTickData[11] = byte(fl >> 8);
            rearTickData[12] = byte(fl >> 16);

            rearTickData[14] = byte(fr);
            rearTickData[15] = byte(fr >> 8);
            rearTickData[16] = byte(fr >> 16);

            rearTickData[18] = byte(rl);
            rearTickData[19] = byte(rl >> 8);
            rearTickData[20] = byte(rl >> 16);

            rearTickData[22] = byte(rr);
            rearTickData[23] = byte(rr >> 8);
            rearTickData[24] = byte(rr >> 16);

            // ローカル時間をrearTickDataに設定
            rearTickData[6] = byte(ms);
            rearTickData[7] = byte(ms >> 8);
            rearTickData[8] = byte(ms >> 16);
            rearTickData[9] = byte(ms >> 24);

            // CLASS ~ ペイロードまでを対象にチェックサムを算出
            int ck_a = 0;
            int ck_b = 0;
            for (int i = 2; i < 26; i++)
            {
                int hex2 = rearTickData[i];
                hex2 &= 0xFF;
                ck_a += hex2;
                ck_a &= 0xFF;
                ck_b += ck_a;
                ck_b &= 0xFF;
            }
            rearTickData[26] = ck_a;
            rearTickData[27] = ck_b;

            // 車輪ごとの速度データをNEO-M8L向けにシリアル出力
            Serial.print("Front Left Wheel Speed: ");
            Serial.println(fl);
            Serial.print("Front Right Wheel Speed: ");
            Serial.println(fr);
            Serial.print("Rear Left Wheel Speed: ");
            Serial.println(rl);
            Serial.print("Rear Right Wheel Speed: ");
            Serial.println(rr);

            // リアの速度データをレシーバに送信
            receiver.write(rearTickData, sizeof(rearTickData));
        }
    }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
