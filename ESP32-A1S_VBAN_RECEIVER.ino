/*  ESP32-A1S VBAN RECEIVER
 *
 *  Inspired by:
 *    https://github.com/flyingKenny/VBAN-Receptor-ESP8266-I2S
 *    https://github.com/rkinnett/ESP32-VBAN-Network-Audio-Player
 *  
 *  Used librarys:
 *    https://github.com/evert-arias/EasyButton
 *    https://github.com/schreibfaul1/AC101
 *  
 *  Supports:
 *    SampleRates 11025<=96000Hz
 *    1 or 2 Channel
 *    16bit PCM format
 * 
 */

#include <WiFi.h>
#include <WiFiUDP.h>
#include "driver/i2s.h"
#include "src/AC101/src/AC101.h"
#include "src/EasyButton/src/EasyButton.h"
#include "VBAN.h"
#include "packet.h"

#define VBAN_BIT_RESOLUTION_MAX 2
#define EINVAL 22

#define UseWifiHeader false          // if you use a header file "myWiFi.h" for your WiFi credentials in the Arduino libraries folder, then set this to true
#if UseWifiHeader
#include "myWiFi.h"
#else
const char *ssid = "ssid";          // Your WiFi SSID
const char *password = "password";  // Your WiFi Key
#endif

char StreamName[16] = "Stream1";      // incoming VBAN Stream Name
IPAddress fromIp (192, 168, 0, 1);  // only receive VBAN Packets from this IP
uint16_t vbanPort = 6980;             // local port to listen on

#define DEBUG             (bool)false   // prints out debug information periodically
#define DEBUGTIME         2500          // time in ms

uint8_t networkQuality = 4; //increase the number to increase the buffer size. So use 1-2 for a fast network and 3-4 for medium and 5-6 for slow networks. In my network 4 works very well

const uint16_t initialSampleRate  = 44100;
const uint16_t initialSamplesPerPacket  = 235;   // 235 fits for 44100, 256 for everything over 44100

//-nothing-to-change-from-here-on-------------------------------------

#define IIS_SCLK          27
#define IIS_LCLK          26
#define IIS_DSIN          25
#define IIS_DSOUT         35

#define IIC_CLK           32
#define IIC_DATA          33

#define GPIO_PA_EN        GPIO_NUM_21

#define HP_DETECT         39      // if Headphones are plugged in the pin reads 0

#define KEY_5             18      // KEY5 Volume Down
#define KEY_6             5       // KEY6 Volume Up

#define LED1              22      // LED4 (but not used in this sketch)

static AC101 ac;

EasyButton volUpBtn(KEY_6), volDoBtn(KEY_5);
static uint8_t volume = 33;
static uint8_t volume_step = 1;

uint32_t last_millis = 0;
uint32_t cycle_time = 0;

uint32_t overflow_counter = 0;

// buffer for receiving data
uint8_t VBANBuffer[VBAN_PROTOCOL_MAX_SIZE];

WiFiUDP udpIn;

i2s_config_t i2s_config;
i2s_pin_config_t i2s_pin_config;

//-some-functions--------------------------

void configure_i2s(int newSampleRate, uint16_t newDmaBufLen, uint8_t newDmaBufCount);
void update_i2s(int newSampleRate, uint16_t newDmaBufLen, uint8_t newDmaBufCount);
void VolUpPressed();
void VolDoPressed();
static int packet_pcm_check(char const* buffer, size_t size);
int vban_packet_check(char const* streamname, char const* buffer, size_t size);

//-----------------------------------------

void setup() {
  Serial.begin(500000);

  Serial.println("Initialize buttons");
  volUpBtn.begin();
  volDoBtn.begin();
  volUpBtn.onPressed(VolUpPressed);
  volDoBtn.onPressed(VolDoPressed);

  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH);   //turn LED off

  Serial.println("HP Detection Circuit");
  pinMode(HP_DETECT, INPUT);

  Serial.printf("Connect to AC101 codec... ");
  while (not ac.begin(IIC_DATA, IIC_CLK))
  {
    Serial.printf("Failed!\n");
    delay(1000);
  }
  Serial.printf("OK\n");

  ac.SetVolumeSpeaker(volume);
  ac.SetVolumeHeadphone(volume);

  // Enable amplifier
  Serial.println("Enable amplifier");
  pinMode(GPIO_PA_EN, OUTPUT);
  digitalWrite(GPIO_PA_EN, HIGH);

  Serial.println("Start I2S driver");
  configure_i2s(initialSampleRate, initialSamplesPerPacket, networkQuality);

  // Enable WiFi
  Serial.println("WiFi begin");
  //WiFi.config(local_ip, gateway, subnet); // manual ip config
  WiFi.mode(WIFI_STA);
  Serial.print("WiFi Connecting");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("WiFi IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println();


  Serial.println("start VBAN WiFi receptor");
  udpIn.begin(vbanPort);
  //udpIn.beginMulticast(IPAddress(239, 1, 1, 1), vbanPort);
  /*
    Experimental! VBAN does not support Multicast offical, but you can enter a Multicast Address in Voicemeeter VBAN Outgoing Streams without an error message.
    I think it depends on how good your network can handle Multicast packets. So beware of a lower link quality.
  */

  delay(250);
  Serial.println("start main loop");
  Serial.println();
  Serial.println("KEY5 = Volume Down    KEY6 = Volume Up");
  Serial.println();
  delay(250);
}


uint8_t VBANSample;
uint32_t* VBANData;
uint16_t* VBANData1Ch;
uint32_t VBANValue;

void loop() {
  cycle_time = micros();

  // if there's data available, read a packet
  int packetSize = udpIn.parsePacket();
  if (packetSize) {
    if (udpIn.remoteIP() == fromIp)
    {
      // read the packet into VBANBuffer
      udpIn.read((char*)VBANBuffer, VBAN_PROTOCOL_MAX_SIZE);

      if (vban_packet_check(StreamName, (char*)VBANBuffer, packetSize) == 0)
      {
        struct VBanHeader const* const hdr = PACKET_HEADER_PTR((char*)VBANBuffer);

        /*
              Serial.print("PacketSize: ");
              Serial.print(packetSize);
              Serial.print(", ");
              Serial.print("SampleRate: ");
              Serial.print((int)VBanSRList[hdr->format_SR & VBAN_SR_MASK]);
              Serial.print(", ");
              Serial.print("Number of samples: ");
              Serial.print(hdr->format_nbs + 1);
              Serial.print(", ");
              Serial.print("Number of channels: ");
              Serial.print(hdr->format_nbc + 1);
              Serial.print(", ");
              Serial.print("Bit format: ");
              Serial.print(hdr->format_bit);
              Serial.print(", ");
              Serial.print("Stream name: ");
              Serial.println(hdr->streamname);
        */

        //do not call set_sample_rates on every received packet, on the esp32 it causes audio glitches
        //i2s_set_sample_rates(I2S_NUM_0, (int)VBanSRList[hdr->format_SR & VBAN_SR_MASK]);
        int rate_temp = (int)VBanSRList[hdr->format_SR & VBAN_SR_MASK];
        if (i2s_config.sample_rate != rate_temp)
        {
          update_i2s(rate_temp, (uint8_t) hdr->format_nbs + 1, networkQuality);
        }

        //copy the received data to i2s buffer
        VBANData = (uint32_t*)(((char*)VBANBuffer) + VBAN_HEADER_SIZE);
        VBANData1Ch = (uint16_t*)VBANData;

        size_t bytes_written = 0;

        switch (hdr->format_nbc)
        {
          case 0:    //one channel
            for (VBANSample = 0; VBANSample < hdr->format_nbs; VBANSample++)
            {
              VBANValue = 0;
              VBANValue = (uint32_t)VBANData1Ch[VBANSample] << 16 | VBANData1Ch[VBANSample];
              i2s_write(I2S_NUM_0, &VBANValue, sizeof(uint32_t), &bytes_written, 0);
              if (bytes_written != 4)
              {
                overflow_counter++;
              }
            }
            break;

          case 1:   //two channels
            i2s_write(I2S_NUM_0, VBANData, (hdr->format_nbs + 1) * sizeof(uint32_t), &bytes_written, 0);
            overflow_counter = overflow_counter + ((hdr->format_nbs + 1) - (bytes_written / 4));
            break;

          default:  //channel # not supported
            Serial.println("wrong number of channels");
            break;
        }

      }
      else
      {
        Serial.println("VBAN Error");
      }
    }
    else
    {
      Serial.println("received packet from wrong IP address");
    }
  }

  //Buttons--------------------------

  volUpBtn.read();
  volDoBtn.read();

  //---------------------------------

  if (DEBUG)
  {

    cycle_time = micros() - cycle_time;

    if ((millis() - last_millis) > DEBUGTIME)
    {
      Serial.println("--------------------------------");
      Serial.println("Debug: ");
      Serial.print("millis: ");
      Serial.print(millis());
      Serial.println("ms");
      Serial.print("Volume: ");
      Serial.println(volume);
      Serial.print("Headphones: ");
      if (digitalRead(HP_DETECT))
        Serial.println("disconnected");
      else
      {
        Serial.println("connected");
      }
      Serial.print("Cycle-Time: ");
      Serial.print(cycle_time);
      Serial.println("us");
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());
      Serial.print("Overflow counter: ");
      Serial.println(overflow_counter);
      Serial.print("KEY 5: ");
      Serial.println(digitalRead(KEY_5));
      Serial.print("KEY 6: ");
      Serial.println(digitalRead(KEY_6));
      Serial.println();

      last_millis = millis();
    }
  }

  yield();

}


void configure_i2s(int newSampleRate, uint16_t newDmaBufLen, uint8_t newDmaBufCount)
{
  //Serial.printf("Configuring i2s driver with new sample rate %u and buffer length %u\n", newSampleRate, newDmaBufLen);

  i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate =  newSampleRate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = newDmaBufCount * 8,     // buffer size is multiplicated by 8
    .dma_buf_len = newDmaBufLen,
    .use_apll = true,
    .tx_desc_auto_clear = true
  };

  i2s_pin_config_t i2s_pin_config = {
    .bck_io_num = IIS_SCLK,
    .ws_io_num = IIS_LCLK,
    .data_out_num = IIS_DSIN,
    .data_in_num = IIS_DSOUT
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &i2s_pin_config);
  i2s_zero_dma_buffer(I2S_NUM_0);

  //enable MCLK on GPIO0
  REG_WRITE(PIN_CTRL, 0xFF0);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);

}

void update_i2s(int newSampleRate, uint16_t newDmaBufLen, uint8_t newDmaBufCount)
{
  //Serial.printf("update i2s driver with new sample rate %u and buffer length %u\n", newSampleRate, newDmaBufLen);

  i2s_stop(I2S_NUM_0);
  i2s_driver_uninstall(I2S_NUM_0);

  i2s_config.sample_rate = newSampleRate;
  i2s_config.dma_buf_len = newDmaBufLen;
  i2s_config.dma_buf_count = newDmaBufCount * 8;  // buffer size is multiplicated by 8

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &i2s_pin_config);
  i2s_zero_dma_buffer(I2S_NUM_0);

  //enable MCLK on GPIO0
  REG_WRITE(PIN_CTRL, 0xFF0);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);

}

void VolUpPressed()
{
  if (volume + volume_step <= 63)
  {
    volume = volume + volume_step;
    ac.SetVolumeHeadphone(volume);
    ac.SetVolumeSpeaker(volume);
  }
}

void VolDoPressed()
{
  if (volume - volume_step >= 0)
  {
    volume = volume - volume_step;
    ac.SetVolumeHeadphone(volume);
    ac.SetVolumeSpeaker(volume);
  }
}

static int packet_pcm_check(char const* buffer, size_t size)
{
  /** the packet is already a valid vban packet and buffer already checked before */

  struct VBanHeader const* const hdr = PACKET_HEADER_PTR(buffer);
  enum VBanBitResolution const bit_resolution = (VBanBitResolution)(hdr->format_bit & VBAN_BIT_RESOLUTION_MASK);
  int const sample_rate   = hdr->format_SR & VBAN_SR_MASK;
  int const nb_samples    = hdr->format_nbs + 1;
  int const nb_channels   = hdr->format_nbc + 1;
  size_t sample_size      = 0;
  size_t payload_size     = 0;

  //logger_log(LOG_DEBUG, "%s: packet is vban: %u, sr: %d, nbs: %d, nbc: %d, bit: %d, name: %s, nu: %u",
  //    __func__, hdr->vban, hdr->format_SR, hdr->format_nbs, hdr->format_nbc, hdr->format_bit, hdr->streamname, hdr->nuFrame);

  if (bit_resolution >= VBAN_BIT_RESOLUTION_MAX)
  {
    Serial.println("invalid bit resolution");
    return -EINVAL;
  }

  if (sample_rate >= VBAN_SR_MAXNUMBER)
  {
    Serial.println("invalid sample rate");
    return -EINVAL;
  }

  sample_size = VBanBitResolutionSize[bit_resolution];
  payload_size = nb_samples * sample_size * nb_channels;

  if (payload_size != (size - VBAN_HEADER_SIZE))
  {
    //    logger_log(LOG_WARNING, "%s: invalid payload size, expected %d, got %d", __func__, payload_size, (size - VBAN_HEADER_SIZE));
    Serial.println("invalid payload size");
    return -EINVAL;
  }

  return 0;
}

int vban_packet_check(char const* streamname, char const* buffer, size_t size)
{
  struct VBanHeader const* const hdr = PACKET_HEADER_PTR(buffer);
  enum VBanProtocol protocol = VBAN_PROTOCOL_UNDEFINED_4;
  enum VBanCodec codec = (VBanCodec)VBAN_BIT_RESOLUTION_MAX;

  if ((streamname == 0) || (buffer == 0))
  {
    Serial.println("null pointer argument");
    return -EINVAL;
  }

  if (size <= VBAN_HEADER_SIZE)
  {
    Serial.println("packet too small");
    return -EINVAL;
  }

  if (hdr->vban != VBAN_HEADER_FOURC)
  {
    Serial.println("invalid vban magic fourc");
    return -EINVAL;
  }

  if (strncmp(streamname, hdr->streamname, VBAN_STREAM_NAME_SIZE))
  {
    Serial.println("different streamname");
    return -EINVAL;
  }

  /** check the reserved bit : it must be 0 */
  if (hdr->format_bit & VBAN_RESERVED_MASK)
  {
    Serial.println("reserved format bit invalid value");
    return -EINVAL;
  }

  /** check protocol and codec */
  protocol        = (VBanProtocol)(hdr->format_SR & VBAN_PROTOCOL_MASK);
  codec           = (VBanCodec)(hdr->format_bit & VBAN_CODEC_MASK);

  switch (protocol)
  {
    case VBAN_PROTOCOL_AUDIO:
      return (codec == VBAN_CODEC_PCM) ? packet_pcm_check(buffer, size) : -EINVAL;
      /*Serial.print("OK: VBAN_PROTOCOL_AUDIO");
        Serial.print(": nuFrame ");
        Serial.print(hdr->nuFrame);
        Serial.print(": format_SR ");
        Serial.print(hdr->format_SR);
        Serial.print(": format_nbs ");
        Serial.print(hdr->format_nbs);
        Serial.print(": format_nbc ");
        Serial.println(hdr->format_nbc);
      */
      return 0;
    case VBAN_PROTOCOL_SERIAL:
    case VBAN_PROTOCOL_TXT:
    case VBAN_PROTOCOL_UNDEFINED_1:
    case VBAN_PROTOCOL_UNDEFINED_2:
    case VBAN_PROTOCOL_UNDEFINED_3:
    case VBAN_PROTOCOL_UNDEFINED_4:
      /** not supported yet */
      return -EINVAL;

    default:
      Serial.println("packet with unknown protocol");
      return -EINVAL;
  }
}
