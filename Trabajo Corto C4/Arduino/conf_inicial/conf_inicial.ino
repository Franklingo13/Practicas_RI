#include "BluetoothA2DPSink.h"

BluetoothA2DPSink a2dp_sink;

void setup() {
  Serial.begin(115200);
  // intento de poder leer el nombre de la canción
//  a2dp_sink.set_on_data_received(data_received_callback);
  a2dp_sink.set_stream_reader(read_data_stream);
  //A2DS Sink empleando el DAC interno
  static const i2s_config_t i2s_config = {
        .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
        .sample_rate = 44100, // corrected by info from bluetooth
        .bits_per_sample = (i2s_bits_per_sample_t) 16, /* the DAC module will only take the 8bits from MSB */
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_MSB,
        .intr_alloc_flags = 0, // default interrupt priority
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false
    };

    a2dp_sink.set_i2s_config(i2s_config);
    a2dp_sink.start("MyMusic");

}

//void data_received_callback() {
//  Serial.println("Data packet received");
//}

void read_data_stream(const uint8_t *data, uint32_t length)
{
  int16_t *samples = (int16_t*) data;
  uint32_t sample_count = length/2;
  Serial.println(*samples);
  // Do something with the data packet
}
void loop() {
  // put your main code here, to run repeatedly:

}
