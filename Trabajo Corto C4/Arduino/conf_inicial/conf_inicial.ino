#include "BluetoothA2DPSink.h"

BluetoothA2DPSink a2dp_sink;
bool is_active = true;
String titulo;
String estado;
String t_play;
void avrc_metadata_callback(uint8_t data1, const uint8_t *data2) {
  
  Serial.printf("AVRC metadata rsp: attribute id 0x%x, %s\n", data1, data2);
    //convertir a string *data2
  String atributo = String((char*)data2);
  //Serial.println(s);
  //Serial.println(s.length());
  //Serial.println(s.substring(0, s.length()-1));

  //condicionales para identificar el atributo
    if (data1 == 0x01){
      Serial.println("Titulo: ");
      titulo = atributo;
      Serial.println(titulo);
    } else if (data1 == 0x02){
      Serial.println("Artista");
    } else if (data1 == 0x03){
      Serial.println("Album");
    } else if (data1 == 0x04){
      Serial.println("Género");
    } else if (data1 == 0x05){
      Serial.println("Tiempo de reproducción: ");
      t_play = atributo;
      Serial.println(t_play);
    }
//  Serial.printf("Reproduciendo: ");
//  Serial.println(data1);
    // condicionales para identificar el estado
    if (a2dp_sink.get_audio_state()==ESP_A2D_AUDIO_STATE_STARTED){
      //Serial.println("play");
      estado = "Play";
      //a2dp_sink.play();
    } else {
      //Serial.println("pause");
      estado = "Pause";
    }
}

void setup() {
  Serial.begin(115200);
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
    //set_avrc_metadata_attribute_mask(ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_PLAYING_TIME);
    a2dp_sink.set_i2s_config(i2s_config);
    a2dp_sink.start("MyMusic");
    
    a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
    //a2dp_sink.set_avrc_metadata_attribute_mask(ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_PLAYING_TIME);

}


void read_data_stream(const uint8_t *data, uint32_t length)
{
  int16_t *samples = (int16_t*) data;
  uint32_t sample_count = length/2;
  Serial.println(*samples);
  // Do something with the data packet
}



void loop() {

 

}
