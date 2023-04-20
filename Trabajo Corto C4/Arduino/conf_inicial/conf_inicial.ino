#include "BluetoothA2DPSink.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

// PANTALLA OLED 
#define ANCHO 128
#define ALTO 64
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03
#define OLED_RESET -1 
#define I2C_SDA 22
#define I2C_SCL 21
Adafruit_SSD1306 oled(ANCHO,ALTO, &Wire, OLED_RESET);
BluetoothA2DPSink a2dp_sink;
String titulo;
String estado;
String t_play;


// funcion para lectura de metadatos
void avrc_metadata_callback(uint8_t data1, const uint8_t *data2) { 
  Serial.printf("AVRC metadata rsp: attribute id 0x%x, %s\n", data1, data2);
    //convertir a string *data2
  String atributo = String((char*)data2);

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

    // condicionales para identificar el estado
    if (a2dp_sink.get_audio_state()==ESP_A2D_AUDIO_STATE_STARTED){
      //Serial.println("play");
      estado = "Encendido";
      //a2dp_sink.play();
    } else {
      //Serial.println("pause");
      estado = "Pause";
    }

    //IMPRIMIR VALORES EN OLED//
      oled.clearDisplay();
      oled.setTextColor(WHITE); //NO MULTIPLES COLORES
      oled.setCursor(0,0); //ELIGE COORDENADAS PARA COMENEZAR A ESCRIBIR
      oled.setTextSize(1.5); //TAMAÑO DEL TEXTO
      oled.print("Reproduciendo: ");
      oled.setCursor(8,20); // NUEVAS COORDENADAS
      oled.setTextSize(1.95);   //TAMAÑO DE TEXTO
      oled.print(titulo);
      oled.setCursor(10,55); // NUEVAS COORDENADAS
      oled.setTextSize(1.7);   //TAMAÑO DE 
      oled.print("Estado: ");
      oled.print(estado);
      oled.display(); 
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
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
    a2dp_sink.start("Grupo4");
    
    a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
    //a2dp_sink.set_stream_reader(read_data_stream);
    /* INICIAR OLED */
    oled.begin(SSD1306_SWITCHCAPVCC,0x3C);
    oled.clearDisplay();
}


void read_data_stream(const uint8_t *data, uint32_t length)
{
  int16_t *samples = (int16_t*) data;
  uint32_t sample_count = length/2;
  //Serial.println(*samples);
  if (sample_count > 0){
      //Serial.println("Titulo: ");
      estado = "play";
    }else{
      estado = "pause";
    }
}


void loop() {


 

}
