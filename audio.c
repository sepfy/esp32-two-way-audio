#include "driver/i2s_std.h"
#include "driver/i2s_pdm.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include "esp_audio_enc.h"
#include "esp_audio_enc_default.h"
#include "esp_audio_enc_reg.h"

#include "esp_audio_dec_default.h"
#include "esp_audio_dec.h"

#include "esp_g711_dec.h"
#include "esp_g711_enc.h"

#include "peer_connection.h"

#define I2S_CLK_GPIO 42
#define I2S_DIN_GPIO 41

#define I2S_STD_BCLK_GPIO 2
#define I2S_STD_WS_GPIO 1
#define I2S_STD_DOUT_GPIO 3

static const char* TAG = "AUDIO";

extern PeerConnection* g_pc;
extern PeerConnectionState eState;
extern int get_timestamp();

#define AUDIO_SAMPLE_SIZE 8000 / 1000 * 20 * 2 // 8000HZ duration 20ms

typedef struct {
    uint8_t data[AUDIO_SAMPLE_SIZE];
    size_t size;
} audio_data_t;

static QueueHandle_t audio_queue;

i2s_chan_handle_t rx_handle = NULL;
i2s_chan_handle_t tx_handle = NULL;

esp_audio_enc_handle_t enc_handle = NULL;
esp_audio_dec_handle_t dec_handle = NULL;

esp_audio_enc_in_frame_t aenc_in_frame = {0};
esp_audio_enc_out_frame_t aenc_out_frame = {0};

esp_audio_dec_in_raw_t adec_in_frame = {0};
esp_audio_dec_out_frame_t adec_out_frame = {0};

esp_g711_enc_config_t g711_enc_cfg;
esp_g711_dec_cfg_t g711_dec_cfg;

esp_audio_enc_config_t enc_cfg;
esp_audio_dec_cfg_t dec_cfg;

esp_err_t audio_enc_init() {

  esp_audio_enc_register_default();
  g711_enc_cfg.sample_rate = ESP_AUDIO_SAMPLE_RATE_8K;
  g711_enc_cfg.channel = ESP_AUDIO_MONO;
  g711_enc_cfg.bits_per_sample = ESP_AUDIO_BIT16;

  enc_cfg.type = ESP_AUDIO_TYPE_G711A;
  enc_cfg.cfg = &g711_enc_cfg;
  enc_cfg.cfg_sz = sizeof(g711_enc_cfg);

  int ret = esp_audio_enc_open(&enc_cfg, &enc_handle);
  if (ret != ESP_AUDIO_ERR_OK) {
    ESP_LOGE(TAG, "audio encoder open failed");
    return ESP_FAIL;
  }

  aenc_in_frame.len = AUDIO_SAMPLE_SIZE;
  aenc_in_frame.buffer = malloc(aenc_in_frame.len);
  aenc_out_frame.len = AUDIO_SAMPLE_SIZE / 2;
  aenc_out_frame.buffer = malloc(aenc_out_frame.len);
  return ESP_OK;
}

esp_err_t audio_dec_init() {

  esp_audio_dec_register_default();
  g711_dec_cfg.channel = ESP_AUDIO_MONO;

  dec_cfg.type = ESP_AUDIO_TYPE_G711A;
  dec_cfg.cfg = &g711_dec_cfg;
  dec_cfg.cfg_sz = sizeof(g711_dec_cfg);

  int ret = esp_audio_dec_open(&dec_cfg, &dec_handle);
  if (ret != ESP_AUDIO_ERR_OK) {
    ESP_LOGE(TAG, "audio encoder open failed");
    return ESP_FAIL;
  }

  adec_in_frame.len = AUDIO_SAMPLE_SIZE / 2;
  adec_in_frame.buffer = malloc(adec_in_frame.len);
  adec_out_frame.len = AUDIO_SAMPLE_SIZE;
  adec_out_frame.buffer = malloc(adec_out_frame.len);

  return ESP_OK;
}

esp_err_t audio_i2s_rx_init(void) {
  // Microphone
  i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_handle));

  i2s_pdm_rx_config_t pdm_rx_cfg = {
      .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(8000),
      .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
          .clk = I2S_CLK_GPIO,
          .din = I2S_DIN_GPIO,
          .invert_flags = {
              .clk_inv = false,
          },
      },
  };

  ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_rx_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
  return ESP_OK;
}

esp_err_t audio_i2s_tx_init(void) {
  // Speaker
  i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
  ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_handle, NULL));

  i2s_std_config_t tx_std_cfg = {
      .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(8000),
      .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
          .mclk = I2S_GPIO_UNUSED,
          .bclk = I2S_STD_BCLK_GPIO,
          .ws   = I2S_STD_WS_GPIO,
          .dout = I2S_STD_DOUT_GPIO,
          .invert_flags = {
              .mclk_inv = false,
              .bclk_inv = false,
              .ws_inv   = false,
          },
      },
  };
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &tx_std_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
  return ESP_OK;
}

int32_t audio_put_samples(uint8_t* buf, size_t size) {
  size_t bytes_written;

  if (i2s_channel_write(tx_handle, (char*)buf, size, &bytes_written, 1000) != ESP_OK) {
    ESP_LOGE(TAG, "i2s write error");
  }
  ESP_LOGI(TAG, "i2s tx: %d", bytes_written);
  return bytes_written;
}

int32_t audio_get_samples(uint8_t* buf, size_t size) {
  size_t bytes_read;

  if (i2s_channel_read(rx_handle, (char*)buf, size, &bytes_read, 1000) != ESP_OK) {
    ESP_LOGE(TAG, "i2s read error");
  }

  return bytes_read;
}

void audio_ontrack(uint8_t* data, size_t size, void* userdata) {
  audio_data_t audio_buffer;
  audio_buffer.size = size;
  memcpy(audio_buffer.data, data, size);
  xQueueSend(audio_queue, &audio_buffer, 0);
}

void audio_tx_task(void* arg) {
  int ret;
  static int64_t last_time;
  int64_t curr_time;
  float bytes = 0;
  audio_data_t audio_buffer;

  ESP_LOGI(TAG, "audio tx task started");

  audio_dec_init();
  audio_i2s_tx_init();
  audio_queue = xQueueCreate(10, sizeof(audio_data_t));

  last_time = get_timestamp();
  for (;;) {
    if (xQueueReceive(audio_queue, &audio_buffer, portMAX_DELAY) == pdPASS) {
      memcpy(adec_in_frame.buffer, audio_buffer.data, audio_buffer.size);
      if (esp_audio_dec_process(dec_handle, &adec_in_frame, &adec_out_frame) == ESP_AUDIO_ERR_OK) {
	ESP_LOGI(TAG, "audio tx %d", audio_buffer.size);
	audio_put_samples(adec_out_frame.buffer, adec_out_frame.decoded_size);
	bytes += audio_buffer.size;
	if (bytes > 10000) {
	  curr_time = get_timestamp();
	  ESP_LOGI(TAG, "audio tx bitrate: %.1f bps", 1000.0 * (bytes * 8.0 / (float)(curr_time - last_time)));
	  last_time = curr_time;
	  bytes = 0;
	}
      }
    }
  }
}

void audio_rx_task(void* arg) {
  int ret;
  static int64_t last_time;
  int64_t curr_time;
  float bytes = 0;

  ESP_LOGI(TAG, "audio rx task started");
  audio_enc_init();
  audio_i2s_rx_init();

  last_time = get_timestamp();
  for (;;) {
    if (eState == PEER_CONNECTION_COMPLETED) {
      ret = audio_get_samples(aenc_in_frame.buffer, aenc_in_frame.len);
      if (ret == aenc_in_frame.len) {
        if (esp_audio_enc_process(enc_handle, &aenc_in_frame, &aenc_out_frame) == ESP_AUDIO_ERR_OK) {
          peer_connection_send_audio(g_pc, aenc_out_frame.buffer, aenc_out_frame.encoded_bytes);

          bytes += aenc_out_frame.encoded_bytes;
          if (bytes > 10000) {
            curr_time = get_timestamp();
            ESP_LOGI(TAG, "audio rx bitrate: %.1f bps", 1000.0 * (bytes * 8.0 / (float)(curr_time - last_time)));
            last_time = curr_time;
            bytes = 0;
          }
        }
      }
      vTaskDelay(pdMS_TO_TICKS(5));

    } else {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}
