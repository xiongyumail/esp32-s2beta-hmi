#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"
#include "esp_spiffs.h"
#include "driver/i2s.h"
#include "audio.h"
#include "esp_log.h"
#include "es8311.h"
#include "touch.h"
#include "mp3dec.h"

static const char *TAG = "AUDIO";

#define SAMPLE_RATE     (32000)
#define I2S_NUM         (0)
#define WAVE_FREQ_HZ    (100)
#define PI              (3.14159265)
#define I2S_BCK_IO      (GPIO_NUM_15)
#define I2S_WS_IO       (GPIO_NUM_16)
#define I2S_DO_IO       (GPIO_NUM_17)
#define I2S_DI_IO       (GPIO_NUM_21)

#define SAMPLE_PER_CYCLE (SAMPLE_RATE/WAVE_FREQ_HZ)

bool play_flag = 0;

void aplay_mp3(char *path)
{
    ESP_LOGI(TAG,"start to decode ...");
    HMP3Decoder hMP3Decoder;
    MP3FrameInfo mp3FrameInfo;
    unsigned char *readBuf=malloc(MAINBUF_SIZE);
    if(readBuf==NULL) {
        ESP_LOGE(TAG,"readBuf malloc failed");
        return;
    }
    short *output=malloc(1153*4);
    if(output==NULL) {
        free(readBuf);
        ESP_LOGE(TAG,"outBuf malloc failed");
    }
    hMP3Decoder = MP3InitDecoder();
    if (hMP3Decoder == 0) {
        free(readBuf);
        free(output);
        ESP_LOGE(TAG,"memory is not enough..");
    }

    int samplerate=0;
    i2s_zero_dma_buffer(0);
    FILE *mp3File=fopen( path,"rb");
    if(mp3File==NULL) {
        MP3FreeDecoder(hMP3Decoder);
        free(readBuf);
        free(output);
        ESP_LOGE(TAG,"open file failed");
    }
    char tag[10];
    int tag_len = 0;
    int read_bytes = fread(tag, 1, 10, mp3File);
    if(read_bytes == 10) {
        if (memcmp(tag,"ID3",3) == 0) {
            tag_len = ((tag[6] & 0x7F)<< 21)|((tag[7] & 0x7F) << 14) | ((tag[8] & 0x7F) << 7) | (tag[9] & 0x7F);
            // ESP_LOGI(TAG,"tag_len: %d %x %x %x %x", tag_len,tag[6],tag[7],tag[8],tag[9]);
            fseek(mp3File, tag_len - 10, SEEK_SET);
        } else {
            fseek(mp3File, 0, SEEK_SET);
        }
    }
    unsigned char* input = &readBuf[0];
    int bytesLeft = 0;
    int outOfData = 0;
    unsigned char* readPtr = readBuf;
    while (1) {    
        while (!play_flag) {
            i2s_zero_dma_buffer(0);
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        if (bytesLeft < MAINBUF_SIZE) {
            memmove(readBuf, readPtr, bytesLeft);
            int br = fread(readBuf + bytesLeft, 1, MAINBUF_SIZE - bytesLeft, mp3File);
            if ((br == 0)&&(bytesLeft==0)) break;

            bytesLeft = bytesLeft + br;
            readPtr = readBuf;
        }
        int offset = MP3FindSyncWord(readPtr, bytesLeft);
        if (offset < 0) {  
            ESP_LOGE(TAG,"MP3FindSyncWord not find");
            bytesLeft=0;
            continue;
        } else {
            readPtr += offset;                    //data start point
            bytesLeft -= offset;                 //in buffer
            int errs = MP3Decode(hMP3Decoder, &readPtr, &bytesLeft, output, 0);
            if (errs != 0)
            {
                ESP_LOGE(TAG,"MP3Decode failed ,code is %d ",errs);
                break;
            }
            MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);   
            if(samplerate!=mp3FrameInfo.samprate)
            {
                samplerate=mp3FrameInfo.samprate;
                i2s_set_clk(0,samplerate,16,mp3FrameInfo.nChans);
                ESP_LOGI(TAG,"mp3file info---bitrate=%d,layer=%d,nChans=%d,samprate=%d,outputSamps=%d",mp3FrameInfo.bitrate,mp3FrameInfo.layer,mp3FrameInfo.nChans,mp3FrameInfo.samprate,mp3FrameInfo.outputSamps);
            }   
            i2s_write_bytes(0,(const char*)output,mp3FrameInfo.outputSamps*2, 1000 / portTICK_RATE_MS);
        }
    }
    i2s_zero_dma_buffer(0);
    MP3FreeDecoder(hMP3Decoder);
    free(readBuf);
    free(output);  
    fclose(mp3File);

    ESP_LOGI(TAG,"end mp3 decode ..");
}

static void audio_task(void *arg)
{
    //for 36Khz sample rates, we create 100Hz sine wave, every cycle need 36000/100 = 360 samples (4-bytes or 8-bytes each sample)
    //depend on bits_per_sample
    //using 6 buffers, we need 60-samples per buffer
    //if 2-channels, 16-bit each channel, total buffer is 360*4 = 1440 bytes
    //if 2-channels, 24/32-bit each channel, total buffer is 360*8 = 2880 bytes
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX,                                  // Only TX
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = 16,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,                           //2-channels
        .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
        .dma_buf_count = 6,
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = true,     /*!< I2S auto clear tx descriptor if there is underflow condition (helps in avoiding noise in case of data unavailability) */
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1                                //Interrupt level 1
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_DO_IO,
        .data_in_num = I2S_DI_IO                                               //Not used
    };
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);

    while (1) {
        aplay_mp3("/spiffs/lemon_tree.mp3");
        vTaskDelay(1000 / portTICK_RATE_MS);

    }
}

static void audio_control_task(void *arg)
{
    uint32_t touch_status = 0, last_touch_status = 0;
    uint32_t volume = 50;
    es8311_set_voice_volume(volume);
    while (1) {
        touch_get_status(&touch_status);
        if (touch_status != last_touch_status) {
            switch (touch_status) {
                case 0x4: {
                    volume+=5;
                    es8311_set_voice_volume(volume);
                }
                break;
                case 0x200: {
                    volume-=5;
                    es8311_set_voice_volume(volume);
                }
                break;

                case 0x100: {
                    play_flag = play_flag ? 0 : 1;
                }
                break;
            }
            last_touch_status = touch_status;
        }
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

int audio_init()
{
    es8311_init(SAMPLE_RATE);
    es8311_set_voice_volume(50);
    es8311_read_all();

    xTaskCreate(audio_task, "audio_task", 4096, NULL, 5, NULL);
    xTaskCreate(audio_control_task, "audio_control_task", 2048, NULL, 5, NULL);

    return 0;
}
