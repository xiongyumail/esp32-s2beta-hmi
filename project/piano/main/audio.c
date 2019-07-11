#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
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

static QueueHandle_t audio_queue = NULL;

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

typedef struct 
{
    char rld[4];    //riff 标志符号
    int  rLen;      //
    char wld[4];    //格式类型（wave）
    char fld[4];    //"fmt"
 
    int fLen;   //sizeof(wave format matex)
 
    short wFormatTag;   //编码格式
    short wChannels;    //声道数
    int   nSamplesPersec;  //采样频率
    int   nAvgBitsPerSample;//WAVE文件采样大小
    short wBlockAlign; //块对齐
    short wBitsPerSample;   //WAVE文件采样大小

    char dld[4];        //”data“
    int  wSampleLength; //音频数据的大小
 }WAV_HEADER;

void aplay_wav(char* filename, uint8_t state)
{
    WAV_HEADER wav_head;
    FILE *f= fopen(filename, "r");
    if (f == NULL) {
            ESP_LOGE(TAG,"Failed to open file:%s",filename);
            return;
    }
    //fprintf(f, "Hello %s!\n", card->cid.name);
    int rlen=fread(&wav_head,1,sizeof(wav_head),f);
    if(rlen!=sizeof(wav_head)){
            ESP_LOGE(TAG,"read faliled");
            return;
    }
    int channels = wav_head.wChannels;
    int frequency = wav_head.nSamplesPersec;
    int bit = wav_head.wBitsPerSample;
    int datalen= wav_head.wSampleLength;
    // i2s_set_clk(I2S_NUM, frequency, 16, channels);
    (void)datalen;
    ESP_LOGI(TAG,"channels:%d,frequency:%d,bit:%d\n",channels,frequency,bit);
    char* samples_data = malloc(1024);
    do{
        rlen=fread(samples_data,1,1024,f);
        //datalen-=rlen;
        i2s_write_bytes(I2S_NUM, samples_data, rlen, 1000 / portTICK_RATE_MS);
    }while(rlen>0);
    fclose(f);
    free(samples_data);
    f=NULL;
}

static void audio_task(void *arg)
{
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
    i2s_set_clk(I2S_NUM, 8000, 16, 2);
    int ret;
    char *file = NULL;
    WAV_HEADER wav_head;
    FILE *f = NULL;
    char samples_data[1024];
    while (1) {
        ret = xQueueReceive(audio_queue, &file, 0);
        if (ret == pdTRUE) {
            i2s_zero_dma_buffer(0);
            if (f) {
                fclose(f);
                f = NULL;
            }

            f = fopen(file, "r");
            if (f != NULL)  {
                fread(&wav_head, 1, sizeof(wav_head), f);
                ESP_LOGI(TAG, "channels:%d,frequency:%d,bit:%d\n", wav_head.wChannels, wav_head.nSamplesPersec, wav_head.wBitsPerSample);
            }
        }

        if (f) {
            ret = fread(samples_data, 1, 1024, f);
            if (ret > 0) {
                i2s_write_bytes(I2S_NUM, samples_data, ret, 1000 / portTICK_RATE_MS);
            } else {
                if (f) {
                    fclose(f);
                    f = NULL;
                }
            }
        } else {
            vTaskDelay(10 / portTICK_RATE_MS);
        }
    }
}

static void audio_control_task(void *arg)
{
    uint32_t touch_status = 0, last_touch_status = 0;
    uint32_t volume = 70;
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

static void udp_server_task(void *pvParameters)
{
    int sockfd;
    struct sockaddr_in saddr;
    int r;
    char recvline[128];
    struct sockaddr_in presaddr;
    socklen_t len;
    
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    bzero(&saddr, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    saddr.sin_port = htons(9999);
    bind(sockfd, (struct sockaddr*)&saddr, sizeof(saddr));

    char *file = NULL;
    char piano_ctr[2];
    while (1)
    {
        r = recvfrom(sockfd, recvline, sizeof(recvline), 0 , (struct sockaddr*)&presaddr, &len);
        if (r > 0) {
            printf("audio: %c\n", recvline[11]);
            piano_ctr[0] = recvline[11];
            piano_ctr[1] = recvline[12];
            if (piano_ctr[0] == '!') {
                file = "";
            } else if (piano_ctr[0] == '#') {
                switch (piano_ctr[1]) {
                    case 'C': {
                        file = "/spiffs/074-16.wav";
                    }
                    break;

                    case 'D': {
                        file = "/spiffs/076-16.wav";
                    }
                    break;

                    case 'F': {
                        file = "/spiffs/079-16.wav";
                    }
                    break;

                    case 'G': {
                        file = "/spiffs/081-16.wav";
                    }
                    break;

                    case 'A': {
                        file = "/spiffs/083-16.wav";
                    }
                    break;
                }
            } else {
                switch (piano_ctr[0]) {
                    case 'C': {
                        file = "/spiffs/073-16.wav";
                    }
                    break;

                    case 'D': {
                        file = "/spiffs/075-16.wav";
                    }
                    break;

                    case 'E': {
                        file = "/spiffs/077-16.wav";
                    }
                    break;

                    case 'F': {
                        file = "/spiffs/078-16.wav";
                    }
                    break;

                    case 'G': {
                        file = "/spiffs/080-16.wav";
                    }
                    break;

                    case 'A': {
                        file = "/spiffs/082-16.wav";
                    }
                    break;

                    case 'B': {
                        file = "/spiffs/084-16.wav";
                    }
                    break;
                }
            }
            if (strlen(file) > 0) {
                xQueueOverwrite(audio_queue, &file);
            }
        }

    }
    vTaskDelete(NULL);
}

int audio_init()
{
    audio_queue = xQueueCreate(1, sizeof(char *));

    es8311_init(SAMPLE_RATE);
    // es8311_read_all();

    xTaskCreate(audio_task, "audio_task", 4096, NULL, 5, NULL);
    xTaskCreate(audio_control_task, "audio_control_task", 2048, NULL, 5, NULL);
    xTaskCreate(udp_server_task, "udp_server", 2048, NULL, 5, NULL);

    return 0;
}
