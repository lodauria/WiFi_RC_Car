#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/api.h"
#include "errno.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "iot_servo.h"

#define MOT_PIN 32
#define SER_PIN 33

#define MOTOR_PWM_CHANNEL_MOT LEDC_CHANNEL_0
#define MOTOR_PWM_CHANNEL_SER LEDC_CHANNEL_1
#define MOTOR_PWM_TIMER LEDC_TIMER_3
#define MOTOR_SPEED_MODE LEDC_HIGH_SPEED_MODE

// Calibrazione ruote sterzanti
#define MAX_L 60  // Ruote tutte a sinistra
#define MAX_R 120 // Ruote tutte a destra

// Calibrazione motore (ESC)
#define MAX_FOR 115 // Massima potenza avanti
#define MAX_REV 65  // Massima potenza retro

// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#include "esp_camera.h"

// EPS32 WROVER Camera
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 21
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 19
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 5
#define Y2_GPIO_NUM 4
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

static const char *TAG = "camera";

float map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

esp_err_t init_camera(void)
{
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 24000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fragment_mode = true;
    config.zero_padding = false;

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed error 0x%x", err);
        return err;
    }

    sensor_t *s = esp_camera_sensor_get();

    s->set_framesize(s, FRAMESIZE_VGA);

    return ESP_OK;
}

#define USE_NETCONN 1
#if USE_NETCONN
struct netconn *camera_conn;
struct netconn *command_conn;
struct ip_addr peer_addr;
#else
static int camera_sock;
static struct sockaddr_in senderinfo;
#endif

static void camera_tx(void *param)
{
    size_t packet_max = 32768;
#if USE_NETCONN
    struct netbuf *txbuf = netbuf_new();
#else
    senderinfo.sin_port = htons(55556);
#endif

    uint32_t frame_count = 0;
    uint32_t frame_size_sum = 0;
    int64_t start = esp_timer_get_time();

    while (1)
    {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb)
        {
            ESP_LOGE(TAG, "Camera capture failed");
            continue;
        }

        uint8_t *buf = fb->buf;
        size_t total = fb->len;
        size_t send = 0;

        // ESP_LOGI(TAG, "send %d", total);

        for (; send < total;)
        {
            size_t txlen = total - send;
            if (txlen > packet_max)
            {
                txlen = packet_max;
            }

#if USE_NETCONN
            err_t err = netbuf_ref(txbuf, &buf[send], txlen);
            if (err == ERR_OK)
            {
                err = netconn_sendto(camera_conn, txbuf, &peer_addr, 55556);
                if (err == ERR_OK)
                {
                    send += txlen;
                }
                else if (err == ERR_MEM)
                {
                    ESP_LOGW(TAG, "netconn_sendto ERR_MEM");
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                else
                {
                    ESP_LOGE(TAG, "netconn_sendto err %d", err);
                    break;
                }
            }
            else
            {
                ESP_LOGE(TAG, "netbuf_ref err %d", err);
                break;
            }
#else
            int err = sendto(camera_sock, &buf[send], txlen, 0, (struct sockaddr *)&senderinfo, sizeof(senderinfo));
            if (err < 0)
            {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                if (errno == ENOMEM)
                {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                else
                {
                    send = total;
                }
            }
            else if (err == 0)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            else
            {
                send += err;
            }
#endif
        }

        esp_camera_fb_return(fb);

        if (buf[0] == 0xff && buf[1] == 0xd8 && buf[2] == 0xff)
        {
            frame_count++;
        }
        frame_size_sum += total;

        int64_t end = esp_timer_get_time();
        int64_t pasttime = end - start;
        if (pasttime > 1000000)
        {
            start = end;
            float adj = 1000000.0 / (float)pasttime;
            float mbps = (frame_size_sum * 8.0 * adj) / 1024.0 / 1024.0;
            ESP_LOGI(TAG, "%.1f fps %.3f Mbps", frame_count * adj, mbps);
            frame_count = 0;
            frame_size_sum = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void do_actuation(char *cmd)
{
    int thr, brk, ang;
    int n = sscanf(cmd, "[%d;%d;%d]|", &thr, &brk, &ang);

    if (n != 3)
        return;

    ESP_LOGI(TAG, "thr: %d; brk: %d; ang: %d", thr, brk, ang);

    iot_servo_write_angle(MOTOR_SPEED_MODE, 1, map(ang, 0, 255, MAX_R, MAX_L));
    if (thr <= brk)
        iot_servo_write_angle(MOTOR_SPEED_MODE, 0, map(thr, 0, 255, MAX_FOR, 90));
    else
        iot_servo_write_angle(MOTOR_SPEED_MODE, 0, map(brk, 0, 255, MAX_REV, 90));
}

void command_rx(void *param)
{

    ESP_LOGI(TAG, "Command_rx started");

#if USE_NETCONN

    command_conn = netconn_new(NETCONN_UDP);
    if (command_conn == NULL)
    {
        ESP_LOGE(TAG, "netconn_new err");
        return;
    }

    err_t err = netconn_bind(command_conn, IPADDR_ANY, 3197);
    if (err != ERR_OK)
    {
        ESP_LOGE(TAG, "netconn_bind err %d", err);
        netconn_delete(command_conn);
        return;
    }

    servo_config_t servo_cfg = {
        .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = MOTOR_PWM_TIMER,
        .channels = {
            .servo_pin = {
                MOT_PIN,
                SER_PIN,
            },
            .ch = {
                MOTOR_PWM_CHANNEL_MOT,
                MOTOR_PWM_CHANNEL_SER,
            },
        },
        .channel_number = 2,
    };
    iot_servo_init(MOTOR_SPEED_MODE, &servo_cfg);

    ESP_LOGI(TAG, "Command_rx init done");

    while (1)
    {
        struct netbuf *rxbuf;
        if (netconn_recv(command_conn, &rxbuf) == ERR_OK)
        {
            uint8_t *data;
            u16_t len;
            netbuf_data(rxbuf, (void **)&data, &len);

            if (len)
            {
                char packet[255];
                for (size_t i = 0; i < len; i++)
                {
                    packet[i] = (char)data[i];
                }
                packet[len] = '\0';
                ESP_LOGD(TAG, "packet received: %s", packet);

                // If the message is correct [xxx;xxx;xxx]! do actuation
                if (packet[0] == '[' && packet[4] == ';' && packet[8] == ';' && packet[12] == ']')
                {
                    do_actuation(packet);
                }
            }
            netbuf_delete(rxbuf);
        }
    }
#endif
}

void start_camera(void)
{
#if USE_NETCONN
    camera_conn = netconn_new(NETCONN_UDP);
    if (camera_conn == NULL)
    {
        ESP_LOGE(TAG, "netconn_new err");
        return;
    }

    err_t err = netconn_bind(camera_conn, IPADDR_ANY, 55555);
    if (err != ERR_OK)
    {
        ESP_LOGE(TAG, "netconn_bind err %d", err);
        netconn_delete(camera_conn);
        return;
    }

    ESP_LOGI(TAG, "Wait a trigger...");
    while (1)
    {
        struct netbuf *rxbuf;
        err = netconn_recv(camera_conn, &rxbuf);
        if (err == ERR_OK)
        {
            uint8_t *data;
            u16_t len;
            netbuf_data(rxbuf, (void **)&data, &len);
            if (len)
            {
                ESP_LOGI(TAG, "netconn_recv %d", len);
                if (data[0] == 0x55)
                {
                    peer_addr = *netbuf_fromaddr(rxbuf);
                    ESP_LOGI(TAG, "peer %x", peer_addr.u_addr.ip4.addr);

                    ESP_LOGI(TAG, "Trigged!");
                    netbuf_delete(rxbuf);
                    break;
                }
            }
            netbuf_delete(rxbuf);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
#else
    camera_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (camera_sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return;
    }

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(55555);
    addr.sin_addr.s_addr = INADDR_ANY;
    bind(camera_sock, (struct sockaddr *)&addr, sizeof(addr));

    socklen_t addrlen = sizeof(senderinfo);
    char buf[128];
    ESP_LOGI(TAG, "Wait a trigger...");
    while (1)
    {
        int n = recvfrom(camera_sock, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&senderinfo, &addrlen);
        if (n > 0 && buf[0] == 0x55)
        {
            break;
        }
        ESP_LOGI(TAG, "recvfrom %d", n);
    }
#endif
    xTaskCreatePinnedToCore(&camera_tx, "camera_tx", 4096, NULL, 10, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&command_rx, "command_rx", 4096, NULL, 10, NULL, tskNO_AFFINITY);
}
