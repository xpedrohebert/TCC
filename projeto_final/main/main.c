/*===================================== Includes ====================================*/
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_chip_info.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_flash.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lvgl_port.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "FreeRTOSConfig.h"
#include "lvgl.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "sdkconfig.h"
#include "soc/soc_caps.h"

#include "esp_private/esp_clk.h"
#include "driver/mcpwm_cap.h"

#include "cJSON.h"

/*===================================== Defines =====================================*/
#define CLEAR_PIN 25
#define GPIO_26 26
#define GPIO_27 27
#define ESP_INTR_FLAG_DEFAULT 0
#define SCL                 18      // Serial Clock I2C pelo pino 18
#define SDA                 19      // Serial Data I2C pelo pino 19

// Parâmetros I2C
#define I2C_HOST            0
#define LCD_PIXEL_CLOCK_HZ  (400 * 1000)
#define RST_I2C             -1
#define I2C_HW_ADDR         0x3C
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306   // testa se o display é o ssd1306
    #define LCD_H_RES       128
    #define LCD_V_RES       64
#endif
#define LCD_CMD_BITS        8       // Quantidade de bits por comando
#define LCD_PARAM_BITS      8       // Quantidade de bits por parâmetro

/*====================================== Logs =======================================*/
static const char* SIST = "System";             // Level: info
static const char* TEST = "Test";               // Level: info
static const char* GPIO = "Gpio";               // Level: info
static const char* COUNT = "Count";             // Level: info
static const char* CALC = "Calculator";         // Level: info
static const char* DISP = "Display";            // Level: info
static const char* MQTT  = "MQTT";              // Level: info

/*===================================== Handles =====================================*/
mcpwm_cap_timer_handle_t cap_teste = NULL;
static QueueHandle_t text_queue = NULL;
static QueueHandle_t channel_queue = NULL;
static QueueHandle_t calculator_queue = NULL;
static QueueHandle_t gpio_queue = NULL;
static QueueHandle_t display_queue = NULL;

/*=================================== Structures ====================================*/
typedef struct {
    uint8_t edge;                               // Triggerd edge 
    uint8_t channel;                            // Triggerd channel
    uint32_t time;                              // Timer count
} Channel_t;

typedef struct {
    int count;                                  // Triggers cont
    float time;                                 // Time in seconds
} Calc_t;

typedef struct {
    float angle;                                // Angle in dgrees
    float abs_energy;                           // Absorbed energy
} Display_t;

/*==================================== Variables ====================================*/
float ANGLE_I = (5*M_PI)/6;
float GRAVITY = 9.80665;
float MASS = 19.964;
float RESOLUTION = 7.5;
float LENGTH = 0.0225;
bool COUNT_FLAG = true;
uintptr_t chanel1;
uintptr_t chanel2;
esp_mqtt_client_handle_t client;

/*==================================== Function =====================================*/
static double angle_calc_f(int count){
    float conf_f = (float) count;
    double angle;
    angle = (conf_f - 2) * RESOLUTION;//) * M_PI) / 180;
    return angle;
}

static float energy_calc_f(float angle_f) {
    float absorbed_energy = 300;
    float RFW = 0;
    absorbed_energy = MASS*GRAVITY*(sin(ANGLE_I - M_PI_2)-sin(angle_f-M_PI_2))-RFW;
    return absorbed_energy;
}

// I2C flush para printar no display
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_t * disp = (lv_disp_t *)user_ctx;
    lvgl_port_flush_ready(disp);
    return false;
}

// MQTT log error
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(MQTT, "Last error %s: 0x%x", message, error_code);
    }
}

// Function to publish the data to MQTT
void publish_data(Display_t display) {
    // cJSON *root = cJSON_CreateObject();
    // cJSON_AddNumberToObject(root, "angle", display.angle);
    // cJSON_AddNumberToObject(root, "abs_energy", display.abs_energy);
    // char *json_string;

    // asprintf(&json_string, "Ang.: %.2f", display.angle);
    // esp_mqtt_client_publish(client, "/topic/angle", json_string, 0, 1, 0);

    // asprintf(&json_string, "Energia_Abs: %.2f", display.abs_energy);
    // esp_mqtt_client_publish(client, "/topic/energy", json_string, 0, 1, 0);

    // cJSON_free(json_string);
    // cJSON_Delete(root);

    char *texto_mqtt;

    asprintf(&texto_mqtt, "Ang.: %.2f", display.angle);
    esp_mqtt_client_publish(client, "/topic/angle", texto_mqtt, 0, 1, 0);

    asprintf(&texto_mqtt, "Energia.: %.2f", display.abs_energy);
    esp_mqtt_client_publish(client, "/topic/energy", texto_mqtt, 0, 1, 0);

    free(texto_mqtt);
}

/*================================== Interruptions ==================================*/
static void IRAM_ATTR gpio_isr_handler(void *arg) {
    // Send to variable the gpio adress and converts into int type
    uint32_t gpio_number = (uint32_t) arg;
    // Send to queue giop int value
    xQueueSendFromISR(gpio_queue, &gpio_number, NULL);
}

/*==================================== Calbacks =====================================*/
static bool gpio_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    Channel_t channel;

    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) channel.edge = 1;
    if (edata->cap_edge == MCPWM_CAP_EDGE_NEG) channel.edge = 0;
    
    channel.time = edata->cap_value;
    if(cap_chan == chanel1) channel.channel = 1;
    if(cap_chan == chanel2) channel.channel = 2;

    xQueueSendFromISR(channel_queue, &channel, NULL);
    
    return high_task_wakeup == pdTRUE;
}

// MQTT event handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(MQTT, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    client = event->client;
    int msg_id;
    uint16_t duty_mqtt = 0;
    char *texto_mqtt;                // Cria o espaço para receber o texto

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(MQTT, "MQTT_EVENT_CONNECTED");
        // msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
        // ESP_LOGI(MQTT, "sent publish successful, msg_id=%d", msg_id);

        // Plotar o ângulo no MQTT
        // asprintf(&texto_mqtt, "Ang.: %.2f\nEnergia_Abs: %.2f", angulo,energia);
        // msg_id = esp_mqtt_client_publish(client, "/topic/angle", texto_mqtt, 0, 1, 0);
        // ESP_LOGI(MQTT, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/reset", 1);       
        ESP_LOGI(MQTT, "sent subscribe successful, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_subscribe(client, "/topic/angle", 1);       
        ESP_LOGI(MQTT, "sent subscribe successful, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_subscribe(client, "/topic/energy", 1);       
        ESP_LOGI(MQTT, "sent subscribe successful, msg_id=%d", msg_id);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(MQTT, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(MQTT, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/reset", "data", 0, 0, 0);
        ESP_LOGI(MQTT, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(MQTT, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(MQTT, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(MQTT, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        asprintf(&texto_mqtt, "%.*s", event->data_len, event->data);

        // Verificar se o tópico é "/topic/reset" para resetar a medição
        if (strncmp("/topic/reset", event->topic, (unsigned int)event->topic_len) == 0) {
            ESP_LOGI(MQTT, "Reset via MQTT");
            // xQueueSendToBack(gpio_queue, CLEAR_PIN, NULL);
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(MQTT, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(MQTT, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(MQTT, "Other event id:%d", event->event_id);
        break;
    }
    // free(texto_mqtt);
}

/*====================================== Tasks ======================================*/
/*------------------------------------ Text_task ------------------------------------*/
static void task_text(void* arg){
    int text_num;
    
    for(;;){
        if (xQueueReceive(text_queue, &text_num, 10/(portTICK_PERIOD_MS))){
            ESP_LOGI(TEST,"%d",text_num);
        }
    }
}

/*------------------------------------ Text_task ------------------------------------*/
static void gpio_task(void *arg) {
    uint32_t gpio_number;
    Display_t display_info;
    gpio_config_t io_config = {};
    io_config.mode = GPIO_MODE_INPUT;
    io_config.intr_type = GPIO_INTR_ANYEDGE;
    io_config.pull_down_en = 0;
    io_config.pull_up_en = 0;
    io_config.pin_bit_mask = (1ULL<<CLEAR_PIN);
    gpio_config(&io_config);

    for(;;) {
        if (xQueueReceive(gpio_queue, &gpio_number, portMAX_DELAY)) {
            if(gpio_number==CLEAR_PIN){
                ESP_LOGI(GPIO,"Clear\n");
                COUNT_FLAG = true;
                xQueueSendToBack(display_queue, &display_info, 20);
            }
        }
    }
}

/*------------------------------- Channel_count_task --------------------------------*/
static void channel_count_task(void* arg){
    Channel_t channel[3];
    Display_t display_info;
    channel[1].channel = 0; channel[1].edge = 0; channel[1].time = 0;
    Calc_t parameters = {
        .count = 0 ,
        .time = 0
    };
    
    for(;;){
        if (xQueueReceive(channel_queue, &channel[0], 10/(portTICK_PERIOD_MS))) {

            if(COUNT_FLAG == true){
                ESP_LOGI(COUNT,"Canal: %d",channel[0].channel);
                ESP_LOGI(COUNT,"Tempo: %lu",channel[0].time);
                if(channel[0].edge == 1) ESP_LOGI(COUNT,"Borda: subida\n");
                else ESP_LOGI(COUNT,"Borda: descida\n");
            
                parameters.count++;
                ESP_LOGI(COUNT,"Count: %d\n", parameters.count);

                if((channel[1].channel == channel[0].channel) && (channel[1].edge != channel[0].edge)) {
                    uint32_t raw_time = (channel[1].time - channel[0].time);
                    ESP_LOGI(COUNT,"Raw time: %lu\n", raw_time);
                    ESP_LOGI(COUNT,"Clock: %i\n", esp_clk_apb_freq());

                    parameters.time = raw_time * 0.00000000000125;
                    // parameters.time = raw_time * 0.0000000125*0.0001;
                    // parameters.time = raw_time * (10000000/esp_clk_apb_freq());
                    ESP_LOGI(COUNT,"Delta time: %f", parameters.time);
                    // parameters.time = raw_time/8;
                    // ESP_LOGI(COUNT,"Delta time: %f\n", parameters.time);

                    xQueueSendToBack(calculator_queue, &parameters, 20);
                    COUNT_FLAG = false;
                    parameters.count = 0;
                } 
            } else ESP_LOGI(COUNT,"Blocked"); 
            channel[1] = channel[0];
        }
    }
}

/*--------------------------------- Calculator_task ---------------------------------*/
static void calculator_task(void *arg){
    Calc_t parameters;
    Display_t display_info;
    float absorbed_energy = 300;
    float h, angle, total_angle, angle_fine;

    for(;;) {
        if (xQueueReceive(calculator_queue, &parameters, portMAX_DELAY)) {
            h = (pow(GRAVITY*parameters.time,2))/8;
            angle = angle_calc_f(parameters.count);
            ESP_LOGI(CALC,"Angle_min = %.1f\n",angle);
            ESP_LOGI(CALC,"Pi = %f\n", M_PI);
            angle_fine = asin(h/LENGTH)*180/M_PI;
            total_angle = angle + angle_fine;
            angle = angle_calc_f(parameters.count+1);
            ESP_LOGI(CALC,"Angle_max = %.1f\n",angle);
            ESP_LOGI(CALC,"Angle = %.1f\n",total_angle);

            ESP_LOGI(CALC,"time = %f", parameters.time);
            ESP_LOGI(CALC,"h = %.4f", h);
            ESP_LOGI(CALC,"Angle_fine = %.4f\n", angle_fine);
            
            absorbed_energy = energy_calc_f(total_angle);
            ESP_LOGI(CALC,"Absorbed Energy = %f\n",absorbed_energy);
            
            display_info.angle = total_angle;
            display_info.abs_energy = absorbed_energy;

            publish_data(display_info);  // Publica no MQTT
            xQueueSendToBack(display_queue, &display_info, 20);         // Publica no display
        }
    }
}

/*---------------------------------- Display_task -----------------------------------*/
static void display_task(void *arg) {
    ESP_LOGI(DISP, "Initialize I2C bus");
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA,
        .scl_io_num = SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = LCD_PIXEL_CLOCK_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_HOST, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(DISP, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = I2C_HW_ADDR,
        .control_phase_bytes = 1,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .dc_bit_offset = 6,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_HOST, &io_config, &io_handle));

    ESP_LOGI(DISP, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = RST_I2C,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(DISP, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = LCD_H_RES * LCD_V_RES,
        .double_buffer = true,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }
    };
    
    lv_disp_t * disp = lvgl_port_add_disp(&disp_cfg);
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, disp);

    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);

    Display_t display;
    char *texto_display;

    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_t *angle_label = lv_label_create(scr);
    lv_obj_t *abs_energy_label = lv_label_create(scr);
    lv_obj_t *status_label = lv_label_create(scr);

    //Initial condition
    
    asprintf(&texto_display, "Iniciando");
    lv_label_set_text(angle_label, texto_display);
    lv_obj_align(angle_label, LV_ALIGN_TOP_MID,0,0);
    asprintf(&texto_display, "Sistema");
    lv_label_set_text(abs_energy_label, texto_display);
    lv_obj_align(abs_energy_label, LV_ALIGN_CENTER,0,0);
    asprintf(&texto_display, " ");
    lv_label_set_text(status_label, texto_display);
    lv_obj_align(status_label, LV_ALIGN_BOTTOM_MID,0,0);
    free(texto_display);

    for(;;){
        if(xQueueReceive(display_queue, &display, portMAX_DELAY)){
            ESP_LOGI(DISP, "Entrou na task display\n");

            if(COUNT_FLAG == false){

                asprintf(&texto_display, "Ang: %.2f°", display.angle);
                ESP_LOGI(DISP, "Ang.: %.2f°", display.angle);
                lv_label_set_text(angle_label, texto_display);
                lv_obj_align(angle_label, LV_ALIGN_TOP_MID,0,0);

                asprintf(&texto_display, "E_Abs: %.2fJ", display.abs_energy);
                ESP_LOGI(DISP, "E_Abs.: %.2fJ", display.abs_energy);
                lv_label_set_text(abs_energy_label, texto_display);
                lv_obj_align(abs_energy_label, LV_ALIGN_CENTER,0,0);

                asprintf(&texto_display, "Bloqueado");
                lv_label_set_text(status_label, texto_display);
                lv_obj_align(status_label, LV_ALIGN_BOTTOM_MID,0,0);

            } else {

                asprintf(&texto_display, "Calculando");
                lv_label_set_text(angle_label, texto_display);
                lv_obj_align(angle_label, LV_ALIGN_TOP_MID,0,0);

                asprintf(&texto_display, "Aguarde");
                lv_label_set_text(abs_energy_label, texto_display);
                lv_obj_align(abs_energy_label, LV_ALIGN_CENTER,0,0);

                asprintf(&texto_display, " ");
                lv_label_set_text(status_label, texto_display);
                lv_obj_align(status_label, LV_ALIGN_BOTTOM_MID,0,0);
            }
            free(texto_display);
        }
    }
}

/*------------------------------------ MQTT_task ------------------------------------*/
static void MQTT_task(){
    ESP_LOGI(MQTT, "[APP] Startup..");
    ESP_LOGI(MQTT, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(MQTT, "[APP] IDF version: %s", esp_get_idf_version());
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper functon configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://device_5:device_5@node02.myqtthub.com:1883",
        // CONFIG_BROKER_URL: username, senha
        .credentials.client_id = "device_5", // ID
    };
    #if CONFIG_BROKER_URL_FROM_STDIN
        char line[128];

        if (strcmp(mqtt_cfg.broker.address.uri, "FROM_STDIN") == 0) {
            int count = 0;
            printf("Please enter url of mqtt broker\n");
            while (count < 128) {
                int c = fgetc(stdin);
                if (c == '\n') {
                    line[count] = '\0';
                    break;
                } else if (c > 0 && c < 127) {
                    line[count] = c;
                    ++count;
                }
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            mqtt_cfg.broker.address.uri = line;
            printf("Broker url: %s\n", line);
        } else {
            ESP_LOGE(MQTT, "Configuration mismatch: wrong broker url");
            abort();
        }
    #endif /* CONFIG_BROKER_URL_FROM_STDIN */

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    // Loop com delay para preservar a task
    while(1){
        vTaskDelay(100);
    }
}

/*====================================== Main =======================================*/
void app_main(void) {

    esp_log_level_set(SIST,ESP_LOG_WARN);
    esp_log_level_set(TEST,ESP_LOG_WARN);
    esp_log_level_set(GPIO,ESP_LOG_WARN);
    esp_log_level_set(COUNT,ESP_LOG_WARN);
    // esp_log_level_set(CALC, ESP_LOG_WARN);
    // esp_log_level_set(DISP, ESP_LOG_WARN);
    // esp_log_level_set(MQTT, ESP_LOG_WARN);

    // Criação das 
    text_queue = xQueueCreate(1, sizeof(uint8_t));
    gpio_queue = xQueueCreate(1, sizeof(uint32_t));
    channel_queue = xQueueCreate(10, sizeof(Channel_t));
    calculator_queue = xQueueCreate(1, sizeof(Calc_t));
    display_queue = xQueueCreate(1, sizeof(Display_t));

    xTaskCreate(task_text, "task_text", 2048, NULL, 11, NULL);
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 1, NULL);
    xTaskCreate(channel_count_task, "channel_count_task", 2048, NULL, 1, NULL);
    xTaskCreate(calculator_task, "calculator_task", 2048, NULL, 1, NULL);
    xTaskCreate(display_task, "display_task", 4096, NULL, 10, NULL);
    xTaskCreate(MQTT_task, "MQTT_task", 4096, NULL, 10, NULL);           // Cria task do MQTT

    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &cap_timer));

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(CLEAR_PIN, gpio_isr_handler, (void *)CLEAR_PIN);
    gpio_set_intr_type(CLEAR_PIN, GPIO_INTR_ANYEDGE);

    //Creation
    mcpwm_cap_channel_handle_t cap_chan    = NULL;
    mcpwm_cap_channel_handle_t cap_chan2    = NULL;

    mcpwm_capture_channel_config_t cap_ch_conf = {
        .gpio_num = GPIO_26,
        .prescale = 1,
        // capture on both edge
        .flags.neg_edge = true,
        .flags.pos_edge = true,
        // pull up internally
        .flags.pull_up = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan));

    mcpwm_capture_channel_config_t cap_ch_conf2 = {
        .gpio_num = GPIO_27,
        .prescale = 1,
        // capture on both edge
        .flags.neg_edge = true,
        .flags.pos_edge = true,
        // pull up internally
        .flags.pull_up = true,
    };//cap_ch_conf2.gpio_num
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf2, &cap_chan2));

    ESP_LOGI(SIST, "Register capture callback");
    TaskHandle_t cur_task = xTaskGetCurrentTaskHandle();
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = gpio_callback,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, cur_task));
    mcpwm_capture_event_callbacks_t cbs2 = {
        .on_cap = gpio_callback,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan2, &cbs2, cur_task));

    ESP_LOGI(SIST, "Enable capture channel");
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan2));

    ESP_LOGI(SIST, "Enable and start capture timer");
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

    chanel1 = cap_chan;
    chanel2 = cap_chan2;

    for(;;) {

        // ESP_LOGI(SIST, "%p", cap_teste);
        // ESP_LOGI(SIST, "Chanel1: %p", cap_chan);
        // ESP_LOGI(SIST, "Chanel2: %p", cap_chan2);
        // ESP_LOGI(SIST, "Chanel1: %#x", chanel1);
        // ESP_LOGI(SIST, "Chanel2: %#x", chanel2);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}














// #include <stdio.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/event_groups.h"
// #include "freertos/semphr.h"
// #include "freertos/queue.h"
// #include "freertos/event_groups.h"
// #include "esp_system.h"
// #include "esp_wifi.h"
// #include "esp_event.h"
// #include "esp_log.h"
// #include "mqtt_client.h"
// #include "cJSON.h"

// #define WIFI_SSID       "your_SSID"
// #define WIFI_PASS       "your_PASSWORD"
// #define MQTT_BROKER     "mqtt_broker_address"
// #define MQTT_TOPIC      "your/mqtt/topic"

// static const char *TAG = "mqtt_example";
// static EventGroupHandle_t wifi_event_group;
// static esp_mqtt_client_handle_t mqtt_client;

// typedef struct {
//     float angle;
//     float abs_energy;
// } Display_t;

// // Function to publish the data to MQTT
// void publish_data(float angle, float abs_energy) {
//     cJSON *root = cJSON_CreateObject();
//     cJSON_AddNumberToObject(root, "angle", angle);
//     cJSON_AddNumberToObject(root, "abs_energy", abs_energy);

//     char *json_string = cJSON_PrintUnformatted(root);
//     esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, json_string, 0, 1, 0);

//     cJSON_free(json_string);
//     cJSON_Delete(root);
// }

// // MQTT event handler
// static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {
//     switch (event->event_id) {
//         case MQTT_EVENT_CONNECTED:
//             ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
//             break;
//         case MQTT_EVENT_DISCONNECTED:
//             ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
//             break;
//         default:
//             break;
//     }
//     return ESP_OK;
// }

// // WiFi event handler
// static esp_err_t wifi_event_handler(void *ctx, system_event_t *event) {
//     switch (event->event_id) {
//         case SYSTEM_EVENT_STA_START:
//             esp_wifi_connect();
//             break;
//         case SYSTEM_EVENT_STA_GOT_IP:
//             xEventGroupSetBits(wifi_event_group, BIT0);
//             break;
//         case SYSTEM_EVENT_STA_DISCONNECTED:
//             esp_wifi_connect();
//             xEventGroupClearBits(wifi_event_group, BIT0);
//             break;
//         default:
//             break;
//     }
//     return ESP_OK;
// }

// // Main application
// void app_main() {
//     ESP_ERROR_CHECK(nvs_flash_init());
//     wifi_event_group = xEventGroupCreate();

//     // Initialize and start WiFi
//     tcpip_adapter_init();
//     ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));
//     ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
//     wifi_config_t wifi_config = {
//         .sta = {
//             .ssid = WIFI_SSID,
//             .password = WIFI_PASS,
//         },
//     };
//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
//     ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
//     ESP_ERROR_CHECK(esp_wifi_start());

//     // Wait for WiFi connection
//     xEventGroupWaitBits(wifi_event_group, BIT0, false, true, portMAX_DELAY);

//     // Initialize and start MQTT
//     esp_mqtt_client_config_t mqtt_cfg = {
//         .uri = MQTT_BROKER,
//     };
//     mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
//     esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client);
//     esp_mqtt_client_start(mqtt_client);

//     // Main loop
//     while (1) {
//         // Your application logic here
//         // For example, sending data to MQTT
//         Display_t display_data;
//         display_data.angle = 45.0;
//         display_data.abs_energy = 10.5;

//         publish_data(display_data.angle, display_data.abs_energy);

//         vTaskDelay(5000 / portTICK_RATE_MS);  // Adjust the delay based on your requirements
//     }
// }