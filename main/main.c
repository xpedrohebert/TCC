// Buscando as bibliotecas necessárias
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

// Definições

// Definindo o tempo de estouro do timer
// #define ESTOURO_RELOGIO  1000             // Estouro do relógio a cada 1 ms
// #define ESTOURO_RELOGIO  10000            // Estouro do relógio a cada 10 ms
#define ESTOURO_RELOGIO     100000           // Estouro do relógio a cada 100 ms
// #define ESTOURO_RELOGIO  1000000          // Estouro do relógio a cada 1 s

// Parâmetros interrupção
#define flag_int            0

// Entradas
#define botao_0             21
#define botao_1             22
#define botao_2             23
#define gpio_entrada        ((1ULL<<botao_0) | (1ULL<<botao_1) | (1ULL<<botao_2))

// Saídas
#define led_azul            2
#define rgb_verde           16
#define rgb_vermelho        17
#define rgb_azul            26
#define osciloscopio_32     32
#define osciloscopio_33     33
#define gpio_saida          ((1ULL<<led_azul) | (1ULL<<rgb_vermelho) | (1ULL<<rgb_verde) | (1ULL<<rgb_azul) | (1ULL<<osciloscopio_32) | (1ULL<<osciloscopio_33))

// Pinos comunicação
#define RXD_PIN             4       // Recepção UART pelo pino 4
#define TXD_PIN             5       // Transmite UART pelo pino 5
#define SCL                 18      // Serial Clock I2C pelo pino 18
#define SDA                 19      // Serial Data I2C pelo pino 19

// Parâmetros ADC
#define adc1_ch0            ADC_CHANNEL_3   // Define canal 3 do ADC (3) como entrada
#define adc_attenuation     ADC_ATTEN_DB_11 // Define a atenuação do ADC

// Parâmetros PWM
#define LEDC_FREQUENCY      (5000)               // Frequency in Hertz. Set frequency at 5 kHz

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

// ******************************************** //
// Pinos PWM da placa utilizada                 //
// Pino 16 -> Verde                             //
// Pino 17 -> Vermelho                          //
// Pino 26 -> Azul                              //
// Pino 32 -> Conectado na entrada do filtro PB //
// Pino 33 -> Desconectado                      //
// ******************************************** //

// Logs
static const char* SIS   = "Sistema";       
static const char* TAG   = "Teste";         
static const char* TIMER = "Relógio";       
static const char* PWM   = "PWM";           
static const char* ADC   = "ADC";           
static const char* RX    = "RX (UART)";     
static const char* TX    = "TX (UART)";     
static const char* DISP  = "Display";       
static const char* MQTT  = "MQTT";          

// Estruturas
typedef struct {
    uint64_t contagem_atual;                // Campo de contagem atual
    uint64_t valor_alarme;                  // Campo de valor do alarme
} estrutura_alarme_t;
estrutura_alarme_t alarme;

typedef struct {
    uint8_t dec;                            // Campo décimo de segundo
    uint8_t segundo;                        // Campo segundo
    uint8_t minuto;                         // Campo minuto
    uint8_t hora;                           // Campo hora
} estrutura_relogio_t;

typedef struct {
    bool modo_PWM;                          // Campo modo do PWM (automático = true ou manual = false)
    uint16_t duty_PWM;                      // Campo duty cycle do PWM (0 a 8191)
} PWM_elements_t;

typedef struct {
    uint16_t valor;                         // Valor bruto do ADC
    uint16_t calibrado;                     // Valor calibrado (em mV)
} ADC_elements_t;

typedef struct {
    estrutura_relogio_t relogio;            // Relógio
    ADC_elements_t adc;                     // Leitura do ADC
} estrutura_display_t;

static QueueHandle_t gpio_evt_queue = NULL;
static SemaphoreHandle_t semaphore_pwm = NULL;
static SemaphoreHandle_t semaphore_adc = NULL;
gptimer_handle_t timer_relogio;             // Criando a variável do timer   
QueueHandle_t timer_int_queue;
QueueHandle_t gpio_pwm_queue;
QueueHandle_t timer_adc_queue;
QueueHandle_t rtc_adj_queue;
QueueHandle_t timer_display_queue;

// Variáveis globais
PWM_elements_t pwm;                         // Variável global que armazena o estado do PWM
static const int RX_BUF_SIZE = 1024;


// ************************** FUNÇÕES ****************************************************************************

// Incrementa o valor atual do duty cycle do PWM e retorna o novo valor
static uint16_t increment_dutycicle(uint16_t duty){
    duty = duty + 1638;
    if(duty > 8191){
        duty = 1;
    }
    return duty;
}

// Atualiza a saída do PWM segundo o duty cycle recebido
static void atualiza_dutycycle(uint8_t canal, uint16_t duty){
    // ESP_LOGI(PWM, "\nFunção 2: Canal:  %u", canal);
    // ESP_LOGI(PWM, "Duty (0 a 8191): %u\n", duty);
    if(canal == 1){     // PWM rgb_vermelho e osciloscopio_32
        // Seta o duty cycle dos 2 PWMs
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, duty));
        // Update duty to apply the new value nos 2 PWMs
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1));
        // ESP_LOGI(PWM, "PWM:  %u", pwm.duty_PWM);            // Imprime o valor do duty cycle do PWM a cada incremento
    }
    if(canal == 2){     // PWM rgb_verde
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, duty));     // Seta o duty cycle
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2));        // Update duty to apply the new value
        // ESP_LOGI(PWM, "PWM:  %u", pwm.duty_PWM);            // Imprime o valor do duty cycle do PWM a cada incremento
        ESP_LOGI(PWM, "\nFunção 2: Canal:  %u", canal);
        ESP_LOGI(PWM, "Duty (0 a 8191): %u\n", duty);
    }
    if(canal == 3){     // PWM rgb_azul
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, duty));     // Seta o duty cycle
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3));        // Update duty to apply the new value
        // ESP_LOGI(PWM, "PWM:  %u", pwm.duty_PWM);            // Imprime o valor do duty cycle do PWM a cada incremento
        ESP_LOGI(PWM, "\nFunção 2: Canal:  %u", canal);
    ESP_LOGI(PWM, "Duty (0 a 8191): %u\n", duty);
    }
}

// Recebe o dutycycle em percentual (0 a 100) e atualiza a saída do PWM
static void atualiza_dutycycle_percent(uint8_t canal, uint16_t duty){
    ESP_LOGI(PWM, "\nFunção 1: Canal:  %u", canal);
    ESP_LOGI(PWM, "Duty (0 a 100):  %u", duty);
    duty = (duty*8191)/100;
    ESP_LOGI(PWM, "Duty (0 a 8191): %u\n", duty);
    if (duty < 1){
        ESP_LOGI(PWM, "Valor inválido! PWM setado em nível baixo!");
        duty = 0;
    } 
    if (duty > 8191){
        ESP_LOGI(PWM, "Valor inválido! PWM setado em nível alto!");
        duty = 8191;
    }
    atualiza_dutycycle(canal, duty);
}

// Calibração ADC
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        if (!calibrated) {
            ESP_LOGI(ADC, "calibration scheme version is %s", "Curve Fitting");
            adc_cali_curve_fitting_config_t cali_config = {
                .unit_id = unit,
                .atten = atten,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
            if (ret == ESP_OK) {
                calibrated = true;
            }
        }
    #endif

    #if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        if (!calibrated) {
            ESP_LOGI(ADC, "calibration scheme version is %s", "Line Fitting");
            adc_cali_line_fitting_config_t cali_config = {
                .unit_id = unit,
                .atten = atten,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
            if (ret == ESP_OK) {
                calibrated = true;
            }
        }
    #endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(ADC, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(ADC, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(ADC, "Invalid arg or no memory");
    }

    return calibrated;
}

// Printa na UART
int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
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

// **************************** INTERRUPÇÕES *********************************************************************

// Interrupção GPIO (Prática 2)
static void IRAM_ATTR gpio_interrupcao(void* arg){
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// Interrupção timer (Prática 3)
static bool IRAM_ATTR interrupt_1(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data){
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = edata -> alarm_value + ESTOURO_RELOGIO,  // Atualiza o alarme segundo o tempo definido
    };
    gptimer_set_alarm_action(timer_relogio, &alarm_config);
    // ESP_LOGI(TIMER, "entrou na interrupção");
    BaseType_t high_task_awoken = pdFALSE;
    
    estrutura_alarme_t alarme2;
    alarme2.valor_alarme = edata -> alarm_value + 1;            // Atribui o valor do alarme da interrupção pra fila
    alarme2.contagem_atual = edata -> count_value;              // Atribui o valor do contador da interrupção pra fila

    xQueueSendFromISR(timer_int_queue, &alarme2, NULL);         // Joga a estrutura alarme2 pra fila do timer
    return (high_task_awoken == pdTRUE);
}

// MQTT event handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(MQTT, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    uint16_t duty_mqtt = 0;
    char *texto_mqtt;                // Cria o espaço para receber o texto

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(MQTT, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
        ESP_LOGI(MQTT, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(MQTT, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);       // LED vermelho
        ESP_LOGI(MQTT, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos2", 2);       // LED verde
        ESP_LOGI(MQTT, "sent subscribe successful, msg_id=%d", msg_id);
        
        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos3", 3);       // LED azul
        ESP_LOGI(MQTT, "sent subscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(MQTT, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(MQTT, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
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

        // Verificar se o tópico é "/topic/qos1" para atualizar o LED VERMELHO
        if (strncmp("/topic/qos1", event->topic, (unsigned int)event->topic_len) == 0) {
            duty_mqtt = (uint16_t)atoi(texto_mqtt);
            ESP_LOGI(MQTT, "LED VERMELHO: %d", duty_mqtt);
            // duty_mqtt    = (data[0] - '0') * 100 + data[1] - '0') * 10 + (data[2] - '0');
            atualiza_dutycycle_percent(1, duty_mqtt);
        }
        // Verificar se o tópico é "/topic/qos2" para atualizar o LED VERDE
        if (strncmp("/topic/qos2", event->topic, (unsigned int)event->topic_len) == 0) {
            duty_mqtt = (uint16_t)atoi(texto_mqtt);
                ESP_LOGI(MQTT, "LED VERDE: %d", duty_mqtt);
            // duty_mqtt    = (data[0] - '0') * 100 + data[1] - '0') * 10 + (data[2] - '0');
            atualiza_dutycycle_percent(2, duty_mqtt);
        }
        // Verificar se o tópico é "/topic/qos3" para atualizar o LED AZUL
        if (strncmp("/topic/qos3", event->topic, (unsigned int)event->topic_len) == 0) {
            ESP_LOGI(MQTT, "LED AZUL STRING: %s", event->data);
            duty_mqtt = (uint16_t)atoi(texto_mqtt);
            ESP_LOGI(MQTT, "LED AZUL ATOI: %d", duty_mqtt);
            // duty_mqtt    = (data[0] - '0') * 100 + data[1] - '0') * 10 + (data[2] - '0');
            atualiza_dutycycle_percent(3, duty_mqtt);
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

// ************************** TASKS ******************************************************************************

// Task GPIO (Prática 2)
static void task_gpio(void* arg){
    uint64_t teste_input = gpio_entrada, teste_output = gpio_saida;
    ESP_LOGI(SIS,"%" PRIu64 "|%" PRIu64, teste_input, teste_output);    //Teste da mascara

    gpio_config_t io_conf = {}; 
    io_conf.intr_type = GPIO_INTR_DISABLE;                              //Desabilitando interrupções
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = gpio_saida;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_NEGEDGE;                              //Interrupção em borda de descida
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = gpio_entrada;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //install gpio isr service
    gpio_install_isr_service(flag_int);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(botao_0, gpio_interrupcao, (void*) botao_0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(botao_1, gpio_interrupcao, (void*) botao_1);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(botao_2, gpio_interrupcao, (void*) botao_2);

    uint32_t io_num;
    pwm.modo_PWM = true;
    pwm.duty_PWM = 1;
    gpio_set_level(led_azul,pwm.modo_PWM);
    
    for(;;) {
        // Talvez o xSemaphoreTake devesse estar dentro de um if
        xSemaphoreTake(semaphore_pwm, portMAX_DELAY);   // Coloca a task em espera, até ser habilitada novamente pelo estouro do timer
        if (xQueueReceive(gpio_evt_queue, &io_num, 10/(portTICK_PERIOD_MS))){
            ESP_LOGI(TAG,"GPIO[%"PRIu32"]", io_num);
            
            // Prática 2 controle LED azul por meio dos botões 0, 1 e 2
            // if(io_num==botao_0) gpio_set_level(led_azul,pwm.modo_PWM=true);      // Botão 0 liga LED azul
            // if(io_num==botao_1) gpio_set_level(led_azul,pwm.modo_PWM=false);     // Botão 1 desliga LED azul
            // if(io_num==botao_2) {                                                // Botão 2 troca o estado do LED azul
            //     pwm.modo_PWM=!pwm.modo_PWM;
            //     gpio_set_level(led_azul,pwm.modo_PWM);
            // }

            // Prática 4 controle PWM por meio dos botões 0, 1 e 2
            if(io_num==botao_0){
                pwm.modo_PWM = true;
                gpio_set_level(led_azul,pwm.modo_PWM);          // Botão 0 PWM automático
                }
            if(io_num==botao_1){
                pwm.modo_PWM = false;
                gpio_set_level(led_azul,pwm.modo_PWM);          // Botão 1 PWM manual
            }
            if(io_num == botao_2 && pwm.modo_PWM == false){     // Botão 2 incrementa duty cycle em 10%
                pwm.duty_PWM = increment_dutycicle(pwm.duty_PWM);
            }
            ESP_LOGI(PWM, "Duty Cycle do PWM: %u", pwm.duty_PWM);           // Imprime o valor do duty cycle atual do PWM
            ESP_LOGI(PWM, "Modo de Atuação do PWM: %u",pwm.modo_PWM);       // Imprime o modo atual do PWM
        }
        xQueueSendToBack(gpio_pwm_queue, &pwm, NULL);
    }
}

// Task timer (Prática 3)
static void task_timer(void *arg){
    // Declarando as variáveis necessárias e setando em 0
    estrutura_relogio_t relogio;
    relogio.dec = 0;
    relogio.segundo = 0;
    relogio.minuto = 0;
    relogio.hora = 0;
    estrutura_relogio_t rtc;
    rtc.dec = 0;
    rtc.segundo = 0;
    rtc.minuto = 0;
    rtc.hora = 0;
    estrutura_alarme_t alarme;
    alarme.contagem_atual = 0;
    alarme.valor_alarme = 0 ;
    timer_relogio = NULL;
    ADC_elements_t adc;
    adc.calibrado = 0;
    adc.valor = 0;
    estrutura_display_t display;
    display.relogio.hora = 0;
    display.relogio.minuto = 0;
    display.relogio.segundo = 0;
    display.adc.calibrado = 0;

    ESP_LOGI(TIMER, "Configurando o timer");     
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,             // Clock de 80Mhz
        .direction = GPTIMER_COUNT_UP,                  // Contando pra cima, 
        .resolution_hz = 1000000,                       // Clock de 1Mhz
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer_relogio));                  // Enviando tudo pro timer
    gptimer_event_callbacks_t cbs = {
        .on_alarm = interrupt_1,                                // Associando callback/interrupção
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_relogio, &cbs, NULL));    // Envia esta informação

    gptimer_alarm_config_t alarm_config1 = {    
        .alarm_count = ESTOURO_RELOGIO,                                 // Configurando o alarme segundo o tempo definido
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer_relogio, &alarm_config1));      //envia configuração do alarme
    ESP_LOGI(TIMER, "Set count value");
    ESP_ERROR_CHECK(gptimer_set_raw_count(timer_relogio, 0));   //contagem começando em 100
    ESP_LOGI(TIMER, "Get count value");
    ESP_LOGI(TIMER, "configurou alarme");
    ESP_LOGI(TIMER, "energiza o timer");                      
    ESP_ERROR_CHECK(gptimer_enable(timer_relogio));             //energizando o timer
    ESP_ERROR_CHECK(gptimer_start(timer_relogio));             //inicia contagem do timer
    ESP_LOGI(TIMER, "iniciou a contagem");

    for(;;){
        if (xQueueReceive(rtc_adj_queue, &rtc, 1/(portTICK_PERIOD_MS))){            // Espera 1 ms por uma entrada da fila rtc_adj
            relogio.segundo = rtc.segundo;
            relogio.minuto = rtc.minuto;
            relogio.hora = rtc.hora;
            sendData(TX, "ok");
        }

        if (xQueueReceive(timer_int_queue, &alarme, pdMS_TO_TICKS(2000))) {
            relogio.dec++;
            if(relogio.dec==10){
                relogio.segundo++;
                relogio.dec=0;

                if(relogio.segundo==60){
                    relogio.minuto++;
                    relogio.segundo=0;

                    if(relogio.minuto==60){
                        relogio.hora++;
                        relogio.minuto=0;
                    
                        if(relogio.hora==24){
                            relogio.hora=0;
                        }
                    }
                }
                ESP_LOGI(TIMER, "%02u:%02u:%02u", relogio.hora,relogio.minuto,relogio.segundo);     // Imprime o valor do relogio
                ESP_LOGD(TIMER, "Valor do alarme: %llu", alarme.valor_alarme);                      // Imprime o valor do alarme atualizado
                ESP_LOGD(TIMER, "Contagem atual:  %llu", alarme.contagem_atual);                    // Imprime o valor da contagem atual

                if(xQueueReceive(timer_adc_queue, &adc, 1/(portTICK_PERIOD_MS))){                   // Espera 1 ms por uma entrada da fila timer_adc
                    ESP_LOGI(ADC, "Valor Bruto:     %d", adc.valor);
                    ESP_LOGI(ADC, "Valor Calibrado: %d", adc.calibrado);
                }

                display.relogio = relogio;
                display.adc = adc;
                xQueueSendToBack(timer_display_queue, &display, NULL);
                
                ESP_LOGD(DISP, "Dados enviados para a task do display\n");
            }
            ESP_LOGD(TAG, "chegou no semaforo");
            
            xSemaphoreGive(semaphore_pwm);      // Função na TASK Timer para sincronizar com a task PWM
            xSemaphoreGive(semaphore_adc);      // Função na TASK Timer para sincronizar com a task ADC
        }
        else {
            ESP_LOGW(TIMER, "Missed one count event");        // Perdeu um evento
        }  
    }
}

//Task PWM  (Prática 4)
static void task_PWM(void* arg){
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configura a saída PWM osciloscopio_32
    ledc_channel_config_t ledc_channel_0 = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = osciloscopio_32,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_0));
    
    // Configura a saída PWM rgb_vermelho
    ledc_channel_config_t ledc_channel_1 = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = rgb_vermelho,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_1));

    // Configura a saída PWM rgb_verde
    ledc_channel_config_t ledc_channel_2 = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_2,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = rgb_verde,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_2));

    // Configura a saída PWM rgb_azul
    ledc_channel_config_t ledc_channel_3 = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_3,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = rgb_azul,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_3));
    
    for(;;){
        if(xQueueReceive(gpio_pwm_queue, &pwm, portMAX_DELAY)){
            if(pwm.modo_PWM == true){
                pwm.duty_PWM = increment_dutycicle(pwm.duty_PWM);
                ESP_LOGD(PWM, "PWM2:  %u", pwm.duty_PWM); 
            }
            atualiza_dutycycle(1,pwm.duty_PWM);
        } 
    }  
}    

//Task ADC (Prática 5)
static void task_ADC(void* arg){
    ESP_LOGD(ADC, "ENTREI NO ADC");
    // ADC1 Init
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // ADC1 Config
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = adc_attenuation,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, adc1_ch0, &config));

    // ADC1 Calibration Init
    adc_cali_handle_t adc1_cali_handle = NULL;
    bool do_calibration1 = adc_calibration_init(ADC_UNIT_1, adc_attenuation, &adc1_cali_handle);

    ADC_elements_t adc;
    adc.calibrado = 0;
    adc.valor = 0;

    for(;;){
        // ESP_LOGD(ADC, "ENTREI NO FOR");
        xSemaphoreTake(semaphore_adc, portMAX_DELAY);   // Coloca a task em espera, até ser habilitada novamente pelo estouro do timer
            
        // Leitura ADC
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, adc1_ch0, &adc.valor));

        if (do_calibration1) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc.valor, &adc.calibrado));
        }
        xQueueSendToBack(timer_adc_queue, &adc, NULL);
    }
}

//Task UART (Prática 6)
static void task_uart(void *arg)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    uint8_t* teste = (uint8_t*) malloc(RX_BUF_SIZE+1);
    estrutura_relogio_t rtc;
    rtc.dec = 0;
    rtc.segundo = 0;
    rtc.minuto = 0;
    rtc.hora = 0;

    for(;;) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX, "data: Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX, teste, rxBytes, ESP_LOG_INFO);

            rtc.hora    = (data[0] - '0') * 10 + (data[1] - '0');
            rtc.minuto  = (data[2] - '0') * 10 + (data[3] - '0');
            rtc.segundo = (data[4] - '0') * 10 + (data[5] - '0');

            ESP_LOGI(RX, "Valores extraídos: H: %2d M: %2d S: %2d", rtc.hora, rtc.minuto, rtc.segundo);

            if(rtc.hora < 24){
                if(rtc.minuto < 60){
                    if(rtc.segundo < 60){
                        xQueueSendToBack(rtc_adj_queue, &rtc, NULL);
                    }
                    else sendData(TX, "Hora inválida");
                }
                else sendData(TX, "Hora inválida");
            }
            else sendData(TX, "Hora inválida");
        }
    }
    free(data);
}

//Task Display e I²C (Prática 7)
static void task_display(void *arg){
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
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = LCD_PARAM_BITS, // According to SSD1306 datasheet
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
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
    /* Register done callback for IO */
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, disp);

    /* Rotation of the screen */
    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);

    // Declarando as variáveis necessárias e setando em 0
    estrutura_display_t display;
    display.relogio.hora = 0;
    display.relogio.minuto = 0;
    display.relogio.segundo = 0;
    display.adc.calibrado = 0;
    char *texto_display;                // Cria o espaço para receber o texto a ser escrito no display

    // Criando as labels para plotar no display
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_t *label_relogio = lv_label_create(scr);
    lv_obj_t *label_tensao = lv_label_create(scr);

    for(;;){
        if(xQueueReceive(timer_display_queue, &display, portMAX_DELAY)){
            ESP_LOGD(DISP, "Entrou na task display\n");

            // asprintf não retorna uma string diretamente, mas aloca espaço de memória para a string formatada e a armazena nesse espaço
            asprintf(&texto_display, "%02u:%02u:%02u", display.relogio.hora, display.relogio.minuto, display.relogio.segundo);
            lv_label_set_text(label_relogio, texto_display);    // Recebe uma string, não um ponteiro
            lv_obj_align(label_relogio, LV_ALIGN_TOP_MID,0,0);
            ESP_LOGD(DISP, "%02u:%02u:%02u", display.relogio.hora, display.relogio.minuto, display.relogio.segundo);
            
            asprintf(&texto_display, "Tensao: %d mV", display.adc.calibrado);
            lv_label_set_text(label_tensao, texto_display);
            lv_obj_align(label_tensao, LV_ALIGN_BOTTOM_MID,0,0);
            
            free(texto_display);    // Libera a memória alocada
        }
    }
}

//Task MQTT (Prática 8)
static void task_MQTT(){
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

// ************************** MAIN *******************************************************************************

void app_main(void){

    // ******************************** //
    // Logging library                  //
    // ESP_LOGE - error     (lowest)    //
    // ESP_LOGW - warning               //
    // ESP_LOGI - info                  //
    // ESP_LOGD - debug                 //
    // ESP_LOGV - verbose   (highest)   //
    // ******************************** //

    esp_log_level_set(SIS,ESP_LOG_WARN);                                // Logging sistema em nivel warning
    esp_log_level_set(TAG,ESP_LOG_INFO);                                // Logging sistema em nivel info
    esp_log_level_set(TIMER,ESP_LOG_INFO);                              // Logging sistema em nivel info
    esp_log_level_set(PWM,ESP_LOG_INFO);                                // Logging sistema em nivel info
    esp_log_level_set(ADC,ESP_LOG_INFO);                                // Logging sistema em nivel info
    esp_log_level_set(RX,ESP_LOG_INFO);                                 // Logging sistema em nivel info
    esp_log_level_set(TX,ESP_LOG_INFO);                                 // Logging sistema em nivel info
    esp_log_level_set(DISP,ESP_LOG_INFO);                               // Logging sistema em nivel info
    esp_log_level_set(MQTT,ESP_LOG_INFO);                               // Logging sistema em nivel info
    
    semaphore_pwm = xSemaphoreCreateBinary();                           // Criação do semaphore binario para o PWM
    semaphore_adc = xSemaphoreCreateBinary();                           // Criação do semaphore binario para o ADC

    // Criação das Filas
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));                // Cria a fila para o tratamento do GPIO por interrupção
    if (!gpio_evt_queue){                                               // Verifica se a fila do GPIO foi criada corretamente
        ESP_LOGE(SIS,"Criação da fila gpio_evt_queue falhou");
        return;
    }
    
    timer_int_queue = xQueueCreate(3, sizeof(estrutura_alarme_t));      // Cria a fila para o tratamento do timer por interrupção
    if (!timer_int_queue){                                              // Verifica se a fila do timer foi criada corretamente
        ESP_LOGE(SIS, "Criação da fila do timer falhou");
        return;
    }

    gpio_pwm_queue = xQueueCreate(10, sizeof(PWM_elements_t));          // Cria a fila para o tratamento do PWM segundo os botões
    if (!gpio_pwm_queue){                                               // Verifica se a fila do PWM foi criada corretamente
        ESP_LOGE(SIS, "Criação da fila gpio-pwm falhou");
        return;
    }

    timer_adc_queue = xQueueCreate(1, sizeof(ADC_elements_t));           // Cria a fila para o tratamento do ADC segundo o timer
    if (!timer_adc_queue){                                               // Verifica se a fila do ADC foi criada corretamente
        ESP_LOGE(SIS, "Criação da fila timer-adc falhou");
        return;
    }

    rtc_adj_queue = xQueueCreate(1, sizeof(estrutura_relogio_t));        // Cria a fila para ajustar o relógio segundo o uart
    if (!rtc_adj_queue){                                                 // Verifica se a fila do RTC foi criada corretamente
        ESP_LOGE(SIS, "Criação da fila rtc_adj falhou");
        return;
    }

    timer_display_queue = xQueueCreate(1, sizeof(estrutura_display_t));  // Cria a fila para atualizar o display segundo o relógio
    if (!rtc_adj_queue){                                                 // Verifica se a fila do display foi criada corretamente
        ESP_LOGE(SIS, "Criação da fila timer_display falhou");
        return;
    }

    // Criação das Tasks
    xTaskCreate(task_gpio, "task_gpio", 2048, NULL, 11, NULL);           // Cria task do GPIO
    xTaskCreate(task_timer,"task_timer", 4096, NULL, 10, NULL);          // Cria task do Timer
    xTaskCreate(task_PWM, "task_PWM", 2048, NULL, 10, NULL);             // Cria task do PWM
    xTaskCreate(task_ADC, "task_ADC", 2048, NULL, 10, NULL);             // Cria task do ADC
    xTaskCreate(task_uart, "task_uart", 2048, NULL, 10, NULL);           // Cria task do UART
    xTaskCreate(task_display, "task_display", 4096, NULL, 10, NULL);     // Cria task do I²C / Display
    xTaskCreate(task_MQTT, "task_MQTT", 4096, NULL, 10, NULL);           // Cria task do MQTT
    
    // Prática 1: Exibindo informações do módulo
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    ESP_LOGI(SIS,"This is %s chip with %d CPU core(s), WiFi%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI(SIS,"silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        ESP_LOGI(SIS,"Get flash size failed");
        return;
    }

    ESP_LOGI(SIS,"%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    ESP_LOGI(SIS,"Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    // ESP_LOGD(TAG, "passou pelo main A");
    // ESP_LOGD(TAG, "passou pelo main B");

}