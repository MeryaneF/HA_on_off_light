// Inclusão de bibliotecas

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "light_driver.h"  



// Definição das constantes e variaveis
#define BUTTON_GPIO 0       // Definição do pino GPIO do botão
#define LED_GPIO 2          // Definição o pino GPIO do LED da placa
#define NUM_LEDS 1          // Número de LEDs na strip 

static led_strip_handle_t s_led_strip;
static QueueHandle_t gpio_evt_queue = NULL;
static const char *TAG = "ESP_LIGHT_SWITCH";
static uint8_t s_red = 255, s_green = 255, s_blue = 255;

// interrupção do GPIO
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// Função para configurar o estado do LED
void light_driver_set_power(bool power)
{
    ESP_ERROR_CHECK(led_strip_set_pixel(s_led_strip, 0, s_red * power, s_green * power, s_blue * power));
    ESP_ERROR_CHECK(led_strip_refresh(s_led_strip));
    ESP_LOGI(TAG, "LED power set to %s", power ? "ON" : "OFF");
}

// iniciar o driver do LED
void light_driver_init(bool power)
{
    led_strip_config_t led_strip_conf = {
        .max_leds = NUM_LEDS,
        .strip_gpio_num = LED_GPIO,
    };
    led_strip_rmt_config_t rmt_conf = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&led_strip_conf, &rmt_conf, &s_led_strip));
    light_driver_set_power(power);
    ESP_LOGI(TAG, "LED strip initialized on GPIO %d", (int)LED_GPIO); 
}
// botão
static void button_task(void *arg)
{
    uint32_t io_num;
    bool led_on = false;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "GPIO[%d] intr, val: %d\n", (int)io_num, gpio_get_level(io_num)); 
            led_on = !led_on;
            light_driver_set_power(led_on);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


//função principal
void app_main(void)
{
    gpio_config_t io_conf;

    // Configuração do botão
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    // Criação da fila de eventos GPIO
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, gpio_isr_handler, (void *)BUTTON_GPIO);

    // Inicialização do driver do LED
    light_driver_init(false);

    // Criação da task do botão
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
}
