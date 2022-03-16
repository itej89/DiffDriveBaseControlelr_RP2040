/**
 * RP2040 FreeRTOS Template - App #3
 *
 * @copyright 2022, Tony Smith (@smittytone)
 * @version   1.1.0
 * @licence   MIT
 *
 */
#include "main.h"

using std::string;
using std::vector;
using std::stringstream;


/*
 * GLOBALS
 */

// This is the inter-task queue
volatile QueueHandle_t queue = NULL;

// The 4-digit display
HT16K33_Segment display;

// The sensor
MCP9808 sensor;
volatile bool sensor_good;
volatile double read_temp = 0.0;
volatile bool do_clear = false;
volatile bool irq_hit = false;


/*
 * LED FUNCTIONS
 */

/**
 * @brief Configure the on-board LED.
 */
void setup_led() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    led_off();
}


/**
 * @brief Turn the on-board LED on.
 */
void led_on() {
    led_set();
}


/**
 * @brief Turn the on-board LED off.
 */
void led_off() {
    led_set(false);
}


/**
 * @brief Set the on-board LED's state.
 */
void led_set(bool state) {
    gpio_put(PICO_DEFAULT_LED_PIN, state);
}


/*
 * SETUP FUNCTIONS
 */

/**
 * @brief Umbrella hardware setup routine.
 */
void setup() {
    setup_i2c();
    setup_led();
    setup_gpio();
}


/**
 * @brief Set up I2C and the devices that use it.
 */
void setup_i2c() {
    // Initialise the I2C bus for the display and sensor
    I2C::setup();

    // Initialise the display
    display = HT16K33_Segment();
    display.init();

    // Initialise the sensor
    sensor = MCP9808();
    sensor_good = sensor.begin();
    if (!sensor_good) {
        printf("[ERROR] Sensor not configured\n");
    }
}


void setup_gpio() {
    gpio_init(SENSOR_ALERT_PIN);
    gpio_set_dir(SENSOR_ALERT_PIN, GPIO_IN);
    //gpio_pull_up(SENSOR_ALERT_PIN);

    // Configure the GPIO LED
    gpio_init(RED_LED_PIN);
    gpio_set_dir(RED_LED_PIN, GPIO_OUT);
    gpio_put(GRN_LED_PIN, false);

    // Configure the GREEN LED
    gpio_init(GRN_LED_PIN);
    gpio_set_dir(GRN_LED_PIN, GPIO_OUT);
    gpio_put(GRN_LED_PIN, false);
}


/**
 * @brief ISR for GPIO
 */
void gpio_cb(uint gpio, uint32_t events) {
    irq_hit = true;
    enable_irq(false);
}


void enable_irq(bool state) {
    gpio_set_irq_enabled_with_callback(SENSOR_ALERT_PIN, GPIO_IRQ_LEVEL_LOW, true, &gpio_cb);
}


/*
 * TASKS
 */

/**
 * @brief Repeatedly flash the Pico's built-in LED.
 */
void led_task_pico(void* unused_arg) {
    // Store the Pico LED state
    uint8_t pico_led_state = 0;

    int count = -1;
    bool state = true;
    TickType_t then = 0;

    // Log app info
    #ifdef DEBUG
    log_device_info();
    #endif
    
    // IRQ on the sensor pin
    if (sensor_good) enable_irq(true);
    
    // Start the task loop
    while (true) {
        // Turn Pico LED on an add the LED state
        // to the FreeRTOS xQUEUE
        TickType_t now = xTaskGetTickCount();
        if (now - then >= 500) {
            then = now;

            if (state) {
                log_debug("PICO LED FLASH");
                led_on();
                pico_led_state = GPIO_LED_OFF;
                xQueueSendToBack(queue, &pico_led_state, 0);
                display_int(++count);
            } else {
                // Turn Pico LED off an add the LED state
                // to the FreeRTOS xQUEUE
                led_off();
                pico_led_state = GPIO_LED_ON;
                xQueueSendToBack(queue, &pico_led_state, 0);
                display_tmp(read_temp);
                
                // Handle post IRQ
                if (read_temp < 25 && irq_hit) {
                    sensor.clear_alert(false);
                    irq_hit = false;
                    enable_irq(true);
                }
            }

            state = !state;
            if (count > 9998) count = 0;
        }

        // Yield -- uncomment the next line to enable,
        // See BLOG POST https://blog.smittytone.net/2022/03/04/further-fun-with-freertos-scheduling/
        //vTaskDelay(0);
    }
}


/**
 * @brief Repeatedly flash an LED connected to GPIO pin 20
 *        based on the value passed via the inter-task queue
 */
void led_task_gpio(void* unused_arg) {
    // This variable will take a copy of the value
    // added to the FreeRTOS xQueue
    uint8_t passed_value_buffer = GPIO_LED_OFF;

    while (true) {
        // Check for an item in the FreeRTOS xQueue
        if (xQueueReceive(queue, &passed_value_buffer, portMAX_DELAY) == pdPASS) {
            // Received a value so flash the GPIO LED accordingly
            // (NOT the sent value)
            if (passed_value_buffer) log_debug("GPIO LED FLASH");
            gpio_put(RED_LED_PIN, (passed_value_buffer == GPIO_LED_ON));
        }
        
        gpio_put(GRN_LED_PIN, irq_hit);
        
        // Yield -- uncomment the next line to enable,
        // See BLOG POST https://blog.smittytone.net/2022/03/04/further-fun-with-freertos-scheduling/
        //vTaskDelay(0);
    }
}


void sensor_read_task(void* unused_arg) {
    while (true) {
        // Just read the sensor and yield
        read_temp = sensor.read_temp();
        vTaskDelay(SENSOR_TASK_DELAY_TICKS);
    }
}


/**
 * @brief Display a four-digit decimal value on the 4-digit display.
 *
 * @param number: The value to show.
 */
void display_int(int number) {
    // Convert the temperature value (a float) to a string value
    // fixed to two decimal places
    if (number < 0 || number > 9999) number = 9999;
    const uint32_t bcd_val = Utils::bcd(number);

    display.clear();
    display.set_number((bcd_val >> 12) & 0x0F, 0, false);
    display.set_number((bcd_val >> 8)  & 0x0F, 1, false);
    display.set_number((bcd_val >> 4)  & 0x0F, 2, false);
    display.set_number(bcd_val         & 0x0F, 3, false);
    display.draw();
}


/**
 * @brief Display a three-digit temperature on the 4-digit display.
 *
 * @param value: The value to show.
 */
void display_tmp(double value) {
    // Convert the temperature value (a float) to a string value
    // fixed to two decimal places
    stringstream stream;
    stream << std::fixed << std::setprecision(2) << value;
    const string temp = stream.str();

    // Display the temperature on the LED
    uint32_t digit = 0;
    char previous_char = 0;
    char current_char = 0;
    for (uint32_t i = 0 ; (i < temp.length() || digit == 3) ; ++i) {
        current_char = temp[i];
        if (current_char == '.' && digit > 0) {
            display.set_alpha(previous_char, digit - 1, true);
        } else {
            display.set_alpha(current_char, digit);
            previous_char = current_char;
            ++digit;
        }
    }

    // Add a final 'c' and update the display
    display.set_alpha('c', 3).draw();
}


/**
 * @brief Generate and print a debug message from a supplied string.
 *
 * @param msg: The base message to which `[DEBUG]` will be prefixed.
 */
void log_debug(const char* msg) {
    uint msg_length = 9 + strlen(msg);
    char* sprintf_buffer = (char *)malloc(msg_length);
    sprintf(sprintf_buffer, "[DEBUG] %s\n", msg);
    printf("%s", sprintf_buffer);
    free(sprintf_buffer);
}


/**
 * @brief Show basic device info.
 */
void log_device_info(void) {
    printf("App: %s %s\nBuild: %i\n", APP_NAME, APP_VERSION, BUILD_NUM);
}


/*
 * RUNTIME START
 */

int main() {
    // DEBUG
    #ifdef DEBUG
    stdio_init_all();
    // Pause to allow the USB path to initialize
    sleep_ms(2000);
    #endif

    // Set up the hardware
    setup();
    display.set_brightness(1);

    // Set up two tasks
    BaseType_t pico_status = xTaskCreate(led_task_pico, "PICO_LED_TASK",  128, NULL, 1, NULL);
    BaseType_t gpio_status = xTaskCreate(led_task_gpio, "GPIO_LED_TASK",  128, NULL, 1, NULL);
    BaseType_t sens_status = xTaskCreate(sensor_read_task, "SENS_LED_TASK", 64, NULL, 1, NULL);

    // Set up the event queue
    queue = xQueueCreate(4, sizeof(uint8_t));

    // Start the FreeRTOS scheduler if any of the tasks are good
    if (pico_status == pdPASS || gpio_status == pdPASS || sens_status == pdPASS) {
        vTaskStartScheduler();
    } else {
        // Flash board LED 5 times
        uint8_t count = 5;
        while (count > 0) {
            led_on();
            sleep_ms(100);
            led_off();
            sleep_ms(100);
            count--;
        }
    }

    // We should never get here, but just in case...
    while(true) {
        // NOP
    };
}
