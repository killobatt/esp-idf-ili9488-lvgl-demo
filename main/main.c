#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "esp_lcd_ili9488.h"
#include "esp_lcd_touch_xpt2046.h"

#include "lvgl.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

// ESP-32(s3) supports SPI on any GPIO pins, but it's a slow virtual SPI (VSPI), up to 80Mhz
// Fast hardware SPI is only supported on IO_MUX PINS, from doc:
// Pin Name | GPIO Number (SPI2)
// CS0         10
// SCLK        12
// MISO        13
// MOSI        11
// QUADWP      14
// QUADHD      9

static const gpio_num_t CONFIG_TFT_CS = GPIO_NUM_10;
static const gpio_num_t CONFIG_TFT_DC = GPIO_NUM_8;
static const gpio_num_t CONFIG_TFT_SPI_MOSI = GPIO_NUM_11;
static const gpio_num_t CONFIG_TFT_SPI_CLOCK = GPIO_NUM_12;
static const gpio_num_t CONFIG_TFT_BACKLIGHT_PIN = GPIO_NUM_18;
static const gpio_num_t CONFIG_TFT_SPI_MISO = GPIO_NUM_13;
static const gpio_num_t CONFIG_TFT_RESET = GPIO_NUM_46;

static const gpio_num_t CONFIG_TOUCH_CS = GPIO_NUM_17;
// static const gpio_num_t CONFIG_TOUCH_IRQ = GPIO_NUM_16;

static const ledc_mode_t BACKLIGHT_LEDC_MODE = LEDC_LOW_SPEED_MODE;
static const ledc_channel_t BACKLIGHT_LEDC_CHANNEL = LEDC_CHANNEL_0;
static const ledc_timer_t BACKLIGHT_LEDC_TIMER = LEDC_TIMER_1;
static const ledc_timer_bit_t BACKLIGHT_LEDC_TIMER_RESOLUTION = LEDC_TIMER_10_BIT;
static const uint32_t BACKLIGHT_LEDC_FRQUENCY = 5000;

static const int DISPLAY_HORIZONTAL_PIXELS = 320;
static const int DISPLAY_VERTICAL_PIXELS = 480;
static const int DISPLAY_COMMAND_BITS = 8;
static const int DISPLAY_PARAMETER_BITS = 8;
static const unsigned int DISPLAY_REFRESH_HZ = 40000000;
static const int DISPLAY_SPI_QUEUE_LEN = 10;
static const int SPI_MAX_TRANSFER_SIZE = 32768;

static const size_t LV_BUFFER_SIZE = DISPLAY_HORIZONTAL_PIXELS * 100;
static const int LVGL_UPDATE_PERIOD_MS = 5;

#define CONFIG_DISPLAY_COLOR_MODE 0 // 0 for RGB, 1 for BGR
#define USE_DOUBLE_BUFFERING 1

#define WIFI_SCAN_MAX_AP 20

static SemaphoreHandle_t lvgl_mutex;

static esp_lcd_panel_io_handle_t lcd_io_handle = NULL;
static esp_lcd_panel_handle_t lcd_handle = NULL;
static esp_lcd_touch_handle_t touch_handle = NULL;
static esp_lcd_panel_io_handle_t touch_panel_io_handle = NULL;

// static lv_disp_draw_buf_t lv_disp_buf;
// static lv_disp_drv_t lv_disp_drv;

/* LVGL display and touch */
static lv_display_t *lv_display = NULL;
static lv_indev_t *lvgl_touch_indev = NULL;

static lv_color_t *lv_buf_1 = NULL;
static lv_color_t *lv_buf_2 = NULL;
static lv_obj_t *meter = NULL;
static lv_style_t style_screen;

static void set_angle(void * obj, int32_t v) {
    lv_arc_set_value(obj, v);
}

static bool notify_lvgl_flush_ready(
    esp_lcd_panel_io_handle_t panel_io,
    esp_lcd_panel_io_event_data_t *edata, 
    void *user_ctx
) {
    lv_disp_flush_ready(lv_display);
    return false;
}

static void lvgl_flush_cb(
    lv_display_t *disp, 
    const lv_area_t *area, 
    uint8_t * px_map
) {
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    esp_lcd_panel_draw_bitmap(lcd_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
}

static void IRAM_ATTR lvgl_tick_cb(void *param) {
	lv_tick_inc(LVGL_UPDATE_PERIOD_MS);
}

static void lvgl_input_read(lv_indev_t *indev, lv_indev_data_t *data) {
    uint16_t x[1];
    uint16_t y[1];
    uint16_t strength[1];
    uint8_t count = 0;

    // Update touch point data.
    ESP_ERROR_CHECK(esp_lcd_touch_read_data(touch_handle));

    data->state = LV_INDEV_STATE_REL;

    if (esp_lcd_touch_get_coordinates(touch_handle, x, y, strength, &count, 1)) {
        data->point.x = x[0];
        data->point.y = y[0];
        data->state = LV_INDEV_STATE_PR;
        ESP_LOGI(TAG, "Touch: %d %d", x[0], y[0]);
    }

    data->continue_reading = false;
}

void display_brightness_init(void) {
    const ledc_channel_config_t lcd_backlight_channel = {
        .gpio_num = (gpio_num_t)CONFIG_TFT_BACKLIGHT_PIN,
        .speed_mode = BACKLIGHT_LEDC_MODE,
        .channel = BACKLIGHT_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BACKLIGHT_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags = 
        {
            .output_invert = 0
        }
    };
    const ledc_timer_config_t lcd_backlight_timer = {
        .speed_mode = BACKLIGHT_LEDC_MODE,
        .duty_resolution = BACKLIGHT_LEDC_TIMER_RESOLUTION,
        .timer_num = BACKLIGHT_LEDC_TIMER,
        .freq_hz = BACKLIGHT_LEDC_FRQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_LOGI(TAG, "Initializing LEDC for backlight pin: %d", CONFIG_TFT_BACKLIGHT_PIN);

    ESP_ERROR_CHECK(ledc_timer_config(&lcd_backlight_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&lcd_backlight_channel));
}

void display_brightness_set(int brightness_percentage) {
    if (brightness_percentage > 100) {
        brightness_percentage = 100;
    } else if (brightness_percentage < 0) {
        brightness_percentage = 0;
    }
    ESP_LOGI(TAG, "Setting backlight to %d%%", brightness_percentage);

    // LEDC resolution set to 10bits, thus: 100% = 1023
    uint32_t duty_cycle = (1023 * brightness_percentage) / 100;
    ESP_ERROR_CHECK(ledc_set_duty(BACKLIGHT_LEDC_MODE, BACKLIGHT_LEDC_CHANNEL, duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(BACKLIGHT_LEDC_MODE, BACKLIGHT_LEDC_CHANNEL));
}

void initialize_spi() {
    ESP_LOGI(TAG, "Initializing SPI bus (MOSI:%d, MISO:%d, CLK:%d)",
             CONFIG_TFT_SPI_MOSI, CONFIG_TFT_SPI_MISO, CONFIG_TFT_SPI_CLOCK);
    spi_bus_config_t bus = {
        .mosi_io_num = CONFIG_TFT_SPI_MOSI,
        .miso_io_num = CONFIG_TFT_SPI_MISO,
        .sclk_io_num = CONFIG_TFT_SPI_CLOCK,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .data4_io_num = GPIO_NUM_NC,
        .data5_io_num = GPIO_NUM_NC,
        .data6_io_num = GPIO_NUM_NC,
        .data7_io_num = GPIO_NUM_NC,
        .max_transfer_sz = SPI_MAX_TRANSFER_SIZE,
        .flags = SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO |
                 SPICOMMON_BUSFLAG_MOSI | SPICOMMON_BUSFLAG_MASTER,
        .intr_flags = ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
}

void initialize_display() {
    const esp_lcd_panel_io_spi_config_t io_config =  {
        .cs_gpio_num = CONFIG_TFT_CS,
        .dc_gpio_num = CONFIG_TFT_DC,
        .spi_mode = 0,
        .pclk_hz = DISPLAY_REFRESH_HZ,
        .trans_queue_depth = DISPLAY_SPI_QUEUE_LEN,
        .on_color_trans_done = notify_lvgl_flush_ready,
        .user_ctx = &lv_display,
        .lcd_cmd_bits = DISPLAY_COMMAND_BITS,
        .lcd_param_bits = DISPLAY_PARAMETER_BITS,
        .flags = {
            .dc_low_on_data = 0,
            .octal_mode = 0,
            .sio_mode = 0,
            .lsb_first = 0,
            .cs_high_active = 0
        }
    };

    const esp_lcd_panel_dev_config_t lcd_config =  {
        .reset_gpio_num = CONFIG_TFT_RESET,
        .color_space = CONFIG_DISPLAY_COLOR_MODE,
        .bits_per_pixel = 18, // TODO: try 16 since lvgl supports 16 only? 
        .flags = {
            .reset_active_high = 0
        },
        .vendor_config = NULL
    };

    ESP_ERROR_CHECK(
        esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &lcd_io_handle)); 

    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9488(lcd_io_handle, &lcd_config, LV_BUFFER_SIZE, &lcd_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(lcd_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(lcd_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_handle, true, true));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(lcd_handle, 0, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_handle, true));
}

void initialize_touch() {
    esp_lcd_panel_io_spi_config_t tp_io_config = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(CONFIG_TOUCH_CS);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &tp_io_config, &touch_panel_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = DISPLAY_HORIZONTAL_PIXELS,
        .y_max = DISPLAY_VERTICAL_PIXELS,
        .rst_gpio_num = -1,
        .int_gpio_num = -1, // CONFIG_TOUCH_IRQ,
        .flags = {
            .swap_xy = 0,
            .mirror_x = true,
            .mirror_y = false,
        },
    };

    ESP_LOGI(TAG, "Initialize touch controller XPT2046");
    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(touch_panel_io_handle, &tp_cfg, &touch_handle));
}

void initialize_lvgl() {
    ESP_LOGI(TAG, "Initializing LVGL");
    lv_init();

    ESP_LOGI(TAG, "Allocating %zu bytes for LVGL buffer", LV_BUFFER_SIZE * sizeof(lv_color_t));
    lv_buf_1 = (lv_color_t *)heap_caps_malloc(LV_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
#if USE_DOUBLE_BUFFERING
    ESP_LOGI(TAG, "Allocating %zu bytes for second LVGL buffer", LV_BUFFER_SIZE * sizeof(lv_color_t));
    lv_buf_2 = (lv_color_t *)heap_caps_malloc(LV_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
#endif

    lv_display = lv_display_create(DISPLAY_HORIZONTAL_PIXELS, DISPLAY_VERTICAL_PIXELS);
    lv_display_set_flush_cb(lv_display, lvgl_flush_cb);
    lv_display_set_color_format(lv_display, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(lv_display, lv_buf_1, lv_buf_2, LV_BUFFER_SIZE * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);

    ESP_LOGI(TAG, "Creating LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvgl_tick_cb,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_UPDATE_PERIOD_MS * 1000));

    /*Register at least one display before you register any input devices*/
    lvgl_touch_indev = lv_indev_create();
    lv_indev_set_type(lvgl_touch_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(lvgl_touch_indev, lvgl_input_read);
}

void create_demo_ui() {
    lv_obj_t *scr = lv_disp_get_scr_act(NULL);

    // Set the background color of the display to black.
    lv_style_init(&style_screen);
    lv_style_set_bg_color(&style_screen, lv_color_black());
    lv_style_set_text_color(&style_screen, lv_color_hex(0xFFFFFF));
    lv_obj_add_style(lv_scr_act(), &style_screen, LV_STATE_DEFAULT);

        /*Create an Arc*/
    lv_obj_t * arc = lv_arc_create(lv_screen_active());
    lv_arc_set_rotation(arc, 270);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);   /*Be sure the knob is not displayed*/
    lv_obj_remove_flag(arc, LV_OBJ_FLAG_CLICKABLE);  /*To not allow adjusting by click*/
    lv_obj_center(arc);

    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text(label, "Scanning WiFi...");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 100);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, arc);
    lv_anim_set_exec_cb(&a, set_angle);
    lv_anim_set_duration(&a, 1000);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);    /*Just for the demo*/
    lv_anim_set_repeat_delay(&a, 500);
    lv_anim_set_values(&a, 0, 100);
    lv_anim_start(&a);
}

static void lvgl_ui_task(void *params) {
    if (pdTRUE == xSemaphoreTake(lvgl_mutex, 1000)) {
        display_brightness_init();
        display_brightness_set(0);

        initialize_spi();
        initialize_display();

        initialize_touch();

        initialize_lvgl();
        create_demo_ui();
        display_brightness_set(100);

        xSemaphoreGive(lvgl_mutex);

        while (true) {
            vTaskDelay(pdMS_TO_TICKS(10));
            if (pdTRUE == xSemaphoreTake(lvgl_mutex, 100)) {
                lv_timer_handler();
                xSemaphoreGive(lvgl_mutex);
            } else {
                ESP_LOGE(TAG, "Could not take lvgl_mutex for timer");
            }
        }
        
    } else {
        ESP_LOGE(TAG, "Could not take lvgl_mutex");
    }
}

typedef struct {
    wifi_ap_record_t *wifi_records;
    uint16_t wifi_records_count;
} wifi_scan_result_t;

void create_wifi_ap_list_screen() {
    // if (!xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(1000))) {
    //     ESP_LOGE(TAG, "Timed out taking semaphore for initial UI creation");
    //     return;
    // }

    // Fake it till you make it
    const uint8_t wifi_records_count = 8;
    wifi_ap_record_t wifi_records[wifi_records_count];
    memset(wifi_records, 0x00, sizeof(wifi_ap_record_t) * wifi_records_count);
    for (int i = 0; i < wifi_records_count; i++) {
        strcpy((char *)wifi_records[i].ssid, "WiFi Point");
        wifi_records[i].rssi = -55 - i;
    }

    // end faking

    lv_obj_t *scr = lv_obj_create(NULL);

    lv_obj_t *list = lv_list_create(scr);
    lv_obj_align(list, LV_ALIGN_TOP_LEFT, 0, 0);

    for (uint16_t record_index = 0; record_index < wifi_records_count; record_index++) {
        wifi_ap_record_t record = wifi_records[record_index];

        char ssid[33];
        sprintf(ssid, "%32s", (char *)record.ssid);
        lv_list_add_button(list, LV_SYMBOL_WIFI, ssid);
    }

    lv_screen_load_anim(scr, LV_SCR_LOAD_ANIM_OVER_BOTTOM, 1000, 0, true);

    // xSemaphoreGive(lvgl_mutex);
}

esp_err_t scan_wifi_networks(uint16_t *number, wifi_ap_record_t *ap_records) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    ESP_LOGI(TAG, "WiFi init");
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&wifi_init_config), TAG, "WiFi init");

    ESP_LOGI(TAG, "WiFi Set Mode Station");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "WiFi Set Mode Station");

    ESP_LOGI(TAG, "WiFi start");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "WiFi start");

    ESP_LOGI(TAG, "WiFi Scan start");
    ESP_RETURN_ON_ERROR(esp_wifi_scan_start(NULL, true), TAG, "WiFi Scan start");

    ESP_LOGI(TAG, "esp_wifi_scan_get_ap_records");
    ESP_RETURN_ON_ERROR(esp_wifi_scan_get_ap_records(number, ap_records), TAG, "Scan error");

    return ESP_OK;
}

void wifi_scan_task(void *params) {
    // Emulating process until I find out why ESP32S3 does not go beyond ESP_WIFI_START();
    vTaskDelay(pdMS_TO_TICKS(4000));

    lv_async_call(create_wifi_ap_list_screen, NULL);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    // End emulating

    wifi_ap_record_t wifi_records[WIFI_SCAN_MAX_AP];
    uint16_t max_record = WIFI_SCAN_MAX_AP;
    ESP_ERROR_CHECK(scan_wifi_networks(&max_record, wifi_records));

    for (int i = 0; i < max_record; i++) {
        ESP_LOGI(TAG, "SSID \t\t%s", wifi_records[i].ssid);
        ESP_LOGI(TAG, "RSSI \t\t%d", wifi_records[i].rssi);
        ESP_LOGI(TAG, "Channel \t\t%d", wifi_records[i].primary);
    }

    create_wifi_ap_list_screen(wifi_records, max_record);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void app_main(void) {
    lvgl_mutex = xSemaphoreCreateMutex();

    xTaskCreate(lvgl_ui_task, "LVGL UI", 50 * 1024, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(wifi_scan_task, "WiFi Scan", 2 * 1024, NULL, tskIDLE_PRIORITY, NULL);
}
