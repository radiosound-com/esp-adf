#include "pcm1681.h"

#include "board.h"
#include "esp_log.h"
#include "i2c_bus.h"
#include "pcm1681_reg_cfg.h"

static const char* TAG = "PCM1681";

#define PCM1681_ADDR (0x98) // 0x9A if the ADR pin is tied high

#define PCM1681_SOFT_MUTE_ALL (0xff)
#define PCM1681_DEEMPH_RATE_MASK (0x18)
#define PCM1681_DEEMPH_MASK (0x01)

#define PCM1681_MIN_VOLUME (0)
#define PCM1681_MAX_VOLUME (100)

// Register Addresses
#define PCM1681_FLT_FMT (0x04) /* Filter rolloff control & Data format */
#define PCM1681_ATT_CONTROL(X) (X <= 6 ? X : X + 9) /* Attenuation level */
#define PCM1681_SOFT_MUTE (0x07) /* Soft mute control register */
#define PCM1681_DAC_CONTROL (0x08) /* DAC operation control */
#define PCM1681_FMT_CONTROL (0x09) /* Audio interface data format */
#define PCM1681_DEEMPH_CONTROL (0x0a) /* De-emphasis control */
#define PCM1681_ZERO_DETECT_STATUS (0x0e) /* Zero detect status reg */

#define PCM1681_ASSERT(a, format, b, ...)                                                                              \
    if ((a) != 0)                                                                                                      \
    {                                                                                                                  \
        ESP_LOGE(TAG, format, ##__VA_ARGS__);                                                                          \
        return b;                                                                                                      \
    }

static i2c_bus_handle_t i2c_handle;

/*
 * Operate fuction of PA
 */
audio_hal_func_t AUDIO_CODEC_PCM1681_DEFAULT_HANDLE = {
    .audio_codec_initialize = pcm1681_init,
    .audio_codec_deinitialize = pcm1681_deinit,
    .audio_codec_ctrl = pcm1681_ctrl,
    .audio_codec_config_iface = pcm1681_config_iface,
    .audio_codec_set_mute = pcm1681_set_mute,
    .audio_codec_set_volume = pcm1681_set_volume,
    .audio_codec_get_volume = pcm1681_get_volume,
    .audio_hal_lock = NULL,
    .handle = NULL,
};

esp_err_t pcm1681_write_reg(uint8_t reg_add, uint8_t data)
{
    int retries = 0;
    esp_err_t result;
    do
    {
        result = i2c_bus_write_bytes(i2c_handle, PCM1681_ADDR, &reg_add, sizeof(reg_add), &data, sizeof(data));
        if (result != ESP_OK)
        {
            // sleep a bit and wait for it to wake up
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    } while (result != ESP_OK && ++retries < 500);
    return result;
}

esp_err_t pcm1681_read_reg(uint8_t reg_add, uint8_t* p_data)
{
    return i2c_bus_read_bytes(i2c_handle, PCM1681_ADDR, &reg_add, sizeof(reg_add), p_data, 1);
}

static esp_err_t i2c_init()
{
    esp_err_t result;
    i2c_config_t pcm1681_i2c_cfg = {.mode = I2C_MODE_MASTER,
                                    .sda_pullup_en = GPIO_PULLUP_ENABLE,
                                    .scl_pullup_en = GPIO_PULLUP_ENABLE,
                                    .master.clk_speed = 100000};
    result = get_i2c_pins(I2C_NUM_0, &pcm1681_i2c_cfg);
    PCM1681_ASSERT(result, "getting i2c pins error", -1);
    i2c_handle = i2c_bus_create(I2C_NUM_0, &pcm1681_i2c_cfg);
    if (!i2c_handle)
    {
        ESP_LOGE(TAG, "no i2c handle?");
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    while (!i2c_handle)
    {
        i2c_handle = i2c_bus_create(I2C_NUM_0, &pcm1681_i2c_cfg);
        if (!i2c_handle)
        {
            ESP_LOGE(TAG, "no i2c handle?");
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    return result;
}

esp_err_t i2c_deinit()
{
    return i2c_bus_delete(i2c_handle);
}

esp_err_t pcm1681_init(audio_hal_codec_config_t* codec_cfg)
{
    esp_err_t result;
    result = i2c_init();

    // Initialize the pcm1681's registers with the values in pcm1681_reg_defaults[]
    int i = 0;
    while ((i < (sizeof(pcm1681_reg_defaults) / sizeof(pcm1681_reg_defaults[0]))) && result == ESP_OK)
    {
        result = pcm1681_write_reg(pcm1681_reg_defaults[i].address, pcm1681_reg_defaults[i].data);
        i++;
    }
    return result;
}

esp_err_t pcm1681_deinit(void)
{
    return i2c_deinit();
}

esp_err_t pcm1681_set_mute(bool enable)
{
    uint8_t register_value;
    // If enable is true, mute all DAC channels
    if (enable)
    {
        register_value = PCM1681_SOFT_MUTE_ALL;
    }
    else
    {
        register_value = 0x00;
    }

    return pcm1681_write_reg(PCM1681_SOFT_MUTE, register_value);
}

esp_err_t pcm1681_ctrl(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state)
{
    // TODO
    return ESP_OK;
}

#define PCM1681_FMT_RJ_24 (0)
#define PCM1681_FMT_RJ_16 (3)
#define PCM1681_FMT_I2S_16_24 (4)
#define PCM1681_FMT_LJ_16_24 (5)
#define PCM1681_FMT_I2S_TDM_24 (6)
#define PCM1681_FMT_LJ_TDM_24 (7)
#define PCM1681_FMT_I2S_DSP_24 (8)
#define PCM1681_FMT_LJ_DSP_24 (9)

esp_err_t pcm1681_config_iface(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t* iface)
{
    // TODO
    return ESP_OK;
}

esp_err_t pcm1681_set_volume(int volume)
{
    esp_err_t result;

    // Don't allow volumes outside the MIN/MAX bounds
    if (volume < PCM1681_MIN_VOLUME)
    {
        volume = PCM1681_MIN_VOLUME;
    }
    if (volume > PCM1681_MAX_VOLUME)
    {
        volume = PCM1681_MAX_VOLUME;
    }

    // Convert the volume level to the corresponding register value
    // NOTE: This function assumes DAMS = 1,
    // therefore attenuation is between 0 and -100
    // and the dB step is 1
    // volume 100 = 255 = 0 dB (no attenuation)
    // volume 0 = 155 = -100 dB
    uint8_t channel_attenuation = 155 + volume;

    // Set each of the 8 DAC channels to the same attenuation
    for (uint8_t i = 1; i <= 8; i++)
    {
        result = pcm1681_write_reg((uint8_t)PCM1681_ATT_CONTROL(i), channel_attenuation);
        if (result != ESP_OK)
        {
            break;
        }
    }
    return result;
}

esp_err_t pcm1681_get_volume(int* value)
{
    esp_err_t result;

    // We're writing the same value to each attenuation register,
    // so let's read one and assume the others are the same
    uint8_t register_volume = 0;
    result = pcm1681_read_reg(PCM1681_ATT_CONTROL(1), &register_volume);
    *value = register_volume - 155;

    return result;
}
