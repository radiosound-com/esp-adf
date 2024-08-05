#ifndef _PCM1681_H_
#define _PCM1681_H_

#include "audio_hal.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initialize PCM1681 codec chip
     *
     * @param cfg configuration of PCM1681
     *
     * @return
     *     - ESP_OK
     *     - ESP_FAIL
     */
    esp_err_t pcm1681_init(audio_hal_codec_config_t* codec_cfg);

    /**
     * @brief Deinitialize PCM1681 codec chip
     *
     * @return
     *     - ESP_OK
     *     - ESP_FAIL
     */
    esp_err_t pcm1681_deinit(void);

    esp_err_t pcm1681_write_reg(uint8_t reg_add, uint8_t data);

    esp_err_t pcm1681_read_reg(uint8_t reg_add, uint8_t* p_data);

    esp_err_t pcm1681_ctrl(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state);
    esp_err_t pcm1681_config_iface(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t* iface);

    /**
     * @brief Set volume/attenuation
     *
     * @param volume:  voice volume (0~100)
     *
     * @return
     *     - ESP_OK
     *     - ESP_FAIL
     */
    esp_err_t pcm1681_set_volume(int vol);

    /**
     * @brief Get volume/attenuation
     *
     * @param[out] *volume:  voice volume (0~100)
     *
     * @return
     *     - ESP_OK
     *     - ESP_FAIL
     */
    esp_err_t pcm1681_get_volume(int* value);

    /**
     * @brief Set PCM1681 mute or not
     *
     * @param enable enable(1) or disable(0)
     *
     * @return
     *     - ESP_FAIL Parameter error
     *     - ESP_OK   Success
     */
    esp_err_t pcm1681_set_mute(bool enable);

#ifdef __cplusplus
}
#endif

#endif // _PCM1681_H_
