/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2020 <ESPRESSIF SYSTEMS (SHANGHAI) CO., LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "modified_codec.h"

#include "pcm1681.h"
#include "esp_log.h"

audio_hal_func_t AUDIO_MODIFIED_PCM1681_CODEC_DEFAULT_HANDLE = {
    .audio_codec_initialize = pcm1681_init,
    .audio_codec_deinitialize = pcm1681_deinit,
    .audio_codec_ctrl = pcm1681_ctrl,
    .audio_codec_config_iface = pcm1681_config_iface,
    .audio_codec_set_mute = pcm1681_set_mute,
    .audio_codec_set_volume = modified_codec_set_volume, // Override PCM1681's "set volume" function
    .audio_codec_get_volume = pcm1681_get_volume,
};

esp_err_t modified_codec_set_volume(int volume)
{
    ESP_LOGI("MOD", "Set volume to %d", volume);
    return ESP_OK;
}

