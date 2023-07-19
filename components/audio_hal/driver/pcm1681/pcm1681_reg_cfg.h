#ifndef _PCM1681_REG_CFG_
#define _PCM1681_REG_CFG_

#ifdef __cplusplus
extern "C"
{
#endif

#define DAMS_BIT_OFFSET (0x80)
#define DEFAULT_ATTENUATION (0xFF) // 0 dB

    typedef struct
    {
        uint8_t address;
        uint8_t data;
    } pcm1681_cfg_reg_t;

    static const pcm1681_cfg_reg_t pcm1681_reg_defaults[] = {
        {0x01, DEFAULT_ATTENUATION},
        {0x02, DEFAULT_ATTENUATION},
        {0x03, DEFAULT_ATTENUATION},
        {0x04, DEFAULT_ATTENUATION},
        {0x05, DEFAULT_ATTENUATION},
        {0x06, DEFAULT_ATTENUATION},
        {0x07, 0x00},
        {0x08, 0x00},
        {0x09, 0x04}, /* format config; 4: I2S stereo 16-bit to 24-bit */
        {0x0A, 0x00},
        {0x0B, 0xff},
        {0x0C, 0x0f},
        {0x0D, DAMS_BIT_OFFSET}, // DAMS = 1
        {0x10, DEFAULT_ATTENUATION},
        {0x11, DEFAULT_ATTENUATION},
        {0x12, 0x00},
        {0x13, 0x00},
    };

#ifdef __cplusplus
}
#endif

#endif // _PCM1681_REG_CFG_
