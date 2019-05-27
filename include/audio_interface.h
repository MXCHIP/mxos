#pragma once

/* open audio device*/
void audio_init(void);

/* start playing */
int audio_dac_start(void);
/* stop playing */
int audio_dac_stop(void);
/* set play channel. default is mono mode
 * channel=1: mono mode.
 * channel=2: stereo mode.
 */
int audio_dac_channel_set(int channel);

/* set audio samplerate.
 * valid samplerate: 11025, 22050, 44100, 12000, 24000, 48000, 8000, 16000 and 32000.
 */
int audio_dac_samplerate_set(int samplerate);

/* set volume. from 0~100*/
int audio_dac_volume_set(int vol);

/* write PCM data to ring buffer */
int audio_dac_write(uint8_t *buf, int len);

/* start recording */
int audio_adc_start(void);

/* stop recording */
int audio_adc_stop(void);

/* set recorder samplerate.
 * valid samplerate: 11025, 22050, 44100, 12000, 24000, 48000, 8000, 16000 and 32000.
 */
int audio_adc_samplerate_set(int samplerate);

/* set recorder channel mode. this function must be called before audio_adc_start
 * channel=1: mono mode.
 * channel=2: stereo mode.
 */
int audio_adc_channel_set(int channel);

/* set the adc gain. valid gain is from 0 to 0x3F, default is 0x2D=0dBm */
int audio_adc_gain_set(int gain);

/* read recording PCM data to buf. */
int audio_adc_read(uint8_t *buf, int maxlen);
