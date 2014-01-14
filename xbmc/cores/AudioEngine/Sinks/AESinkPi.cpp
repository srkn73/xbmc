/*
 *      Copyright (C) 2010-2013 Team XBMC
 *      http://xbmc.org
 *
 *  This Program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This Program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with XBMC; see the file COPYING.  If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 */

#include "system.h"

#if defined(TARGET_RASPBERRY_PI)

#include <stdint.h>
#include <limits.h>

#include "AESinkPi.h"
#include "cores/AudioEngine/Utils/AEUtil.h"
#include "utils/log.h"
#include "settings/Settings.h"
#include "linux/RBP.h"

#define CLASSNAME "CAESinkPi"

#define NUM_OMX_BUFFERS 2
#define AUDIO_PLAYBUFFER (0.08) // 80ms

// See CEA spec: Table 20, Audio InfoFrame data byte 4 for the ordering here
static const enum AEChannel CEAChannelMap[] = {
  AE_CH_FL, AE_CH_FR, AE_CH_FC, AE_CH_LFE, AE_CH_BL, AE_CH_BR, AE_CH_SL, AE_CH_SR
};

CAEDeviceInfo CAESinkPi::m_info;

static void BuildChannelMapOMX(enum OMX_AUDIO_CHANNELTYPE *channelMap, uint64_t layout)
{
  int index = 0;

  if (layout & AV_CH_FRONT_LEFT           ) channelMap[index++] = OMX_AUDIO_ChannelLF;
  if (layout & AV_CH_FRONT_RIGHT          ) channelMap[index++] = OMX_AUDIO_ChannelRF;
  if (layout & AV_CH_FRONT_CENTER         ) channelMap[index++] = OMX_AUDIO_ChannelCF;
  if (layout & AV_CH_LOW_FREQUENCY        ) channelMap[index++] = OMX_AUDIO_ChannelLFE;
  if (layout & AV_CH_BACK_LEFT            ) channelMap[index++] = OMX_AUDIO_ChannelLR;
  if (layout & AV_CH_BACK_RIGHT           ) channelMap[index++] = OMX_AUDIO_ChannelRR;
  if (layout & AV_CH_SIDE_LEFT            ) channelMap[index++] = OMX_AUDIO_ChannelLS;
  if (layout & AV_CH_SIDE_RIGHT           ) channelMap[index++] = OMX_AUDIO_ChannelRS;
  if (layout & AV_CH_BACK_CENTER          ) channelMap[index++] = OMX_AUDIO_ChannelCS;
  // following are not in openmax spec, but gpu does accept them
  if (layout & AV_CH_FRONT_LEFT_OF_CENTER ) channelMap[index++] = (enum OMX_AUDIO_CHANNELTYPE)10;
  if (layout & AV_CH_FRONT_RIGHT_OF_CENTER) channelMap[index++] = (enum OMX_AUDIO_CHANNELTYPE)11;
  if (layout & AV_CH_TOP_CENTER           ) channelMap[index++] = (enum OMX_AUDIO_CHANNELTYPE)12;
  if (layout & AV_CH_TOP_FRONT_LEFT       ) channelMap[index++] = (enum OMX_AUDIO_CHANNELTYPE)13;
  if (layout & AV_CH_TOP_FRONT_CENTER     ) channelMap[index++] = (enum OMX_AUDIO_CHANNELTYPE)14;
  if (layout & AV_CH_TOP_FRONT_RIGHT      ) channelMap[index++] = (enum OMX_AUDIO_CHANNELTYPE)15;
  if (layout & AV_CH_TOP_BACK_LEFT        ) channelMap[index++] = (enum OMX_AUDIO_CHANNELTYPE)16;
  if (layout & AV_CH_TOP_BACK_CENTER      ) channelMap[index++] = (enum OMX_AUDIO_CHANNELTYPE)17;
  if (layout & AV_CH_TOP_BACK_RIGHT       ) channelMap[index++] = (enum OMX_AUDIO_CHANNELTYPE)18;

  while (index<OMX_AUDIO_MAXCHANNELS)
    channelMap[index++] = OMX_AUDIO_ChannelNone;
}

CAESinkPi::CAESinkPi() :
    m_sinkbuffer_size(0),
    m_sinkbuffer_padded_size(0),
    m_sinkbuffer_sec_per_byte(0),
    m_Initialized(false),
    m_submitted(0),
    m_padded_pitch(0)
{
}

CAESinkPi::~CAESinkPi()
{
}

void CAESinkPi::SetAudioDest()
{
  OMX_ERRORTYPE omx_err   = OMX_ErrorNone;
  OMX_CONFIG_BRCMAUDIODESTINATIONTYPE audioDest;
  OMX_INIT_STRUCTURE(audioDest);
  if (CSettings::Get().GetString("audiooutput.audiodevice") == "Pi:Analogue")
    strncpy((char *)audioDest.sName, "local", strlen("local"));
  else
    strncpy((char *)audioDest.sName, "hdmi", strlen("hdmi"));
  omx_err = m_omx_render.SetConfig(OMX_IndexConfigBrcmAudioDestination, &audioDest);
  if (omx_err != OMX_ErrorNone)
    CLog::Log(LOGERROR, "%s::%s - m_omx_render.SetConfig omx_err(0x%08x)", CLASSNAME, __func__, omx_err);
}

bool CAESinkPi::Initialize(AEAudioFormat &format, std::string &device)
{
  m_initDevice = device;
  m_initFormat = format;
  // setup for a 50ms sink feed from SoftAE
  format.m_dataFormat    = AE_FMT_S16NE;
  format.m_frameSamples  = format.m_channelLayout.Count();
  int sample_size        = CAEUtil::DataFormatToBits(format.m_dataFormat) >> 3;
  format.m_frameSize     = format.m_frameSamples * sample_size;
  //format.m_sampleRate    = std::max(8000, std::min(CSettings::Get().GetInt("audiooutput.samplerate"), (int)format.m_sampleRate));

  format.m_frames        = format.m_sampleRate * AUDIO_PLAYBUFFER;

  #if 0 // not yet
  CAEChannelInfo layout;
  for (unsigned int i = 0; i < format.m_frameSamples; ++i)
    layout += CEAChannelMap[i];
  format.m_channelLayout = layout;
  #endif

  m_format = format;
  // round up to power of 2
  unsigned int padded_channels = m_format.m_frameSamples > 4 ? 8 : m_format.m_frameSamples > 2 ? 4 : m_format.m_frameSamples;
  m_padded_pitch = padded_channels * sample_size;
  m_sinkbuffer_size = m_format.m_frameSize * m_format.m_frames * NUM_OMX_BUFFERS;
  m_sinkbuffer_padded_size = m_padded_pitch * m_format.m_frames * NUM_OMX_BUFFERS;
  m_sinkbuffer_sec_per_byte = 1.0 / (double)(m_format.m_frameSize * m_format.m_sampleRate);

  CLog::Log(LOGDEBUG, "%s:%s Format:%d Channels:%d Samplerate:%d framesize:%d bufsize:%d bytes/s=%.2f", CLASSNAME, __func__,
                m_format.m_dataFormat, m_format.m_frameSamples, m_format.m_sampleRate, m_format.m_frameSize, m_sinkbuffer_size, 1.0/m_sinkbuffer_sec_per_byte);

  // This may be called before Application calls g_RBP.Initialise, so call it here too
  g_RBP.Initialize();

  CLog::Log(LOGDEBUG, "%s:%s", CLASSNAME, __func__);

  OMX_ERRORTYPE omx_err   = OMX_ErrorNone;

  if (!m_omx_render.Initialize("OMX.broadcom.audio_render", OMX_IndexParamAudioInit))
    CLog::Log(LOGERROR, "%s::%s - m_omx_render.Initialize omx_err(0x%08x)", CLASSNAME, __func__, omx_err);

  OMX_INIT_STRUCTURE(m_pcm_input);
  m_pcm_input.nPortIndex            = m_omx_render.GetInputPort();
  m_pcm_input.eNumData              = OMX_NumericalDataSigned;
  m_pcm_input.eEndian               = OMX_EndianLittle;
  m_pcm_input.bInterleaved          = OMX_TRUE;
  m_pcm_input.nBitPerSample         = sample_size * 8;
  m_pcm_input.ePCMMode              = OMX_AUDIO_PCMModeLinear;
  m_pcm_input.nChannels             = padded_channels;
  m_pcm_input.nSamplingRate         = m_format.m_sampleRate;

  uint64_t channelMap = (1<<m_format.m_frameSamples)-1; // TODO: correct channel map
  BuildChannelMapOMX(m_pcm_input.eChannelMapping, channelMap);

  omx_err = m_omx_render.SetParameter(OMX_IndexParamAudioPcm, &m_pcm_input);
  if (omx_err != OMX_ErrorNone)
    CLog::Log(LOGERROR, "%s::%s - error m_omx_render SetParameter in omx_err(0x%08x)", CLASSNAME, __func__, omx_err);

  m_omx_render.ResetEos();

  SetAudioDest();

  // set up the number/size of buffers for decoder input
  OMX_PARAM_PORTDEFINITIONTYPE port_param;
  OMX_INIT_STRUCTURE(port_param);
  port_param.nPortIndex = m_omx_render.GetInputPort();

  omx_err = m_omx_render.GetParameter(OMX_IndexParamPortDefinition, &port_param);
  if (omx_err != OMX_ErrorNone)
    CLog::Log(LOGERROR, "%s:%s - error get OMX_IndexParamPortDefinition (input) omx_err(0x%08x)", CLASSNAME, __func__, omx_err);

  port_param.nBufferCountActual = std::max((unsigned int)port_param.nBufferCountMin, (unsigned int)NUM_OMX_BUFFERS);
  port_param.nBufferSize = m_sinkbuffer_padded_size / port_param.nBufferCountActual;

  omx_err = m_omx_render.SetParameter(OMX_IndexParamPortDefinition, &port_param);
  if (omx_err != OMX_ErrorNone)
    CLog::Log(LOGERROR, "%s:%s - error set OMX_IndexParamPortDefinition (intput) omx_err(0x%08x)", CLASSNAME, __func__, omx_err);

  omx_err = m_omx_render.AllocInputBuffers();
  if (omx_err != OMX_ErrorNone)
    CLog::Log(LOGERROR, "%s:%s - Error alloc buffers 0x%08x", CLASSNAME, __func__, omx_err);

  omx_err = m_omx_render.SetStateForComponent(OMX_StateExecuting);
  if (omx_err != OMX_ErrorNone)
    CLog::Log(LOGERROR, "%s:%s - m_omx_render OMX_StateExecuting omx_err(0x%08x)", CLASSNAME, __func__, omx_err);

  m_Initialized = true;
  return true;
}


void CAESinkPi::Deinitialize()
{
  CLog::Log(LOGDEBUG, "%s:%s", CLASSNAME, __func__);
  if (m_Initialized)
  {
    m_omx_render.FlushAll();
    m_omx_render.Deinitialize();
    m_Initialized = false;
  }
}

bool CAESinkPi::IsCompatible(const AEAudioFormat &format, const std::string &device)
{
  bool compatible =
      /* compare against the requested format and the real format */
      (m_initFormat.m_sampleRate    == format.m_sampleRate    || m_format.m_sampleRate    == format.m_sampleRate   ) &&
      (m_initFormat.m_dataFormat    == format.m_dataFormat    || m_format.m_dataFormat    == format.m_dataFormat   ) &&
      (m_initFormat.m_channelLayout == format.m_channelLayout || m_format.m_channelLayout == format.m_channelLayout) &&
      (m_initDevice == device);
  CLog::Log(LOGDEBUG, "%s:%s Format:%d Channels:%d Samplerate:%d = %d", CLASSNAME, __func__, format.m_dataFormat, format.m_channelLayout.Count(), format.m_sampleRate, compatible);
  return compatible;
}

double CAESinkPi::GetDelay()
{
  OMX_PARAM_U32TYPE param;
  OMX_INIT_STRUCTURE(param);

  if (!m_Initialized)
    return 0.0;

  param.nPortIndex = m_omx_render.GetInputPort();

  OMX_ERRORTYPE omx_err = m_omx_render.GetConfig(OMX_IndexConfigAudioRenderingLatency, &param);

  if (omx_err != OMX_ErrorNone)
  {
    CLog::Log(LOGERROR, "%s::%s - error getting OMX_IndexConfigAudioRenderingLatency error 0x%08x",
      CLASSNAME, __func__, omx_err);
  }
  double sinkbuffer_seconds_to_empty = m_sinkbuffer_sec_per_byte * param.nU32 * m_format.m_frameSize;
  return sinkbuffer_seconds_to_empty;
}

double CAESinkPi::GetCacheTime()
{
  return GetDelay();
}

double CAESinkPi::GetCacheTotal()
{
  double audioplus_buffer = 0.0;//AUDIO_PLAYBUFFER;
  return m_sinkbuffer_sec_per_byte * (double)m_sinkbuffer_size + audioplus_buffer;
}

unsigned int CAESinkPi::AddPackets(uint8_t *data, unsigned int frames, bool hasAudio, bool blocking)
{
  unsigned int sent = 0;

  if (!m_Initialized)
    return frames;

  OMX_ERRORTYPE omx_err = OMX_ErrorNone;
  OMX_BUFFERHEADERTYPE *omx_buffer = NULL;
  while (sent < frames)
  {
    double delay = GetDelay();
    double ideal_submission_time = AUDIO_PLAYBUFFER - delay;
    // ideal amount of audio we'd like submit (to make delay match AUDIO_PLAYBUFFER)
    int timeout = blocking ? 1000 : 0;
#if 1
    int ideal_submission_samples = ideal_submission_time / (m_sinkbuffer_sec_per_byte * m_format.m_frameSize);
    // if we are almost full then sleep (to avoid repeatedly sending a few samples)
    bool too_laggy = ideal_submission_time < 0.25 * AUDIO_PLAYBUFFER;
    int sleeptime = (int)(AUDIO_PLAYBUFFER * 0.25 * 1000.0);
#else
    // delay compared to maximum we'd like (to keep lag low)
    unsigned int ideal_submission_samples = 1<<30;
    bool too_laggy = ideal_submission_time < 0.0;
    int sleeptime = (int)((-ideal_submission_time) * 1000.0);
#endif
    if (too_laggy)
    {
      if (blocking)
      {
        Sleep(sleeptime);
        continue;
      }
      break;
    }
    omx_buffer = m_omx_render.GetInputBuffer(timeout);
    if (omx_buffer == NULL)
    {
      if (blocking)
        CLog::Log(LOGERROR, "COMXAudio::Decode timeout");
      break;
    }
    unsigned int space = omx_buffer->nAllocLen / m_padded_pitch;
    unsigned int samples = std::min(std::min(space, (unsigned int)ideal_submission_samples), frames - sent);

    omx_buffer->nFilledLen = samples * m_padded_pitch;
    omx_buffer->nTimeStamp = ToOMXTime(0);
    omx_buffer->nFlags = 0;
    if (m_format.m_frameSize == m_padded_pitch)
    {
      memcpy(omx_buffer->pBuffer, (uint8_t *)data + sent * m_format.m_frameSize, omx_buffer->nFilledLen);
    }
    else
    {
      uint8_t *src = (uint8_t *)data + sent * m_format.m_frameSize;
      uint8_t *dst = (uint8_t *)omx_buffer->pBuffer;
      for (unsigned int i = 0; i < samples; i++)
      {
        memcpy(dst, src, m_format.m_frameSize);
        dst += m_padded_pitch;
        src += m_format.m_frameSize;
      }
    }
    sent += samples;

    if (sent == frames)
      omx_buffer->nFlags |= OMX_BUFFERFLAG_ENDOFFRAME;

    if (delay <= 0.0 && m_submitted)
      CLog::Log(LOGNOTICE, "%s:%s Underrun (delay:%.2f frames:%d)", CLASSNAME, __func__, delay, frames);

    omx_err = m_omx_render.EmptyThisBuffer(omx_buffer);
    if (omx_err != OMX_ErrorNone)
      CLog::Log(LOGERROR, "%s:%s frames=%d err=%x", CLASSNAME, __func__, frames, omx_err);
    m_submitted++;
  }

  return sent;
}

void CAESinkPi::Drain()
{
  int delay = (int)(GetDelay() * 1000.0);
  if (delay)
    Sleep(delay);
  CLog::Log(LOGDEBUG, "%s:%s delay:%dms now:%dms", CLASSNAME, __func__, delay, (int)(GetDelay() * 1000.0));
}

void CAESinkPi::EnumerateDevicesEx(AEDeviceInfoList &list, bool force)
{
  //m_info.m_channels.Reset();
  //m_info.m_dataFormats.clear();
  //m_info.m_sampleRates.clear();

  m_info.m_deviceType = AE_DEVTYPE_HDMI;
  m_info.m_deviceName = "HDMI";
  m_info.m_displayName = "HDMI";
  m_info.m_displayNameExtra = "";
  //m_info.m_channels += AE_CH_FL;
  //m_info.m_channels += AE_CH_FR;
  //m_info.m_sampleRates.push_back(48000);
  //m_info.m_dataFormats.push_back(AE_FMT_S16LE);

  list.push_back(m_info);

  //m_info.m_channels.Reset();
  //m_info.m_dataFormats.clear();
  //m_info.m_sampleRates.clear();

  m_info.m_deviceType = AE_DEVTYPE_PCM;
  m_info.m_deviceName = "Analogue";
  m_info.m_displayName = "Analogue";
  m_info.m_displayNameExtra = "";
  //m_info.m_channels += AE_CH_FL;
  //m_info.m_channels += AE_CH_FR;
  //m_info.m_sampleRates.push_back(48000);
  //m_info.m_dataFormats.push_back(AE_FMT_S16LE);

  list.push_back(m_info);
}

#endif
