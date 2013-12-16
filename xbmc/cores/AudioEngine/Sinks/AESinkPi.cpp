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

#include <stdint.h>
#include <limits.h>

#include "AESinkPi.h"
#include "cores/AudioEngine/Utils/AEUtil.h"
#include "utils/log.h"
#include "settings/Settings.h"
#include "linux/RBP.h"

#define CLASSNAME "CAESinkPi"

#define NUM_OMX_BUFFERS 2
#define LAGGINESS (1.0/20.0)

CAEDeviceInfo CAESinkPi::m_info;

CAESinkPi::CAESinkPi() :
    m_sinkbuffer_size(0),
    m_sinkbuffer_sec_per_byte(0),
    m_Initialized(false),
    m_draining(false),
    m_submitted(0)
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
  format.m_frames        = format.m_sampleRate * LAGGINESS;
  format.m_frameSamples  = format.m_channelLayout.Count();
  format.m_frameSize     = format.m_frameSamples * (CAEUtil::DataFormatToBits(format.m_dataFormat) >> 3);
  format.m_sampleRate    = std::max(8000U, std::min(96000U, format.m_sampleRate));

  m_format = format;

  m_sinkbuffer_size = format.m_frameSize * format.m_frames * NUM_OMX_BUFFERS;
  m_sinkbuffer_sec_per_byte = 1.0 / (double)(format.m_frameSize * format.m_sampleRate);

  CLog::Log(LOGDEBUG, "%s:%s Format:%d Channels:%d Samplerate:%d framesize:%d bufsize:%d bytes/s=%.2f", CLASSNAME, __func__,
                format.m_dataFormat, format.m_channelLayout.Count(), format.m_sampleRate, format.m_frameSize, m_sinkbuffer_size, 1.0/m_sinkbuffer_sec_per_byte);
  return true;
}


void CAESinkPi::Initialize2()
{
  static bool first = true;
  if (first)
    g_RBP.WaitForOMXInit();
  first = false;

  CLog::Log(LOGDEBUG, "%s:%s", CLASSNAME, __func__);

  OMX_ERRORTYPE omx_err   = OMX_ErrorNone;

  if (!m_omx_render.Initialize("OMX.broadcom.audio_render", OMX_IndexParamAudioInit))
    CLog::Log(LOGERROR, "%s::%s - m_omx_render.Initialize omx_err(0x%08x)", CLASSNAME, __func__, omx_err);

  OMX_INIT_STRUCTURE(m_pcm_input);
  m_pcm_input.nPortIndex            = m_omx_render.GetInputPort();
  m_pcm_input.eNumData              = OMX_NumericalDataSigned;
  m_pcm_input.eEndian               = OMX_EndianLittle;
  m_pcm_input.bInterleaved          = OMX_TRUE;
  m_pcm_input.nBitPerSample         = 16;
  m_pcm_input.ePCMMode              = OMX_AUDIO_PCMModeLinear;
  m_pcm_input.nChannels             = m_format.m_frameSamples;
  m_pcm_input.nSamplingRate         = m_format.m_sampleRate;
  m_pcm_input.eChannelMapping[0] = OMX_AUDIO_ChannelLF;
  m_pcm_input.eChannelMapping[1] = OMX_AUDIO_ChannelRF;
  m_pcm_input.eChannelMapping[2] = OMX_AUDIO_ChannelMax;

  omx_err = m_omx_render.SetParameter(OMX_IndexParamAudioPcm, &m_pcm_input);
  if (omx_err != OMX_ErrorNone)
    CLog::Log(LOGERROR, "%s::%s - error m_omx_render SetParameter omx_err(0x%08x)", CLASSNAME, __func__, omx_err);

  m_omx_render.ResetEos();

  SetAudioDest();

  // set up the number/size of buffers for decoder input
  OMX_PARAM_PORTDEFINITIONTYPE port_param;
  OMX_INIT_STRUCTURE(port_param);
  port_param.nPortIndex = m_omx_render.GetInputPort();

  omx_err = m_omx_render.GetParameter(OMX_IndexParamPortDefinition, &port_param);
  if (omx_err != OMX_ErrorNone)
    CLog::Log(LOGERROR, "COMXAudio::Initialize error get OMX_IndexParamPortDefinition (input) omx_err(0x%08x)", omx_err);

  port_param.nBufferCountActual = std::max((unsigned int)port_param.nBufferCountMin, (unsigned int)NUM_OMX_BUFFERS);
  port_param.nBufferSize = m_sinkbuffer_size / port_param.nBufferCountActual;

  omx_err = m_omx_render.SetParameter(OMX_IndexParamPortDefinition, &port_param);
  if (omx_err != OMX_ErrorNone)
    CLog::Log(LOGERROR, "COMXAudio::Initialize error set OMX_IndexParamPortDefinition (intput) omx_err(0x%08x)", omx_err);

  omx_err = m_omx_render.AllocInputBuffers();
  if (omx_err != OMX_ErrorNone)
    CLog::Log(LOGERROR, "COMXAudio::Initialize - Error alloc buffers 0x%08x", omx_err);

  omx_err = m_omx_render.SetStateForComponent(OMX_StateExecuting);
  if (omx_err != OMX_ErrorNone)
    CLog::Log(LOGERROR, "%s:%s - m_omx_render OMX_StateExecuting omx_err(0x%08x)", CLASSNAME, __func__, omx_err);

  m_Initialized = true;
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

  if(omx_err != OMX_ErrorNone)
  {
    CLog::Log(LOGERROR, "%s::%s - error getting OMX_IndexConfigAudioRenderingLatency error 0x%08x\n",
      CLASSNAME, __func__, omx_err);
  }
  double sinkbuffer_seconds_to_empty = m_sinkbuffer_sec_per_byte * param.nU32 * m_format.m_frameSize;
  //CLog::Log(LOGDEBUG, "%s:%s %.2f (%d)\n", CLASSNAME, __func__, sinkbuffer_seconds_to_empty, param.nU32);
  return sinkbuffer_seconds_to_empty;
}

double CAESinkPi::GetCacheTime()
{
  return GetDelay();
}

double CAESinkPi::GetCacheTotal()
{
  double audioplus_buffer = 0; //512.0 * 32.0 / (double)m_format.m_sampleRate;
  return m_sinkbuffer_sec_per_byte * (double)m_sinkbuffer_size + audioplus_buffer;
}

unsigned int CAESinkPi::AddPackets(uint8_t *data, unsigned int frames, bool hasAudio, bool blocking)
{
  unsigned int sent = 0;

  // hack - need first initialisation to be delayed
  if (!m_Initialized)
    Initialize2();

  if (!m_Initialized)
    return frames;

  OMX_ERRORTYPE omx_err   = OMX_ErrorNone;
  OMX_BUFFERHEADERTYPE *omx_buffer = NULL;
  while(sent < frames)
  {
    int timeout = blocking ? 1000:0;

    // delay compared to maximum we'd like (to keep lag low)
    double delay = GetDelay();
    bool laggy = delay - LAGGINESS > 0.0;
    omx_buffer = laggy ? NULL:m_omx_render.GetInputBuffer(timeout);

    if (omx_buffer == NULL)
    {
      if (laggy)
      {
        //CLog::Log(LOGDEBUG, "%s:%s laggy %p,%d=%d (%d%d%d)", CLASSNAME, __func__, data, frames, sent, hasAudio, blocking, m_Initialized);
        Sleep((int)((delay - LAGGINESS) * 1000.0));
        continue;
      }
      if (blocking)
        CLog::Log(LOGERROR, "COMXAudio::Decode timeout");
      break;
    }

    omx_buffer->nFilledLen = std::min(omx_buffer->nAllocLen, (frames - sent) * m_format.m_frameSize);
    omx_buffer->nTimeStamp = ToOMXTime(0);
    omx_buffer->nFlags = 0;
    memcpy(omx_buffer->pBuffer, (uint8_t *)data + sent * m_format.m_frameSize, omx_buffer->nFilledLen);
    sent += omx_buffer->nFilledLen / m_format.m_frameSize;

    if (sent == frames)
      omx_buffer->nFlags |= OMX_BUFFERFLAG_ENDOFFRAME;

    if (delay <= 0.0 && m_submitted)
      CLog::Log(LOGERROR, "%s:%s Underrun (delay:%.2f frames:%d)", CLASSNAME, __func__, delay, frames);

    #ifdef DUMP_AUDIO
    FILE *fp = fopen("piaudio.dat", "ab");
    fwrite(omx_buffer->pBuffer, 1, omx_buffer->nFilledLen, fp);
    fclose(fp);
    #endif
    omx_err = m_omx_render.EmptyThisBuffer(omx_buffer);
    if (omx_err != OMX_ErrorNone)
      CLog::Log(LOGERROR, "%s:%s frames=%d err=%x", CLASSNAME, __func__, frames, omx_err);
    m_submitted += omx_buffer->nFilledLen;
  }
  //CLog::Log(LOGDEBUG, "%s:%s %p,%d=%d (%d%d%d)", CLASSNAME, __func__, data, frames, sent, hasAudio, blocking, m_Initialized);

  return sent;
}

void CAESinkPi::Drain()
{
  int delay = (int)(GetDelay() * 1000.0);
  if (delay)
    Sleep(delay);
  CLog::Log(LOGDEBUG, "%s:%s delay:%dms now:%dms (%d)", CLASSNAME, __func__, delay, (int)(GetDelay() * 1000.0), m_draining);
  m_draining = true;
}

void CAESinkPi::EnumerateDevicesEx(AEDeviceInfoList &list, bool force)
{
  m_info.m_channels.Reset();
  m_info.m_dataFormats.clear();
  m_info.m_sampleRates.clear();

  m_info.m_deviceType = AE_DEVTYPE_HDMI;
  m_info.m_deviceName = "HDMI";
  m_info.m_displayName = "HDMI";
  m_info.m_displayNameExtra = "";
  m_info.m_channels += AE_CH_FL;
  m_info.m_channels += AE_CH_FR;
  m_info.m_sampleRates.push_back(48000);
  m_info.m_dataFormats.push_back(AE_FMT_S16LE);

  list.push_back(m_info);

  m_info.m_channels.Reset();
  m_info.m_dataFormats.clear();
  m_info.m_sampleRates.clear();

  m_info.m_deviceType = AE_DEVTYPE_PCM;
  m_info.m_deviceName = "Analogue";
  m_info.m_displayName = "Analogue";
  m_info.m_displayNameExtra = "";
  m_info.m_channels += AE_CH_FL;
  m_info.m_channels += AE_CH_FR;
  m_info.m_sampleRates.push_back(48000);
  m_info.m_dataFormats.push_back(AE_FMT_S16LE);

  list.push_back(m_info);
}
