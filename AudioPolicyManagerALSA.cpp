/*
 * Copyright (C) 2009 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "AudioPolicyManagerALSA"
//#define LOG_NDEBUG 0
#include <utils/Log.h>
#include "AudioPolicyManagerALSA.h"
#include <media/mediarecorder.h>

namespace android {

status_t AudioPolicyManagerALSA::setDeviceConnectionState(AudioSystem::audio_devices device,
                                                  AudioSystem::device_connection_state state,
                                                  const char *device_address)
{

    LOGV("setDeviceConnectionState() device: %x, state %d, address %s", device, state, device_address);

    // connect/disconnect only 1 device at a time
    if (AudioSystem::popCount(device) != 1) return BAD_VALUE;

    if (strlen(device_address) >= MAX_DEVICE_ADDRESS_LEN) {
        LOGE("setDeviceConnectionState() invalid address: %s", device_address);
        return BAD_VALUE;
    }

    // handle output devices
    if (AudioSystem::isOutputDevice(device)) {

#ifndef WITH_A2DP
        if (AudioSystem::isA2dpDevice(device)) {
            LOGE("setDeviceConnectionState() invalid device: %x", device);
            return BAD_VALUE;
        }
#endif

        switch (state)
        {
        // handle output device connection
        case AudioSystem::DEVICE_STATE_AVAILABLE:
            if (mAvailableOutputDevices & device) {
                LOGW("setDeviceConnectionState() device already connected: %x", device);
                return INVALID_OPERATION;
            }
            LOGV("setDeviceConnectionState() connecting device %x", device);

            // register new device as available
            mAvailableOutputDevices |= device;

#ifdef WITH_A2DP
            // handle A2DP device connection
            if (AudioSystem::isA2dpDevice(device)) {
                status_t status = handleA2dpConnection(device, device_address);
                if (status != NO_ERROR) {
                    mAvailableOutputDevices &= ~device;
                    return status;
                }
            } else
#endif
            {
                if (AudioSystem::isBluetoothScoDevice(device)) {
                    LOGV("setDeviceConnectionState() BT SCO  device, address %s", device_address);
                    // keep track of SCO device address
                    mScoDeviceAddress = String8(device_address, MAX_DEVICE_ADDRESS_LEN);
#ifdef WITH_A2DP
                    if (mA2dpOutput != 0 &&
                        mPhoneState != AudioSystem::MODE_NORMAL) {
                        mpClientInterface->suspendOutput(mA2dpOutput);
                    }
#endif
                }
            }
            break;
        // handle output device disconnection
        case AudioSystem::DEVICE_STATE_UNAVAILABLE: {
            if (!(mAvailableOutputDevices & device)) {
                LOGW("setDeviceConnectionState() device not connected: %x", device);
                return INVALID_OPERATION;
            }


            LOGV("setDeviceConnectionState() disconnecting device %x", device);
            // remove device from available output devices
            mAvailableOutputDevices &= ~device;

#ifdef WITH_A2DP
            // handle A2DP device disconnection
            if (AudioSystem::isA2dpDevice(device)) {
                status_t status = handleA2dpDisconnection(device, device_address);
                if (status != NO_ERROR) {
                    mAvailableOutputDevices |= device;
                    return status;
                }
            } else
#endif
            {
                if (AudioSystem::isBluetoothScoDevice(device)) {
                    mScoDeviceAddress = "";
#ifdef WITH_A2DP
                    if (mA2dpOutput != 0 &&
                        mPhoneState != AudioSystem::MODE_NORMAL) {
                        mpClientInterface->restoreOutput(mA2dpOutput);
                    }
#endif
                }
            }
            } break;

        default:
            LOGE("setDeviceConnectionState() invalid state: %x", state);
            return BAD_VALUE;
        }

        // request routing change if necessary
        uint32_t newDevice = getNewDevice(mHardwareOutput, false);
#ifdef WITH_A2DP
        checkOutputForAllStrategies(newDevice);
        // A2DP outputs must be closed after checkOutputForAllStrategies() is executed
        if (state == AudioSystem::DEVICE_STATE_UNAVAILABLE && AudioSystem::isA2dpDevice(device)) {
            closeA2dpOutputs();
        }
#endif
        updateDeviceForStrategy();
        setOutputDevice(mHardwareOutput, newDevice);

        if (device == AudioSystem::DEVICE_OUT_WIRED_HEADSET) {
            device = AudioSystem::DEVICE_IN_WIRED_HEADSET;
        } else if (device == AudioSystem::DEVICE_OUT_BLUETOOTH_SCO ||
                   device == AudioSystem::DEVICE_OUT_BLUETOOTH_SCO_HEADSET ||
                   device == AudioSystem::DEVICE_OUT_BLUETOOTH_SCO_CARKIT) {
            device = AudioSystem::DEVICE_IN_BLUETOOTH_SCO_HEADSET;
        } else {
            return NO_ERROR;
        }
    }
    // handle input devices
    if (AudioSystem::isInputDevice(device)) {

        switch (state)
        {
        // handle input device connection
        case AudioSystem::DEVICE_STATE_AVAILABLE: {
            if (mAvailableInputDevices & device) {
                LOGW("setDeviceConnectionState() device already connected: %d", device);
                return INVALID_OPERATION;
            }
            mAvailableInputDevices |= device;
            }
            break;

        // handle input device disconnection
        case AudioSystem::DEVICE_STATE_UNAVAILABLE: {
            if (!(mAvailableInputDevices & device)) {
                LOGW("setDeviceConnectionState() device not connected: %d", device);
                return INVALID_OPERATION;
            }
            mAvailableInputDevices &= ~device;
            } break;

        default:
            LOGE("setDeviceConnectionState() invalid state: %x", state);
            return BAD_VALUE;
        }

        audio_io_handle_t activeInput = getActiveInput();
        if (activeInput != 0) {
            AudioInputDescriptor *inputDesc = mInputs.valueFor(activeInput);
            uint32_t newDevice = getDeviceForInputSource(inputDesc->mInputSource);
            if (newDevice != inputDesc->mDevice) {
                LOGV("setDeviceConnectionState() changing device from %x to %x for input %d",
                        inputDesc->mDevice, newDevice, activeInput);
                inputDesc->mDevice = newDevice;
                AudioParameter param = AudioParameter();
                param.addInt(String8(AudioParameter::keyRouting), (int)newDevice);
                mpClientInterface->setParameters(activeInput, param.toString());
            }
        }
        else {
           if (device == AudioSystem::DEVICE_IN_FM_ANALOG) {
               routing_strategy strategy = getStrategy((AudioSystem::stream_type)3);
               uint32_t curOutdevice = getDeviceForStrategy(strategy);
               /* If A2DP headset is connected then route FM to Headset */
               if (curOutdevice == AudioSystem::DEVICE_OUT_BLUETOOTH_A2DP ||
                     curOutdevice == AudioSystem::DEVICE_OUT_BLUETOOTH_SCO) {
                  curOutdevice = AudioSystem::DEVICE_OUT_WIRED_HEADSET;
               }

               if (state) {
                   // routing_strategy strategy = getStrategy((AudioSystem::stream_type)3);
                   // uint32_t curOutdevice = getDeviceForStrategy(strategy);

                   /* Get the new input descriptor for FM Rx In */
                    mfmInput = getFMInput(AUDIO_SOURCE_FM_ANALOG,8000,1,
                           AudioSystem::CHANNEL_IN_MONO,(AudioSystem::audio_in_acoustics)7);

                   /* Forcely open the current output device again for
                    * FM Rx playback path to open
                    */
                    LOGV("curOutdevice = %x",curOutdevice);
                    setOutputDevice(mHardwareOutput, curOutdevice, true);

                   /* Tell the audio flinger playback thread that
                    * FM Rx is active
                    */
                    mpClientInterface->setFMRxActive(true);
               } else {
                    int newDevice=0;
                    AudioParameter param = AudioParameter();
                    param.addInt(String8(AudioParameter::keyRouting), (int)newDevice);

                   /* Change the input device from FM to default before releasing Input */
                    mpClientInterface->setParameters(mfmInput, param.toString());
                    param.addInt(String8("fm_off"), (int)newDevice);

                   /* Close the capture handle */
                    mpClientInterface->setParameters(mfmInput, param.toString());

                   /* Release the input descriptor for FM Rx In */
                    releaseInput(mfmInput);

                   /* Close the playback handle */
                    mpClientInterface->setParameters(mHardwareOutput, param.toString());

                   /* Tell the audio flinger playback thread that
                    * FM Rx is not active now.
                    */
                    mpClientInterface->setFMRxActive(false);
                }
             }
      }
        return NO_ERROR;
    }

    LOGW("setDeviceConnectionState() invalid device: %x", device);
    return BAD_VALUE;
}

audio_io_handle_t AudioPolicyManagerALSA::getFMInput(int inputSource,
                                    uint32_t samplingRate,
                                    uint32_t format,
                                    uint32_t channels,
                                    AudioSystem::audio_in_acoustics acoustics)
{
    audio_io_handle_t input = 0;
    uint32_t device = 0;

    if (inputSource == AUDIO_SOURCE_FM_ANALOG)
         device = AudioSystem::DEVICE_IN_FM_ANALOG;
    else {
         /* wrong input source */
         return 0;
    }

    LOGV("getFMInput() inputSource %d, samplingRate %d, format %d, channels %x, acoustics %x", inputSource, samplingRate, format, channels, acoustics);


    AudioInputDescriptor *inputDesc = new AudioInputDescriptor();

    inputDesc->mInputSource = inputSource;
    inputDesc->mDevice = device;
    inputDesc->mSamplingRate = samplingRate;
    inputDesc->mFormat = format;
    inputDesc->mChannels = channels;
    inputDesc->mAcoustics = acoustics;
    inputDesc->mRefCount = 0;
    input = mpClientInterface->openInput(&inputDesc->mDevice,
                                    &inputDesc->mSamplingRate,
                                    &inputDesc->mFormat,
                                    &inputDesc->mChannels,
                                    inputDesc->mAcoustics);

    // only accept input with the exact requested set of parameters
    if (input == 0 ||
        (samplingRate != inputDesc->mSamplingRate) ||
        (format != inputDesc->mFormat) ||
        (channels != inputDesc->mChannels)) {
        LOGV("getInput() failed opening input: samplingRate %d, format %d, channels %d",
                samplingRate, format, channels);
        if (input != 0) {
            mpClientInterface->closeInput(input);
        }
        delete inputDesc;
        return 0;
    }
    mInputs.add(input, inputDesc);
    return input;
}
// ----------------------------------------------------------------------------
// AudioPolicyManagerALSA
// ----------------------------------------------------------------------------

// ---  class factory

extern "C" AudioPolicyInterface* createAudioPolicyManager(AudioPolicyClientInterface *clientInterface)
{
    return new AudioPolicyManagerALSA(clientInterface);
}

extern "C" void destroyAudioPolicyManager(AudioPolicyInterface *interface)
{
    delete interface;
}

// Nothing currently different between the Base implementation.

AudioPolicyManagerALSA::AudioPolicyManagerALSA(AudioPolicyClientInterface *clientInterface)
    : AudioPolicyManagerBase(clientInterface)
{
}

AudioPolicyManagerALSA::~AudioPolicyManagerALSA()
{
}

}; // namespace android
