/*****************************************************************************
 * This file is part of uvgVPCCenc V-PCC encoder.
 *
 * Copyright (c) 2024-present, Tampere University, ITU/ISO/IEC, project contributors
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 * * Neither the name of the Tampere University or ITU/ISO/IEC nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * INCLUDING NEGLIGENCE OR OTHERWISE ARISING IN ANY WAY OUT OF THE USE OF THIS
 ****************************************************************************/

#pragma once

/// \file Interface between uvgVPCCenc and the 2D encoder Kvazaar that implement the 'abstract2DMapEncoder'.


extern "C" {
#include <libavcodec/avcodec.h>
}

#include "uvgvpcc/uvgvpcc.hpp"
#include "abstract2DMapEncoder.hpp"

#include <map>

using namespace uvgvpcc_enc;

class EncoderFFmpeg : public Abstract2DMapEncoder {
public:
    EncoderFFmpeg(
        const ENCODER_TYPE& encoderType, 
        const std::string&codecName, // ex: "libx265"
        const std::string& deviceType,
        const std::string& codecOptions, // Set general options, ex: "preset=fast:crf=26"
        const std::string& codecParams // Set codec specific params, ex: "x265-params=log-level=0:crf=26:qp=22"
    ) : Abstract2DMapEncoder(encoderType), codecName_(codecName), deviceType_(deviceType) {
        char delimiter = ':';
        size_t start = 0;
        size_t end = 0;

        // Check if the general options, that are specified by the codec, are separated by colons (:)
        if (codecOptions != "")
        {
            end = codecOptions.find(delimiter);
            while (end != std::string::npos) {
                std::string temp = codecOptions.substr(start, end - start);
                size_t end_temp = temp.find('=');
                std::string key = temp.substr(0, end_temp);
                std::string value = temp.substr(end_temp+1, temp.length() - end_temp);
                codecOptions_[key] = value;
                start = end + 1;
                end = codecOptions.find(delimiter, start);
            }
            std::string temp = codecOptions.substr(start);
            size_t end_temp = temp.find('=', 0);
            codecOptions_[temp.substr(0, end_temp)] = temp.substr(end_temp+1, temp.length() - end_temp);
        } 

        if (codecParams == "")
        {
            codecParamsEnabler_ = "";
            codecParamsValues_ = "";
            return;
        }

        // Get the param enabler for the codec, ex: "x265-params" or "kvazaar-params"
        start = 0;
        delimiter = '=';
        end = codecParams.find(delimiter);
        codecParamsEnabler_ = codecParams.substr(start, end - start);
        
        // Get the param values for the codec, ex: "log-level=0:crf=26:qp=22". Two possible separators: comma (,) or colon (:)
        /*
            Commas (,) separators are required by some specific codecs (e.g., libkvazaar); however, they conflict with the --uvgvpc setting
            that uses commas(,) to separate global parameters.
            
            Thus, to work around this issue, we use plus (+) as separators in the --uvgvpc setting and then convert them to commas (,).
            
            Example: preset=fast+threads=40+period=1+psnr=0+info=0+qp=22 --> preset=fast,threads=40,period=1,psnr=0,info=0,qp=22
        */
        if (codecParams.find('+', end+1) != std::string::npos)
        {
            delimiter = '+';
            start = end + 1;
            end = codecParams.find(delimiter, start);
            std::string params_temp = "";
            while (end != std::string::npos) {
                params_temp += codecParams.substr(start, end - start)+ ",";
                start = end + 1;
                end = codecParams.find(delimiter, start);
            }
            params_temp += codecParams.substr(start, codecParams.length() - start);
            //printf("params_temp: %s\n", params_temp.c_str());
            codecParamsValues_ = params_temp;
        } else { // Else the params are separated by colons (:) as other codecs
            codecParamsValues_ = codecParams.substr(end+1, codecParams.length() - end);
        }
    };
    static void initializeLogCallback();
    void encodeGOFMaps(const std::shared_ptr<uvgvpcc_enc::GOF>& gof) override;

private:
    const std::string codecName_;
    const std::string deviceType_;
    std::string codecParamsEnabler_;
    std::string codecParamsValues_;
    std::map<std::string, std::string> codecOptions_;
};


