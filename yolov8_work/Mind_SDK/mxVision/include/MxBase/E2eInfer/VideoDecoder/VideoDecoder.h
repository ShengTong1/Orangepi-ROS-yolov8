/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
 * Description: Processing of the Video Decode Function.
 * Author: MindX SDK
 * Create: 2022
 * History: NA
 */

#ifndef MX_VIDEODECODER_H
#define MX_VIDEODECODER_H

#include "MxBase/E2eInfer/DataType.h"
#include "MxBase/E2eInfer/Image/Image.h"
#include "MxBase/ErrorCode/ErrorCode.h"

namespace MxBase {
typedef APP_ERROR (*VideoDecodeCallBack)(Image& decodedImage, uint32_t channelId, uint32_t frameId, void* userData);

struct VideoDecodeConfig {
    uint32_t width = 1920;
    uint32_t height = 1080;
    StreamFormat inputVideoFormat = StreamFormat::H264_MAIN_LEVEL;
    ImageFormat outputImageFormat = ImageFormat::YUV_SP_420;
    VideoDecodeCallBack callbackFunc = nullptr;
    uint32_t skipInterval = 0;
    uint32_t cscMatrix = 0;
};

class VideoDecoderDptr;

class VideoDecoder {
public:
    /**
     * @description: Construction function.
     * @param: vDecodeConfig: video decode config; deviceId
     */
    VideoDecoder(const VideoDecodeConfig& vDecodeConfig, const int32_t deviceId = 0, const uint32_t channelId = 0);
    /**
     * @description: Default construction function.
     */
    ~VideoDecoder();

    /**
     * @description: Decode process.
     * @param: data: input raw frame data, dataSize: size of frame data, channelId: Video Channel Index,
     *         frameId: Video Frame Index, userData: User defined object pointer as callback input,
     *         inputFrameImage: struct of frame data info.
     */
    APP_ERROR Decode(const std::shared_ptr<uint8_t> data, const uint32_t dataSize,
                     const uint32_t frameId, void* userData);
    APP_ERROR Decode(const std::shared_ptr<uint8_t> data, const uint32_t dataSize, const uint32_t frameId,
                     Image& preMallocData, void* userData);

    /**
     * @description: Flush the remaining flames in queue.
     */
    APP_ERROR Flush();

private:
    void SetImageWH(Image& inputImage, const Size size);

private:
    friend class VideoDecoderDptr;
    std::shared_ptr<MxBase::VideoDecoderDptr> videoDecoderDptr_;
};
}
#endif