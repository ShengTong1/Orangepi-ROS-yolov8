/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2023-2023. All rights reserved.
 * Description: Tensor Fusion Operation include file.
 * Author: MindX SDK
 * Create: 2023
 * History: NA
 */

#ifndef MXBASE_TENSORFUSION_H
#define MXBASE_TENSORFUSION_H

namespace MxBase {
    /**
     * @description: BackgroundReplace tensors, replace tensor and src tensor support UINT8, FLOAT16,
     * mask tensor support FLOAT16.
     * @param background: source tensor.
     * @param replace: replace tensor.
     * @param mask: mask tensor.
     * @param dst: out tensor.
     * @param stream: stream to operate op.
     */
    APP_ERROR BackgroundReplace(Tensor &background, const Tensor &replace, const Tensor &mask, Tensor &dst,
                                AscendStream &stream = AscendStream::DefaultStream());
                          
    /**
    * @description: Blend caption to backgound frame.
    * @param frame: Input and output tensor, the background frame to paste.
    * @param caption: Input tensor, the caption image.
    * @param captionAlpha: Input tensor, the opacity of caption imag, one channel.
    * @param captionBg: Input tensor, the background of caption.
    * @param captionBgOpacity: float [0, 1], the opacity of captionBg.
    * @param stream: stream to operate BlendImageCaption.
    */
    APP_ERROR BlendImageCaption(Tensor &frame, const Tensor &caption, const Tensor &captionAlpha,
                                const Tensor &captionBg,  float captionBgOpacity,
                                AscendStream &stream = AscendStream::DefaultStream());

    /**
    * @description: Blend images to material and frame.
    * @param material: Input tensor, the material image, support UINT8, RGBA format.
    * @param frame: Input and output tensor, the background image to blend, support UINT8, RGB format.
    * @param stream: stream to operate BlendImages.
    */
    APP_ERROR BlendImages(const Tensor &material, Tensor &frame, AscendStream& stream = AscendStream::DefaultStream());
}

#endif // MXBASE_TENSORFUSION_H
