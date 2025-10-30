import numpy as np
import cv2
from mindx.sdk import Tensor


# 前处理
def resize_image(image, size, letterbox_image):
    """
    对输入图像进行resize
    Args:
        size: 目标尺寸
        letterbox_image: bool 是否进行letterbox变换
    Returns: 指定尺寸的图像和变换信息
    """
    ih, iw, _ = image.shape
    h, w = size
    if letterbox_image:
        scale = min(w/iw, h/ih)
        nw = int(iw*scale)
        nh = int(ih*scale)
        image = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)
        image_back = np.ones((h, w, 3), dtype=np.uint8) * 128
        dh = (h-nh)//2
        dw = (w-nw)//2
        image_back[dh: dh + nh, dw: dw + nw, :] = image
        return image_back, scale, dh, dw
    else:
        return image, 1.0, 0, 0


def img2input(img):
    img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, HWC to CHW
    img = np.expand_dims(img, 0).astype(np.float32)  # 得到(1, 3, 640, 640)
    img = np.ascontiguousarray(img) / 255.0  # 归一化
    img = Tensor(img)  # 将numpy转为Tensor类
    return img


def std_output(pred):
    """
    将(1, 13, 8400)处理成(8400, 13);  13 = box:4 (x,y,w,h) + cls:9
    注意：YOLOv8的输出顺序可能是不同的，需要根据实际调整
    """
    pred = np.squeeze(pred)
    if len(pred.shape) == 2:
        # 已经是 (13, 8400)，转置为 (8400, 13)
        pred = pred.transpose((1, 0))
    # 现在 pred 是 (8400, 13)
    # 分离box坐标 (4个) 和类别分数 (9个)
    # 根据YOLOv8格式，通常是 [x, y, w, h, cls_scores...]
    pred_boxes = pred[..., :4]  # (8400, 4)
    pred_class = pred[..., 4:]  # (8400, 9)
    # 获取最大类别分数作为置信度
    pred_conf = np.max(pred_class, axis=-1)  # (8400,)
    # 重新组织为 [x, y, w, h, conf, cls_id, cls_scores...]
    # 其中 cls_id 是最大分数对应的类别
    cls_ids = np.argmax(pred_class, axis=-1)  # (8400,)
    # 返回格式： [x, y, w, h, conf, cls_id]
    pred = np.column_stack([pred_boxes, pred_conf, cls_ids])
    return pred


def xywh2xyxy(*box):
    """
    将xywh转换为左上角点和右下角点
    Args:
        box: x, y, w, h
    Returns: x1, y1, x2, y2
    """
    ret = [box[0] - box[2] // 2, box[1] - box[3] // 2,
           box[0] + box[2] // 2, box[1] + box[3] // 2]
    return ret


def get_inter(box1, box2):
    """
    计算相交部分面积
    Args:
        box1: 第一个框
        box2: 第二个框
    Returns: 相交部分的面积
    """
    x1, y1, x2, y2 = xywh2xyxy(*box1)
    x3, y3, x4, y4 = xywh2xyxy(*box2)
    # 验证是否存在交集
    if x1 >= x4 or x2 <= x3:
        return 0
    if y1 >= y4 or y2 <= y3:
        return 0
    # 将x1,x2,x3,x4排序，因为已经验证了两个框相交，所以x3-x2就是交集的宽
    x_list = sorted([x1, x2, x3, x4])
    x_inter = x_list[2] - x_list[1]
    # 将y1,y2,y3,y4排序，因为已经验证了两个框相交，所以y3-y2就是交集的宽
    y_list = sorted([y1, y2, y3, y4])
    y_inter = y_list[2] - y_list[1]
    # 计算交集的面积
    inter = x_inter * y_inter
    return inter


def get_iou(box1, box2):
    """
    计算交并比： (A n B)/(A + B - A n B)
    Args:
        box1: 第一个框
        box2: 第二个框
    Returns: 返回交并比的值
    """
    box1_area = box1[2] * box1[3]  # 计算第一个框的面积
    box2_area = box2[2] * box2[3]  # 计算第二个框的面积
    inter_area = get_inter(box1, box2)
    union = box1_area + box2_area - inter_area
    iou = inter_area / union
    return iou


def nms(pred, conf_thres, iou_thres):
    """
    非极大值抑制nms
    Args:
        pred: 模型输出特征图，格式 [x, y, w, h, conf, cls_id]
        conf_thres: 置信度阈值
        iou_thres: iou阈值
    Returns: 输出后的结果
    """
    # 置信度筛选
    box = pred[pred[..., 4] > conf_thres]
    if len(box) == 0:
        return []
    
    # cls_id 已经在第5列（索引5）
    cls = [int(b[5]) for b in box]
    
    total_cls = list(set(cls))  # 记录图像内共出现几种物体
    output_box = []
    
    # 每个预测类别分开考虑
    for clss in total_cls:
        cls_box = []
        for j in range(len(box)):
            # 记录[x,y,w,h,conf,class]值
            if cls[j] == clss:
                cls_box.append(box[j])
        
        # cls_box 里面是[x,y,w,h,conf,class]
        cls_box = np.array(cls_box)
        sort_cls_box = sorted(cls_box, key=lambda x: -x[4])  # 按置信度从大到小排序
        
        # 得到置信度最大的预测框
        max_conf_box = sort_cls_box[0]
        output_box.append(max_conf_box)
        sort_cls_box = np.delete(sort_cls_box, 0, 0)
        
        # 对除max_conf_box外其他的框进行非极大值抑制
        while len(sort_cls_box) > 0:
            # 得到当前最大的框
            max_conf_box = output_box[-1]
            del_index = []
            for j in range(len(sort_cls_box)):
                current_box = sort_cls_box[j]
                iou = get_iou(max_conf_box, current_box)
                if iou > iou_thres:
                    # 筛选出与当前最大框IoU大于阈值的框的索引
                    del_index.append(j)
            # 删除这些索引
            sort_cls_box = np.delete(sort_cls_box, del_index, 0)
            if len(sort_cls_box) > 0:
                output_box.append(sort_cls_box[0])
                sort_cls_box = np.delete(sort_cls_box, 0, 0)
    
    return output_box


def cod_trf(result, pre, after, letterbox_info):
    """
    因为预测框是在经过letterbox后的图像上做预测，所以需要将预测框的坐标映射回原图像上
    Args:
        result: [x,y,w,h,conf(最大类别概率),class]
        pre: 原尺寸图像
        after: 经过letterbox处理后的图像
        letterbox_info: (scale, (dw, dh))
    Returns: 坐标变换后的结果
    """
    res = np.array(result)
    if len(res) == 0:
        return []
    
    # res 格式: [x, y, w, h, conf, cls]
    ret = []
    for r in res:
        x, y, w, h, conf, cls = r
        x1, y1, x2, y2 = xywh2xyxy(x, y, w, h)
        
        scale, dw, dh = letterbox_info
        
        # 坐标转换回原图
        ret_x1 = (x1 - dw) / scale
        ret_x2 = (x2 - dw) / scale
        ret_y1 = (y1 - dh) / scale
        ret_y2 = (y2 - dh) / scale
        
        ret.append([ret_x1, ret_y1, ret_x2, ret_y2, conf, cls])
    
    return np.array(ret)


def draw(res, image, cls):
    """
    将预测框绘制在image上
    Args:
        res: 预测框数据 [x1, y1, x2, y2, conf, cls_id]
        image: 原图
        cls: 类别列表
    Returns:
    """
    if len(res) == 0:
        return image
    
    for r in res:
        if len(r) < 6:
            continue
        x1, y1, x2, y2 = int(r[0]), int(r[1]), int(r[2]), int(r[3])
        conf = r[4]
        cls_id = int(r[5])
        
        # 确保坐标在图像范围内
        x1 = max(0, min(x1, image.shape[1] - 1))
        y1 = max(0, min(y1, image.shape[0] - 1))
        x2 = max(x1 + 1, min(x2, image.shape[1]))
        y2 = max(y1 + 1, min(y2, image.shape[0]))
        
        # 画框
        image = cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2)
        # 表明类别
        if cls_id < len(cls):
            text = "{}:{}".format(cls[cls_id], round(float(conf), 2))
            print(text)
            h, w = y2 - y1, x2 - x1
            font_size = min(h/640, w/640) * 1
            image = cv2.putText(image, text, (max(10, x1), max(20, y1)),
                              cv2.FONT_HERSHEY_COMPLEX, max(font_size, 0.5), (0, 0, 255), 2)
    return image

