/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
 * Description: Definition of Packet.
 * Author: MindX SDK
 * Create: 2022
 * History: NA
 */

#ifndef PACKET_H
#define PACKET_H

namespace MxStream {
template <class T>
class Packet {
public:
    Packet() = default;
    Packet(T item) : item_(item) {}
    T GetItem()
    {
        return item_;
    }
    void SetItem(T& item)
    {
        item_ = item;
    }

private:
    T item_;
};
}
    
#endif