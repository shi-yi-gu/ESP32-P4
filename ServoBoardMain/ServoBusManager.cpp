#include "ServoBusManager.h"

/* ==================== 构造函数 ==================== */

ServoBusManager::ServoBusManager() {
    _serial = nullptr;
    _writeCount = 0;

    // 初始化反馈缓存
    for (int i = 0; i <= MAX_SERVO_ID; i++) {
        _feedback[i].online = false;
        _feedback[i].rawPosition = 0;
        _feedback[i].absolutePosition = 0;
        _feedback[i].turnCount = 0;
        _feedback[i].lastRawPosition = 0;
        _feedback[i].initialized = false;
        _feedback[i].lastUpdate = 0;
    }
}

/* ==================== 初始化 ==================== */

void ServoBusManager::begin(uint8_t busIndex, int rxPin, int txPin, uint32_t baud) {
    // ESP32-P4 串口映射
    HardwareSerial* s = nullptr;
    switch (busIndex) {
        case 0: s = &Serial1; break;
        case 1: s = &Serial2; break;
        case 2: s = &Serial3; break;
        case 3: s = &Serial4; break;
        default: return;
    }

    _serial = s;
    s->begin(baud, SERIAL_8N1, rxPin, txPin);
    
    // 绑定串口到飞特库
    _sms.pSerial = s;
}

/* ==================== 同步写入 ==================== */

void ServoBusManager::setTarget(uint8_t id, int16_t position, uint16_t speed, uint8_t acc) {
    if (_writeCount >= MAX_SERVOS_PER_BUS) return;

    // 限制目标位置范围 (-30719 到 30719)
    if (position < -30719) position = -30719;
    if (position > 30719) position = 30719;

    _writeIDs[_writeCount] = id;
    _writePos[_writeCount] = position;
    _writeSpd[_writeCount] = speed;
    _writeAcc[_writeCount] = acc;
    _writeCount++;
}

void ServoBusManager::syncWriteAll() {
    if (_writeCount == 0 || !_serial) return;

    // 调用飞特库的同步写函数
    _sms.SyncWritePosEx(_writeIDs, _writeCount, _writePos, _writeSpd, _writeAcc);
    
    // 清空缓存
    _writeCount = 0;
}

/* ==================== 同步读取（带跨圈检测） ==================== */

int ServoBusManager::syncReadPositions(const uint8_t* ids, uint8_t count) {
    if (!_serial || count == 0) return 0;

    // 飞特舵机内存表地址
    const uint8_t ADDR_PRESENT_POSITION = 56;  // 位置地址
    const uint8_t LEN_POSITION = 2;            // 位置长度（2 字节）

    int successCount = 0;

    // 使用飞特库的同步读功能
    // 1. 初始化同步读
    _sms.syncReadBegin(count, LEN_POSITION, 100);  // 100ms 超时

    // 2. 发送同步读请求
    int ret = _sms.syncReadPacketTx((uint8_t*)ids, count, ADDR_PRESENT_POSITION, LEN_POSITION);
    if (ret <= 0) {
        for (uint8_t i = 0; i < count; i++) {
            uint8_t id = ids[i];
            if (id <= MAX_SERVO_ID) {
                _feedback[id].online = false;
            }
        }
        _sms.syncReadEnd();
        return 0;
    }

    // 3. 接收并解析每个舵机的返回包
    for (uint8_t i = 0; i < count; i++) {
        uint8_t id = ids[i];
        uint8_t rxBuf[LEN_POSITION];
        
        int rxLen = _sms.syncReadPacketRx(id, rxBuf);
        
        if (rxLen == LEN_POSITION) {
            // 解析位置数据（小端序）
            uint16_t rawWord = ((uint16_t)rxBuf[1] << 8) | rxBuf[0];
            int16_t rawPos = (rawWord & (1 << 15))
                                 ? -(int16_t)(rawWord & ~(1 << 15))
                                 : (int16_t)rawWord;
            
            // 更新多圈位置（自动跨圈检测）
            _updateMultiTurnPosition(id, rawPos);
            
            // 更新状态
            _feedback[id].online = true;
            _feedback[id].lastUpdate = millis();
            successCount++;
        } else {
            _feedback[id].online = false;
        }
    }

    // 4. 结束同步读
    _sms.syncReadEnd();

    return successCount;
}

/* ==================== 跨圈检测核心算法 ==================== */

void ServoBusManager::_updateMultiTurnPosition(uint8_t id, int16_t newRawPos) {
    if (id > MAX_SERVO_ID) return;

    ServoFeedback& fb = _feedback[id];
    
    // 首次初始化
    if (!fb.initialized) {
        fb.rawPosition = newRawPos;
        fb.lastRawPosition = newRawPos;
        fb.absolutePosition = newRawPos;
        fb.turnCount = 0;
        fb.initialized = true;
        return;
    }

    // 计算位置变化量
    int16_t delta = newRawPos - fb.lastRawPosition;

    // 跨圈检测阈值（当变化量超过半圈时认为发生了跨圈）
    const int16_t HALF_TURN = 2048;  // 4096 / 2

    if (delta > HALF_TURN) {
        // 反向跨圈（从 4095 -> 0）
        fb.turnCount--;
    } else if (delta < -HALF_TURN) {
        // 正向跨圈（从 0 -> 4095）
        fb.turnCount++;
    }

    // 更新数据
    fb.rawPosition = newRawPos;
    fb.lastRawPosition = newRawPos;
    
    // 计算绝对位置 = 圈数 * 4096 + 当前位置
    fb.absolutePosition = (int32_t)fb.turnCount * 4096 + newRawPos;

    // 限制在有效范围内 (-30719 到 30719)
    if (fb.absolutePosition > 30719) {
        fb.absolutePosition = 30719;
    } else if (fb.absolutePosition < -30719) {
        fb.absolutePosition = -30719;
    }
}

/* ==================== 数据访问 ==================== */

int16_t ServoBusManager::getRawPosition(uint8_t id) const {
    if (id > MAX_SERVO_ID) return -1;
    return _feedback[id].rawPosition;
}

int32_t ServoBusManager::getAbsolutePosition(uint8_t id) const {
    if (id > MAX_SERVO_ID) return 0;
    return _feedback[id].absolutePosition;
}

void ServoBusManager::resetTurnCounter(uint8_t id) {
    if (id > MAX_SERVO_ID) return;
    
    ServoFeedback& fb = _feedback[id];
    fb.turnCount = 0;
    fb.absolutePosition = fb.rawPosition;
}

const ServoFeedback& ServoBusManager::getFeedback(uint8_t id) const {
    static ServoFeedback dummy;
    if (id > MAX_SERVO_ID) return dummy;
    return _feedback[id];
}

bool ServoBusManager::isOnline(uint8_t id) const {
    if (id > MAX_SERVO_ID) return false;
    return _feedback[id].online;
}
