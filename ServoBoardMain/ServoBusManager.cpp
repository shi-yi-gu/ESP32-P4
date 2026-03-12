#include "ServoBusManager.h"

/* ==================== 闂佸搫顑呯€氫即鍩€椤掑倸孝闁搞倝浜跺?==================== */

ServoBusManager::ServoBusManager() {
    _serial = nullptr;
    _writeCount = 0;

    // 闂佸憡甯楃换鍌烇綖閹版澘绀岄柡宓啫鍞ㄦ俊顐到閻楀繒妲愰敂閿亾?
    for (int i = 0; i <= MAX_SERVO_ID; i++) {
        _feedback[i].online = false;
        _feedback[i].rawPosition = 0;
        _feedback[i].speed = 0;
        _feedback[i].load = 0;
        _feedback[i].voltage = 0;
        _feedback[i].temperature = 0;
        _feedback[i].absolutePosition = 0;
        _feedback[i].turnCount = 0;
        _feedback[i].lastRawPosition = 0;
        _feedback[i].initialized = false;
        _feedback[i].lastUpdate = 0;
    }
}

/* ==================== 闂佸憡甯楃换鍌烇綖閹版澘绀?==================== */

void ServoBusManager::begin(uint8_t busIndex, int rxPin, int txPin, uint32_t baud) {
    // ESP32-P4 婵炴垶鎸昏ぐ鍐亹濞戙垹鍙婇柣妯垮皺濞?
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
    
    // 缂傚倷鐒﹂崹鐢告偩閻愵剛鈻旈柟缁樺笒缂嶆捇鏌涢幒宥呭祮妞ゃ儻缍侀幃褔鎮烽幍顔界殤
    _sms.pSerial = s;
}

/* ==================== 闂佸憡鑹鹃張顒勵敆閻愬搫绀冩繛鍡楃箰瀵?==================== */

void ServoBusManager::setTarget(uint8_t id, int16_t position, uint16_t speed, uint8_t acc) {
    if (_writeCount >= MAX_SERVOS_PER_BUS) return;

    // 闂傚倸瀚崝鏇㈠春濡ゅ懏鍎庢い鏃傛櫕閸ㄧ厧霉閿濆懐肖闁汇倕妫濋幊鐘诲礃閵婏妇鍑?(-30719 闂?30719)
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

    // 闁荤姴顑呴崯浼村极閵堝棭妲归柣鎰版涧椤ユ骞栫€涙ɑ灏垫繛鍫熷灴瀹曘儳浠︾紒銏犲Ρ闂佸憡鍔栭悷銉╁吹闁秴鏋?
    _sms.SyncWritePosEx(_writeIDs, _writeCount, _writePos, _writeSpd, _writeAcc);
    
    // 濠电偞鎸搁幊鎰板煘閺嶎偆纾介柟鎯х－閹?
    _writeCount = 0;
}

/* ==================== 闂佸憡鑹鹃張顒勵敆閻愬灚瀚氶悹鍥ㄥ絻缁插潡鏌ㄥ☉妯煎闁汇埄鍋嗛幑鍕Ω閵夛妇鐣卞┑鈽嗗亐閸嬫挻绻涢弶鎴劀缂?==================== */

int ServoBusManager::syncReadPositions(const uint8_t* ids, uint8_t count) {
    if (!_serial || count == 0) return 0;

    // 婵＄偛顑囬崑鐔哥珶閹烘鍤嬮柛顭戝亝缁ㄦ岸鏌涢幇顒佸櫣闁宦板妿閹即濡搁妷锕€瀣€闂佺鈧崑?
    const uint8_t ADDR_PRESENT_POSITION = 56;  // 婵炶揪绲界粔鍫曟偪閸℃稑鎹堕柡澶嬪缁?
    const uint8_t LEN_FEEDBACK = 8;            // pos(2) + speed(2) + load(2) + voltage(1) + temperature(1)

    int successCount = 0;

    // 婵炶揪缍€濞夋洟寮妶鍡╂Ч闁绘劙娼чˉ妤呭箹鐎涙ɑ灏垫繛鍫熷灴瀹曘儳浠︾紒銏犲Ρ闁荤姴娲╅褎鎱ㄥ☉銏″殑?
    // 1. 闂佸憡甯楃换鍌烇綖閹版澘绀岄柡宓嫬鈧鲸鎱ㄥ┑鍕姤妞?
    _sms.syncReadBegin(count, LEN_FEEDBACK, 100);  // 100ms 闁烩剝甯掗幊蹇擃渻?

    // 2. 闂佸憡鐟﹂崹鍧楀焵椤戣法顦﹂柟顔芥尭椤垽濡烽…鎴濇闁荤姴娲弨閬嶆儑?
    int ret = _sms.syncReadPacketTx((uint8_t*)ids, count, ADDR_PRESENT_POSITION, LEN_FEEDBACK);
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

    // 3. 闂佽浜介崕鏌ュ极瑜版帞宓侀柟顖炴緩閹烘鍑犻柟閭﹀幘濡层劌鈽夐幙鍐ф捣闁糕晜鐗犲鐢告惞鐟欏嫮鏆犻柡澶嗘櫆閺屻劌煤閺嶎厼绀?
    for (uint8_t i = 0; i < count; i++) {
        uint8_t id = ids[i];
        uint8_t rxBuf[LEN_FEEDBACK];
        
        int rxLen = _sms.syncReadPacketRx(id, rxBuf);
        
        if (rxLen == LEN_FEEDBACK) {
            auto decodeSigned = [](uint8_t lo, uint8_t hi) -> int16_t {
                uint16_t raw = ((uint16_t)hi << 8) | lo;
                return (raw & (1 << 15))
                           ? -(int16_t)(raw & ~(1 << 15))
                           : (int16_t)raw;
            };

            // 解析位置/速度/负载/电压/温度数据（小端序）
            int16_t rawPos = decodeSigned(rxBuf[0], rxBuf[1]);
            int16_t rawSpeed = decodeSigned(rxBuf[2], rxBuf[3]);
            int16_t rawLoad = decodeSigned(rxBuf[4], rxBuf[5]);
            uint8_t rawVoltage = rxBuf[6];
            uint8_t rawTemp = rxBuf[7];

            // 更新多圈位置（自动跨圈检测）
            _updateMultiTurnPosition(id, rawPos);

            // 更新其他反馈数据
            if (id <= MAX_SERVO_ID) {
                _feedback[id].speed = rawSpeed;
                _feedback[id].load = rawLoad;
                _feedback[id].voltage = rawVoltage;
                _feedback[id].temperature = rawTemp;
            }

            // 更新状态
            _feedback[id].online = true;
            _feedback[id].lastUpdate = millis();
            successCount++;
        } else {
            _feedback[id].online = false;
        }
    }

    // 4. 缂傚倷鐒﹂幐璇差焽椤愶箑瑙﹂悘鐐佃檸閸斿嫰鎮?
    _sms.syncReadEnd();

    return successCount;
}

/* ==================== 闁荤姵浜介崝灞斤耿閳ь剚淇婇鐔蜂壕濠电偞娼欓鍡涙偋鐎圭姷鐤€闁告劦浜為弳顒佺箾?==================== */

void ServoBusManager::_updateMultiTurnPosition(uint8_t id, int16_t newRawPos) {
    if (id > MAX_SERVO_ID) return;

    ServoFeedback& fb = _feedback[id];
    
    // 婵☆偓绲鹃悧妤咁敃婵傜绀嗘繝闈涙－濞兼鏌?
    if (!fb.initialized) {
        fb.rawPosition = newRawPos;
        fb.lastRawPosition = newRawPos;
        fb.absolutePosition = newRawPos;
        fb.turnCount = 0;
        fb.initialized = true;
        return;
    }

    // 闁荤姳绶ょ槐鏇㈡偩缂佹ɑ濯寸€广儱娲ㄩ弸鍌炴煕濞嗘兎顏嗏偓鍨叀閺?
    int16_t delta = newRawPos - fb.lastRawPosition;

    // 闁荤姵浜介崝灞斤耿閳ь剚淇婇鐔蜂壕濠电偞娼欓澶娢ｉ崶顒€纾圭痪顓㈩棑缁€鍕喐閻楀牊灏憸鏉款樀瀹曠娀寮介鐐搭仭闁烩剝甯掗幊鎾舵崲閸愵喖纭€濠电姴鍊圭粻鎴︽煛閸愨晛鍔舵い鎾剁帛缁嬪宕崟顐ゅ矝闂佹眹鍨婚崰宥囪姳閿涘嫭宕夐柕濞垮劜缁犳垿鏌?
    const int16_t HALF_TURN = 2048;  // 4096 / 2

    if (delta > HALF_TURN) {
        // 闂佸憡鐟ョ粔鎾箖濠婂懏宕夐柕濞垮劜缁犳垿鏌ㄥ☉妯煎缂?4095 -> 0闂?
        fb.turnCount--;
    } else if (delta < -HALF_TURN) {
        // 濠殿喗绻愮徊浠嬪箖濠婂懏宕夐柕濞垮劜缁犳垿鏌ㄥ☉妯煎缂?0 -> 4095闂?
        fb.turnCount++;
    }

    // 闂佸搫娲ら悺銊╁蓟婵犲洤鏋侀柣妤€鐗嗙粊?
    fb.rawPosition = newRawPos;
    fb.lastRawPosition = newRawPos;
    
    // 闁荤姳绶ょ槐鏇㈡偩閼姐倗纾兼繝闈涙－閸ょ姴霉閿濆懐肖闁?= 闂侀潻绠戦悧濠囧汲?* 4096 + 閻熸粎澧楅幐鍛婃櫠閻樿櫕濯寸€广儱娲ㄩ弸?
    fb.absolutePosition = (int32_t)fb.turnCount * 4096 + newRawPos;

    // 闂傚倸瀚崝鏇㈠春濡ゅ懎鎹堕柕濠忛檮缁犳帡鏌℃担鍝ュⅲ閻庡灚鐗犲畷鍫曞级閹存繃鏆?(-30719 闂?30719)
    if (fb.absolutePosition > 30719) {
        fb.absolutePosition = 30719;
    } else if (fb.absolutePosition < -30719) {
        fb.absolutePosition = -30719;
    }
}

/* ==================== 闂佽桨鑳舵晶妤€鐣垫担鐑樺闁秆勵殕閿?==================== */

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

int16_t ServoBusManager::readLoad(uint8_t id) {
    if (!_serial || id > MAX_SERVO_ID) return 0;

    int load = _sms.ReadLoad(id);
    if (load == -1) {
        _feedback[id].online = false;
        return 0;
    }

    _feedback[id].load = (int16_t)load;
    _feedback[id].online = true;
    _feedback[id].lastUpdate = millis();
    return (int16_t)load;
}
