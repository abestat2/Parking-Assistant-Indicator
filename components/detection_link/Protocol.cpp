#include "detection_link/Protocol.hpp"

namespace detection_link {

static void put_u16(uint8_t* out, uint16_t value) {
    out[0] = static_cast<uint8_t>(value);
    out[1] = static_cast<uint8_t>(value >> 8);
}

static void put_u32(uint8_t* out, uint32_t value) {
    for (int i = 0; i < 4; ++i) {
        out[i] = static_cast<uint8_t>(value >> (8 * i));
    }
}

static uint16_t get_u16(const uint8_t* in) {
    return static_cast<uint16_t>(in[0] | (in[1] << 8));
}

static uint32_t get_u32(const uint8_t* in) {
    return static_cast<uint32_t>(in[0]) | (static_cast<uint32_t>(in[1]) << 8) |
           (static_cast<uint32_t>(in[2]) << 16) | (static_cast<uint32_t>(in[3]) << 24);
}

static bool is_valid_type(uint8_t type) {
    return type >= static_cast<uint8_t>(MessageType::StatusRequest) &&
           type <= static_cast<uint8_t>(MessageType::StatusAck);
}

static bool is_valid_item_state(uint8_t state) {
    return state <= static_cast<uint8_t>(ItemState::Present);
}

static size_t size_of(MessageType type) {
    return type == MessageType::StatusResponse ? RESPONSE_SIZE : HEADER_SIZE;
}

size_t encode(const Message& message, uint8_t* out, size_t capacity) {
    if (!out || message.exchange_id == NO_EXCHANGE || !is_valid_type(static_cast<uint8_t>(message.type))) {
        return 0;
    }
    const size_t size = size_of(message.type);
    if (capacity < size) {
        return 0;
    }

    put_u16(&out[0], PROTOCOL_ID);
    out[2] = PROTOCOL_VERSION;
    out[3] = static_cast<uint8_t>(message.type);
    put_u32(&out[4], message.exchange_id);
    if (message.type == MessageType::StatusResponse) {
        if (!is_valid_item_state(static_cast<uint8_t>(message.item_state))) {
            return 0;
        }
        out[8] = static_cast<uint8_t>(message.item_state);
    }
    return size;
}

DecodeResult decode(const uint8_t* data, size_t length, Message& out) {
    if (!data || length < HEADER_SIZE || get_u16(&data[0]) != PROTOCOL_ID) {
        return DecodeResult::WrongProtocol;
    }
    if (data[2] != PROTOCOL_VERSION) {
        return DecodeResult::WrongVersion;
    }
    if (!is_valid_type(data[3])) {
        return DecodeResult::UnknownType;
    }
    const auto type = static_cast<MessageType>(data[3]);
    if (length != size_of(type)) {
        return DecodeResult::WrongLength;
    }
    const uint32_t exchange_id = get_u32(&data[4]);
    if (exchange_id == NO_EXCHANGE) {
        return DecodeResult::InvalidExchangeId;
    }
    ItemState state = ItemState::Unknown;
    if (type == MessageType::StatusResponse) {
        if (!is_valid_item_state(data[8])) {
            return DecodeResult::InvalidItemState;
        }
        state = static_cast<ItemState>(data[8]);
    }

    out.type = type;
    out.exchange_id = exchange_id;
    out.item_state = state;
    return DecodeResult::Ok;
}

const char* to_string(ItemState state) {
    switch (state) {
    case ItemState::Present: return "Present";
    case ItemState::Absent: return "Absent";
    case ItemState::Unknown: return "Unknown";
    }
    return "Invalid";
}

} // namespace detection_link
