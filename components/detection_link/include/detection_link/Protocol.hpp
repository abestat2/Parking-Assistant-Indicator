#pragma once

#include <cstdint>
#include <cstddef>

namespace detection_link {

// State of the monitored item as reported by the detector. Unknown means the detector has no
// valid sensor reading; it is still a successful exchange, never a stand-in for missing contact.
enum class ItemState : uint8_t {
    Unknown = 0,
    Absent = 1,
    Present = 2,
};

enum class MessageType : uint8_t {
    StatusRequest = 1,
    StatusResponse = 2,
    StatusAck = 3,
};

/*
 * Wire format. Fields are encoded explicitly, multi-byte fields little-endian:
 *
 *   offset  size  field
 *   0       2     protocol identifier (PROTOCOL_ID)
 *   2       1     protocol version (PROTOCOL_VERSION)
 *   3       1     message type (MessageType)
 *   4       4     exchange ID (never 0)
 *   8       1     item state (ItemState), STATUS_RESPONSE only
 *
 * STATUS_REQUEST and STATUS_ACK are HEADER_SIZE bytes; STATUS_RESPONSE is RESPONSE_SIZE bytes.
 */
constexpr uint16_t PROTOCOL_ID = 0x4150; // "PA" on the wire
constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr size_t HEADER_SIZE = 8;
constexpr size_t RESPONSE_SIZE = HEADER_SIZE + 1;
constexpr size_t MAX_MESSAGE_SIZE = RESPONSE_SIZE;

// Exchange ID 0 is never generated, so it can mean "no exchange"
constexpr uint32_t NO_EXCHANGE = 0;

struct Message {
    MessageType type;
    uint32_t exchange_id;
    ItemState item_state; // Only meaningful for StatusResponse
};

enum class DecodeResult {
    Ok,
    WrongProtocol,    // Too short to hold a header, or the identifier does not match
    WrongVersion,
    UnknownType,
    WrongLength,      // Length does not match the message type
    InvalidExchangeId,
    InvalidItemState,
};

/**
 * @brief Encode a message
 * @param message Message to encode
 * @param out Output buffer
 * @param capacity Size of the output buffer
 * @return size_t Number of bytes written, or 0 if the buffer is too small or the message is invalid
 */
size_t encode(const Message& message, uint8_t* out, size_t capacity);

/**
 * @brief Decode and validate a received packet
 * @param data Packet bytes
 * @param length Packet length
 * @param out Decoded message, written only when the result is DecodeResult::Ok
 * @return DecodeResult Ok, or the first validation that failed
 */
DecodeResult decode(const uint8_t* data, size_t length, Message& out);

const char* to_string(ItemState state);

} // namespace detection_link
