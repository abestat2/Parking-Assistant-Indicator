// Tier 1: message encoding and validation, shared by the host and target test apps

#include "unity.h"
#include "detection_link/Protocol.hpp"
#include <cstring>

using namespace detection_link;

TEST_CASE("protocol: request encodes to the documented little-endian layout", "[detection_link][protocol]")
{
    uint8_t out[MAX_MESSAGE_SIZE] = {};
    const Message message = {MessageType::StatusRequest, 0x12345678, ItemState::Unknown};
    TEST_ASSERT_EQUAL_size_t(HEADER_SIZE, encode(message, out, sizeof(out)));
    const uint8_t expected[] = {0x50, 0x41, PROTOCOL_VERSION, 1, 0x78, 0x56, 0x34, 0x12};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, sizeof(expected));
}

TEST_CASE("protocol: response carries the item state after the header", "[detection_link][protocol]")
{
    uint8_t out[MAX_MESSAGE_SIZE] = {};
    const Message message = {MessageType::StatusResponse, 0xA1B2C3D4, ItemState::Present};
    TEST_ASSERT_EQUAL_size_t(RESPONSE_SIZE, encode(message, out, sizeof(out)));
    const uint8_t expected[] = {0x50, 0x41, PROTOCOL_VERSION, 2, 0xD4, 0xC3, 0xB2, 0xA1, 2};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, sizeof(expected));
}

TEST_CASE("protocol: every message type and item state round-trips", "[detection_link][protocol]")
{
    const MessageType types[] = {MessageType::StatusRequest, MessageType::StatusResponse, MessageType::StatusAck};
    const ItemState states[] = {ItemState::Unknown, ItemState::Absent, ItemState::Present};
    for (MessageType type : types) {
        for (ItemState state : states) {
            const Message message = {type, 0xFFFFFFFF, state};
            uint8_t bytes[MAX_MESSAGE_SIZE];
            const size_t length = encode(message, bytes, sizeof(bytes));
            TEST_ASSERT_NOT_EQUAL(0, length);

            Message decoded = {};
            TEST_ASSERT_EQUAL(DecodeResult::Ok, decode(bytes, length, decoded));
            TEST_ASSERT_EQUAL(type, decoded.type);
            TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFF, decoded.exchange_id);
            if (type == MessageType::StatusResponse) {
                TEST_ASSERT_EQUAL(state, decoded.item_state);
            } else {
                TEST_ASSERT_EQUAL(ItemState::Unknown, decoded.item_state);
            }
        }
    }
}

TEST_CASE("protocol: encode refuses small buffers and invalid messages", "[detection_link][protocol]")
{
    uint8_t out[MAX_MESSAGE_SIZE];
    TEST_ASSERT_EQUAL_size_t(0, encode({MessageType::StatusResponse, 1, ItemState::Absent}, out, RESPONSE_SIZE - 1));
    TEST_ASSERT_EQUAL_size_t(0, encode({MessageType::StatusRequest, 1, ItemState::Unknown}, out, HEADER_SIZE - 1));
    TEST_ASSERT_EQUAL_size_t(0, encode({MessageType::StatusRequest, 1, ItemState::Unknown}, nullptr, sizeof(out)));
    TEST_ASSERT_EQUAL_size_t(0, encode({MessageType::StatusAck, NO_EXCHANGE, ItemState::Unknown}, out, sizeof(out)));
    TEST_ASSERT_EQUAL_size_t(0, encode({static_cast<MessageType>(9), 1, ItemState::Unknown}, out, sizeof(out)));
    TEST_ASSERT_EQUAL_size_t(0, encode({MessageType::StatusResponse, 1, static_cast<ItemState>(7)}, out, sizeof(out)));
}

// A valid response to corrupt one field at a time
static void valid_response(uint8_t (&bytes)[RESPONSE_SIZE])
{
    TEST_ASSERT_EQUAL_size_t(RESPONSE_SIZE, encode({MessageType::StatusResponse, 42, ItemState::Absent}, bytes, sizeof(bytes)));
}

TEST_CASE("protocol: decode rejects each malformed field", "[detection_link][protocol]")
{
    uint8_t bytes[RESPONSE_SIZE];
    Message out = {MessageType::StatusAck, 99, ItemState::Present};

    TEST_ASSERT_EQUAL(DecodeResult::WrongProtocol, decode(nullptr, RESPONSE_SIZE, out));
    valid_response(bytes);
    TEST_ASSERT_EQUAL(DecodeResult::WrongProtocol, decode(bytes, HEADER_SIZE - 1, out));

    valid_response(bytes);
    bytes[1] = 0x00;
    TEST_ASSERT_EQUAL(DecodeResult::WrongProtocol, decode(bytes, sizeof(bytes), out));

    valid_response(bytes);
    bytes[2] = PROTOCOL_VERSION + 1;
    TEST_ASSERT_EQUAL(DecodeResult::WrongVersion, decode(bytes, sizeof(bytes), out));

    valid_response(bytes);
    bytes[3] = 0;
    TEST_ASSERT_EQUAL(DecodeResult::UnknownType, decode(bytes, sizeof(bytes), out));
    bytes[3] = 4;
    TEST_ASSERT_EQUAL(DecodeResult::UnknownType, decode(bytes, sizeof(bytes), out));

    valid_response(bytes);
    TEST_ASSERT_EQUAL(DecodeResult::WrongLength, decode(bytes, HEADER_SIZE, out));

    valid_response(bytes);
    std::memset(&bytes[4], 0, 4);
    TEST_ASSERT_EQUAL(DecodeResult::InvalidExchangeId, decode(bytes, sizeof(bytes), out));

    valid_response(bytes);
    bytes[8] = 3;
    TEST_ASSERT_EQUAL(DecodeResult::InvalidItemState, decode(bytes, sizeof(bytes), out));

    // A failed decode leaves the output alone
    TEST_ASSERT_EQUAL(MessageType::StatusAck, out.type);
    TEST_ASSERT_EQUAL_UINT32(99, out.exchange_id);
}

TEST_CASE("protocol: request or ack with trailing bytes is rejected", "[detection_link][protocol]")
{
    uint8_t bytes[RESPONSE_SIZE] = {};
    Message out;
    TEST_ASSERT_EQUAL_size_t(HEADER_SIZE, encode({MessageType::StatusAck, 5, ItemState::Unknown}, bytes, sizeof(bytes)));
    TEST_ASSERT_EQUAL(DecodeResult::WrongLength, decode(bytes, HEADER_SIZE + 1, out));
    TEST_ASSERT_EQUAL(DecodeResult::Ok, decode(bytes, HEADER_SIZE, out));
}

TEST_CASE("protocol: item states have names", "[detection_link][protocol]")
{
    TEST_ASSERT_EQUAL_STRING("Present", to_string(ItemState::Present));
    TEST_ASSERT_EQUAL_STRING("Absent", to_string(ItemState::Absent));
    TEST_ASSERT_EQUAL_STRING("Unknown", to_string(ItemState::Unknown));
}
