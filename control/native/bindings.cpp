#include <cstdint>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

struct NativeCanFrame {
    std::uint16_t arbitration_id;
    std::vector<std::uint8_t> data;
};

std::string encode_slcan_frame(const NativeCanFrame& frame) {
    if (frame.arbitration_id > 0x7FF) {
        throw std::invalid_argument("standard CAN id must be <= 0x7FF");
    }
    if (frame.data.size() > 8) {
        throw std::invalid_argument("CAN data length must be <= 8");
    }
    std::ostringstream out;
    out << 't' << std::uppercase << std::hex << std::setw(3) << std::setfill('0') << frame.arbitration_id;
    out << std::hex << frame.data.size();
    for (const std::uint8_t byte : frame.data) {
        out << std::setw(2) << static_cast<int>(byte);
    }
    out << '\r';
    return out.str();
}
