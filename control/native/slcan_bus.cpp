#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>

class SlcanSerialBus {
public:
    SlcanSerialBus() = default;
    ~SlcanSerialBus() { close_port(); }

    void open_port(const std::string& path, speed_t baud = B115200) {
        close_port();
        fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            throw std::runtime_error("failed to open serial port: " + std::string(std::strerror(errno)));
        }

        termios options{};
        if (tcgetattr(fd_, &options) != 0) {
            close_port();
            throw std::runtime_error("failed to read termios settings");
        }
        cfmakeraw(&options);
        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);
        options.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
        options.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;
        if (tcsetattr(fd_, TCSANOW, &options) != 0) {
            close_port();
            throw std::runtime_error("failed to apply termios settings");
        }
    }

    void close_port() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    void write_line(const std::string& line) const {
        require_open();
        const std::string data = line + "\r";
        const ssize_t written = ::write(fd_, data.data(), data.size());
        if (written < 0 || static_cast<size_t>(written) != data.size()) {
            throw std::runtime_error("failed to write serial line");
        }
    }

    std::string read_line(int timeout_ms) const {
        require_open();
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd_, &readfds);
        timeval timeout{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
        const int ready = ::select(fd_ + 1, &readfds, nullptr, nullptr, &timeout);
        if (ready < 0) {
            throw std::runtime_error("failed waiting for serial data");
        }
        if (ready == 0) {
            return {};
        }
        std::string out;
        char ch = 0;
        while (::read(fd_, &ch, 1) == 1) {
            if (ch == '\r' || ch == '\n') {
                break;
            }
            out.push_back(ch);
        }
        return out;
    }

private:
    void require_open() const {
        if (fd_ < 0) {
            throw std::runtime_error("serial port is not open");
        }
    }

    int fd_ = -1;
};
