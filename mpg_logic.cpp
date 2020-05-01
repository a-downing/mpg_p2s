#include <array>

#include <cstring>
#include <cstdio>
#include <cstdint>

#include <unistd.h>

#include <libusb-1.0/libusb.h>

extern "C" {
    void hal_estop_activate();
    void hal_estop_reset();
    int hal_estop_is_activated();
    int hal_machine_is_on();
    void hal_zero_offset(float);
    void hal_zero_axis(int);
    void hal_jog_counts(float);
    void hal_jog_scale(float);
    void hal_jog_enable_axis(int);
    float hal_axis_cmd_pos(unsigned int);
}

struct MPGState {
    enum class LeftDial : std::uint8_t {
        STEP = 1,
        VELOCITY = 2,
        CONTINUOUS = 3,
        SPEED = 4,
        ZERO = 5,
        FUNCTION = 6,
        OFF = 7
    };

    enum class RightDial : std::uint8_t {
        X_F1 = 1,
        Y_F2 = 2,
        Z_F3 = 3,
        A_F4 = 7,
        START_PAUSE = 4,
        STOP_BACK = 5,
        SPINDLE = 6
    };

    enum class Button: uint8_t {
        RELEASED = 0,
        PRESSED = 1,
        DOUBLE_PRESSED = 3
    };

    enum class EStop: uint8_t {
        UNACTIVATED = 0,
        ACTIVATED = 1
    };

    std::uint8_t wheel;
    std::uint8_t wheel_previous;
    LeftDial left_dial;
    RightDial right_dial;
    Button button;
    EStop estop;
    bool off;
    std::uint8_t counter1;
    std::uint8_t counter2;
    std::uint8_t counter3;
    bool initialized = false;
    std::int8_t ticks = 0;
    std::int64_t tick_pos = 0;

    void update(std::array<std::uint8_t, 8> &data) {
        wheel = data[0];
        left_dial = LeftDial(data[3] & 0b111);
        right_dial = RightDial(data[2] & 0b111);
        button = Button((data[2] >> 6) & 0b11);
        estop = EStop((data[2] >> 5) & 0b1);
        off = (data[2] >> 4) & 0b1;
        counter1 = data[1];
        counter2 = data[4];
        counter3 = data[5];

        if(off) {
            left_dial = LeftDial::OFF;
        }

        if(!initialized) {
            wheel_previous = wheel;
            initialized = true;
            return;
        }

        ticks = wheel - wheel_previous;
        tick_pos += ticks;
        wheel_previous = wheel;
    }
};

struct MPGLogic {
    enum class Axis : unsigned int {
        NONE = 0,
        X = 1,
        Y = 2,
        Z = 3,
        LAST
    };

    MPGState mpgState;
    Axis axis = Axis::NONE;
    Axis prev_axis = axis;
    float offsets[(std::size_t)Axis::LAST] = {0};
    std::array<char, 20> screen;
    std::uint8_t screen_seq;
    char str[9];

    char getAxis() {
        switch(axis) {
        case Axis::X:
            return 'X';
        case Axis::Y:
            return 'Y';
        case Axis::Z:
            return 'Z';
        }

        return 0;
    }

    float &axisOffset(Axis axis) {
        return offsets[(std::size_t)axis];
    }

    void update(std::array<std::uint8_t, 8> &data) {
        mpgState.update(data);
        float step_size = (mpgState.button == MPGState::Button::RELEASED) ? 0.001f : 0.01f;
        axis = Axis::NONE;
        Axis enable_axis = Axis::NONE;
        Axis zero_axis = Axis::NONE;

        screen.fill(' ');
        screen[17] = screen_seq++;
        screen[18] = '\0';
        screen[19] = '\0';

        switch(mpgState.right_dial) {
            case MPGState::RightDial::X_F1:
                axis = Axis::X;
                break; 
            case MPGState::RightDial::Y_F2:
                axis = Axis::Y;
                break; 
            case MPGState::RightDial::Z_F3:
                axis = Axis::Z;
                break;
        }

        if(mpgState.estop == MPGState::EStop::ACTIVATED) {
            hal_estop_activate();
        } else {
            hal_estop_reset();
        }

        if(hal_estop_is_activated()) {
            axis = Axis::NONE;
            const char *estop = "E-Stop  Active";
            std::memcpy(screen.data(), estop, std::strlen(estop));
        }

        if(!hal_machine_is_on() && !hal_estop_is_activated()) {
            axis = Axis::NONE;
            const char *off = "Machine Off";
            std::memcpy(screen.data(), off, std::strlen(off));
        }

        if(axis != Axis::NONE) {
            int size = snprintf(str, sizeof(str), "%+7.03f", hal_axis_cmd_pos(static_cast<unsigned int>(axis)));
            std::memcpy(screen.data() + 1, str, size);
            screen[0] = getAxis();

            if(mpgState.left_dial == MPGState::LeftDial::STEP) {
                enable_axis = axis;
            } else if(mpgState.left_dial == MPGState::LeftDial::ZERO) {
                offsets[(std::size_t)axis] += mpgState.ticks * step_size;
                hal_zero_offset(offsets[(std::size_t)axis]);

                if(mpgState.button == MPGState::Button::DOUBLE_PRESSED) {
                    zero_axis = axis;
                }

                int size = snprintf(str, sizeof(str), "%+7.03f", offsets[(std::size_t)axis]);
                std::memcpy(screen.data() + 9, str, size);
            }
        }

        hal_zero_axis((unsigned int)zero_axis);
        hal_jog_enable_axis((unsigned int)enable_axis);
        hal_jog_counts(mpgState.tick_pos);
        hal_jog_scale(step_size);
    }
};

static libusb_context *ctx = nullptr;
static libusb_device_handle *handle = nullptr;
static MPGLogic mpgLogic;

extern "C" {
    void init() {
        libusb_init(&ctx);
        handle = libusb_open_device_with_vid_pid(ctx, 0x04d8, 0xfce2);
        libusb_detach_kernel_driver(handle, 0);
        libusb_claim_interface(handle, 0);
    }

    void deinit() {
        libusb_exit(ctx);
    }
}

extern "C" {
    void update() {
        std::array<std::uint8_t, 8> buf;
        int actual;
        libusb_interrupt_transfer(handle, 0x81, buf.data(), buf.size(), &actual, 0);
        mpgLogic.update(buf);
        libusb_interrupt_transfer(handle, 0x01, (unsigned char *)mpgLogic.screen.data(), mpgLogic.screen.size(), &actual, 0);
    }
}