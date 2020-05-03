#include <array>
#include <fstream>
#include <vector>

#include <cctype>
#include <cstring>
#include <cstdio>
#include <cstdint>

#include <unistd.h>

#include <libusb-1.0/libusb.h>
#include <linuxcnc/emc.hh>
#include <linuxcnc/emc_nml.hh>

extern "C" {
    void hal_estop_activate();
    void hal_estop_reset();
    int hal_estop_is_activated();
    int hal_machine_is_on();
    void hal_zero_offset(float);
    void hal_zero_axis(int);
    void hal_jog_counts(int);
    void hal_jog_scale(float);
    void hal_jog_enable_axis(int);
    float hal_axis_cmd_pos(unsigned int);
    void hal_feed_override_counts(int i);
    void hal_feed_override_count_enable(int i);
    float hal_feed_override_value();
}

template<typename T, T t>
class State {
public:
    void update(T state) {
        if(state == m_state) {
            m_falling = t;
            m_rising = t;
            return;
        }

        m_falling = m_state;
        m_previous = m_state;
        m_rising = state;
        m_state = state;
    }

    T state() const { return m_state; }
    T rising() const { return m_rising; }
    T falling() const { return m_falling; }
    T previous() const { return m_previous; }

private:
    T m_state = t;
    T m_rising = t;
    T m_falling = t;
    T m_previous = t;
};

struct MPGState {
    enum class LeftDialState {
        UNDEFINED = 0,
        STEP = 1,
        VELOCITY = 2,
        CONTINUOUS = 3,
        SPEED = 4,
        ZERO = 5,
        FUNCTION = 6,
        OFF = 7
    };

    enum class RightDialState {
        UNDEFINED = 0,
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
    State<LeftDialState, LeftDialState::UNDEFINED> left_dial;
    State<RightDialState, RightDialState::UNDEFINED> right_dial;
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
        left_dial.update(LeftDialState(data[3] & 0b111));
        right_dial.update(RightDialState(data[2] & 0b111));
        button = Button((data[2] >> 6) & 0b11);
        estop = EStop((data[2] >> 5) & 0b1);
        off = (data[2] >> 4) & 0b1;
        counter1 = data[1];
        counter2 = data[4];
        counter3 = data[5];

        if(off) {
            left_dial.update(LeftDialState::OFF);
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

class MPGLogic {
public:
    enum class AxisId {
        X = 1,
        Y = 2,
        Z = 3
    };

    struct axis_t {
        char name;
        AxisId id;
        float offset;
    };

private:
    axis_t m_xAxis{'X', AxisId::X, 0.0f};
    axis_t m_yAxis{'Y', AxisId::Y, 0.0f};
    axis_t m_zAxis{'Z', AxisId::Z, 0.0f};
    
    std::array<char, 20> m_screen;
    std::uint8_t m_screenSeq;
    char m_str[9];
    int m_coordSys;
    const char *m_coordSystems[10] = {"??? 0", "G54 1", "G55 2", "G56 3", "G57 4", "G58 5", "G59 6", "G59.1 7", "G59.2 8", "G59.3 9"};

public:
    const auto &screen() { return m_screen; }

    void update(const MPGState &mpgState, int coord_sys) {
        float step_size = (mpgState.button == MPGState::Button::RELEASED) ? 0.001f : 0.01f;
        axis_t *axis = nullptr;
        bool enable_jog = false;
        bool zero = false;
        bool on = true;

        m_screen.fill(' ');
        m_screen[17] = m_screenSeq++;
        m_screen[18] = '\0';
        m_screen[19] = '\0';

        switch(mpgState.right_dial.falling()) {
            case MPGState::RightDialState::X_F1:
                m_xAxis.offset = 0.0f;
                break; 
            case MPGState::RightDialState::Y_F2:
                m_yAxis.offset = 0.0f;
                break; 
            case MPGState::RightDialState::Z_F3:
                m_zAxis.offset = 0.0f;
                break;
        }

        switch(mpgState.right_dial.state()) {
            case MPGState::RightDialState::X_F1:
                axis = &m_xAxis;
                break; 
            case MPGState::RightDialState::Y_F2:
                axis = &m_yAxis;
                break; 
            case MPGState::RightDialState::Z_F3:
                axis = &m_zAxis;
                break;
        }

        if(mpgState.estop == MPGState::EStop::ACTIVATED) {
            hal_estop_activate();
        } else {
            hal_estop_reset();
        }

        if(hal_estop_is_activated()) {
            on = false;
            axis = nullptr;
            const char *estop = "E-Stop  Active";
            std::memcpy(m_screen.data(), estop, std::strlen(estop));
        }

        if(!hal_machine_is_on() && !hal_estop_is_activated()) {
            on = false;
            axis = nullptr;
            const char *off = "Machine Off";
            std::memcpy(m_screen.data(), off, std::strlen(off));
        }

        if(mpgState.left_dial.state() != MPGState::LeftDialState::STEP && mpgState.left_dial.state() != MPGState::LeftDialState::ZERO) {
            axis = nullptr;
        }

        int halAxisId = (axis) ? static_cast<int>(axis->id) : 0;

        if(axis) {
            int size = snprintf(m_str, sizeof(m_str), "%+7.03f", hal_axis_cmd_pos(halAxisId));
            std::memcpy(m_screen.data() + 1, m_str, size);
            m_screen[0] = axis->name;

            if(mpgState.left_dial.state() == MPGState::LeftDialState::STEP) {
                enable_jog = true;
            } else if(mpgState.left_dial.state() == MPGState::LeftDialState::ZERO) {
                axis->offset += mpgState.ticks * step_size;
                hal_zero_offset(axis->offset);

                if(mpgState.button == MPGState::Button::DOUBLE_PRESSED) {
                    zero = true;
                }

                int size = snprintf(m_str, sizeof(m_str), "%+7.03f", axis->offset);
                std::memcpy(m_screen.data() + 9, m_str, size);
            }
        }

        if(coord_sys != 0) {
            m_coordSys = coord_sys;
        }

        if(on) {
            if(mpgState.left_dial.state() == MPGState::LeftDialState::ZERO) {
                m_screen[8] = '0' + m_coordSys;
            } else {
                std::memcpy(m_screen.data() + 8, m_coordSystems[m_coordSys], strlen(m_coordSystems[m_coordSys]));
            }
        }

        if(mpgState.left_dial.state() == MPGState::LeftDialState::SPEED) {
            hal_feed_override_count_enable(1);
            int size = snprintf(m_str, sizeof(m_str), "F %5.01f%%", hal_feed_override_value() * 100.0f);
            std::memcpy(m_screen.data(), m_str, size);
        } else {
            hal_feed_override_count_enable(0);
        }

        hal_zero_axis((zero) ? halAxisId : 0);
        hal_jog_enable_axis((enable_jog) ? halAxisId : 0);
        hal_jog_counts(mpgState.tick_pos);
        hal_feed_override_counts(mpgState.tick_pos);
        hal_jog_scale(step_size);
    }
};

static libusb_context *ctx = nullptr;
static libusb_device_handle *handle = nullptr;
static MPGState mpgState;
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

const char *nmlfile = "/usr/share/linuxcnc/linuxcnc.nml";
RCS_STAT_CHANNEL *stat = new RCS_STAT_CHANNEL(emcFormat, "emcStatus", "xemc", nmlfile);

extern "C" {
    void update() {
        int coord_sys = 0;

        if(stat->valid()) {
            if(stat->peek() == EMC_STAT_TYPE) {
                EMC_STAT *emcStatus = static_cast<EMC_STAT*>(stat->get_address());
                coord_sys = emcStatus->task.g5x_index;
            }
        }

        std::array<std::uint8_t, 8> buf;
        int actual;
        libusb_interrupt_transfer(handle, 0x81, buf.data(), buf.size(), &actual, 0);
        mpgState.update(buf);
        mpgLogic.update(mpgState, coord_sys);
        libusb_interrupt_transfer(handle, 0x01, (unsigned char *)mpgLogic.screen().data(), mpgLogic.screen().size(), &actual, 0);
    }
}