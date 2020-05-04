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
#undef ON
#undef OFF

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

        ticks = static_cast<std::int8_t>(wheel - wheel_previous);
        tick_pos += ticks;
        wheel_previous = wheel;
    }
};

enum class MPGLogicState {
    UNDEFINED,
    ESTOP,
    MACHINE_OFF,
    STEP,
    ZERO,
    FO_SPEED
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
    State<MPGLogicState, MPGLogicState::UNDEFINED> m_state;
    axis_t m_xAxis{'X', AxisId::X, 0.0f};
    axis_t m_yAxis{'Y', AxisId::Y, 0.0f};
    axis_t m_zAxis{'Z', AxisId::Z, 0.0f};
    axis_t *m_axis = nullptr;
    bool m_estop = false;
    bool m_machineOn = true;
    bool m_enableJog = false;
    bool m_enableZero = false;
    bool m_showZero = false;
    bool m_enableFeedOverride = false;
    float m_tickScale = 0.0f;

public:
    const axis_t *axis() const { return m_axis; }
    bool estop() const { return m_estop; }
    bool machineOn() const { return m_machineOn; }
    bool enableJog() const { return m_enableJog; }
    bool enableZero() const { return m_enableZero; }
    bool showZero() const { return m_showZero; }
    bool enableFeedOverride() const { return m_enableFeedOverride; }
    float tickScale() const { return m_tickScale; }

    void update(const MPGState &mpgState) {
        m_axis = nullptr;
        MPGLogicState state = MPGLogicState::UNDEFINED;

        if(hal_estop_is_activated()) {
            state = MPGLogicState::ESTOP;
        } else if(!hal_machine_is_on()) {
            state = MPGLogicState::MACHINE_OFF;
        } else if(mpgState.left_dial.state() == MPGState::LeftDialState::STEP) {
            switch(mpgState.right_dial.state()) {
            case MPGState::RightDialState::X_F1:
                state = MPGLogicState::STEP;
                m_axis = &m_xAxis;
                break; 
            case MPGState::RightDialState::Y_F2:
                state = MPGLogicState::STEP;
                m_axis = &m_yAxis;
                break; 
            case MPGState::RightDialState::Z_F3:
                state = MPGLogicState::STEP;
                m_axis = &m_zAxis;
                break;
            }
        } else if(mpgState.left_dial.state() == MPGState::LeftDialState::ZERO) {
            switch(mpgState.right_dial.state()) {
            case MPGState::RightDialState::X_F1:
                state = MPGLogicState::ZERO;
                m_axis = &m_xAxis;
                break; 
            case MPGState::RightDialState::Y_F2:
                state = MPGLogicState::ZERO;
                m_axis = &m_yAxis;
                break; 
            case MPGState::RightDialState::Z_F3:
                state = MPGLogicState::ZERO;
                m_axis = &m_zAxis;
                break;
            }
        } else if(mpgState.left_dial.state() == MPGState::LeftDialState::SPEED) {
            state = MPGLogicState::FO_SPEED;
        }

        m_state.update(state);

        switch(mpgState.right_dial.falling()) {
            case MPGState::RightDialState::X_F1:
            case MPGState::RightDialState::Y_F2:
            case MPGState::RightDialState::Z_F3:
                m_xAxis.offset = 0.0f;
                m_yAxis.offset = 0.0f;
                m_zAxis.offset = 0.0f;
                break;
        }

        m_estop = false;
        m_machineOn = true;
        m_enableJog = false;
        m_enableZero = false;
        m_showZero = false;
        m_enableFeedOverride = false;
        m_tickScale = (mpgState.button == MPGState::Button::RELEASED) ? 0.001f : 0.01f;


        if(m_state.state() == MPGLogicState::ESTOP) {
            m_estop = true;
            m_machineOn = false;
        } else if(m_state.state() == MPGLogicState::MACHINE_OFF) {
            m_machineOn = false;
        } else if(m_state.state() == MPGLogicState::STEP) {
            m_enableJog = true;
        } else if(m_state.state() == MPGLogicState::ZERO) {
            m_axis->offset += mpgState.ticks * m_tickScale;
            m_showZero = true;

            if(mpgState.button == MPGState::Button::DOUBLE_PRESSED) {
                m_enableZero = true;
            }
        } if(m_state.state() == MPGLogicState::FO_SPEED) {
            m_enableFeedOverride = true;
        }
    }
};

static libusb_context *ctx = nullptr;
static libusb_device_handle *handle = nullptr;
static MPGState mpgState;
static MPGLogic mpgLogic;

extern "C" {
    void init() {
        int success = libusb_init(&ctx);

        if(success != 0) {
            std::fprintf(stderr, "libusb_init failed\n");
            exit(1);
        }

        handle = libusb_open_device_with_vid_pid(ctx, 0x04d8, 0xfce2);

        if(!handle) {
            std::fprintf(stderr, "libusb_open_device_with_vid_pid failed\n");
            exit(1);
        }

        libusb_detach_kernel_driver(handle, 0);
        libusb_claim_interface(handle, 0);
    }

    void deinit() {
        libusb_exit(ctx);
    }
}

static const char *nmlfile = "/usr/share/linuxcnc/linuxcnc.nml";
static RCS_STAT_CHANNEL *stat = new RCS_STAT_CHANNEL(emcFormat, "emcStatus", "xemc", nmlfile);
static unsigned char screen[20];
static std::uint8_t screen_seq = 0;
static int coord_sys = 0;

extern "C" {
    void update() {

        if(stat->valid()) {
            if(stat->peek() == EMC_STAT_TYPE) {
                EMC_STAT *emcStatus = static_cast<EMC_STAT*>(stat->get_address());

                if(emcStatus->task.g5x_index != 0) {
                    coord_sys = emcStatus->task.g5x_index;
                }
            }
        }

        std::array<std::uint8_t, 8> buf;
        int actual;
        libusb_interrupt_transfer(handle, 0x81, buf.data(), buf.size(), &actual, 0);
        mpgState.update(buf);
        mpgLogic.update(mpgState);

        char str[sizeof(screen) + 1];
        int axisId = 0;

        std::memset(screen, ' ', sizeof(screen));

        if(mpgLogic.axis()) {
            switch(mpgLogic.axis()->id) {
            case MPGLogic::AxisId::X:
                axisId = 1;
                break;
            case MPGLogic::AxisId::Y:
                axisId = 2;
                break;
            case MPGLogic::AxisId::Z:
                axisId = 3;
                break;
            }

            int size = std::sprintf(str, "%c%+7.3f", mpgLogic.axis()->name, hal_axis_cmd_pos(axisId));
            std::memcpy(screen, str, size);
        }

        if(mpgLogic.estop()) {
            const char *estop = "EStop   Active";
            std::memcpy(screen, estop, strlen(estop));
        } else if(!mpgLogic.machineOn()) {
            const char *off = "Machine Off";
            std::memcpy(screen, off, strlen(off));
        }

        if(mpgLogic.showZero()) {
            int size = std::sprintf(str, "%d %+6.3f", coord_sys, mpgLogic.axis()->offset);
            std::memcpy(screen + 8, str, size);
        }

        if(mpgLogic.enableFeedOverride()) {
            int size = std::sprintf(str, "F%6.1f%%", hal_feed_override_value() * 100.0f);
            std::memcpy(screen, str, size);
        }

        screen[17] = screen_seq++;
        screen[18] = 0;
        screen[19] = 0;
        libusb_interrupt_transfer(handle, 0x01, screen, sizeof(screen), &actual, 0);

        hal_zero_offset((mpgLogic.axis()) ? mpgLogic.axis()->offset : 0.0f);
        hal_zero_axis((mpgLogic.enableZero()) ? axisId : 0);
        hal_jog_enable_axis((mpgLogic.enableJog()) ? axisId : 0);
        hal_jog_counts(mpgState.tick_pos);
        hal_feed_override_counts(mpgState.tick_pos);
        hal_feed_override_count_enable(mpgLogic.enableFeedOverride());
        hal_jog_scale(mpgLogic.tickScale());
    }
}
