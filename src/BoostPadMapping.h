#pragma once

#if __has_include(<RLGymCPP/GameStates/GameState.h>)
#include <RLGymCPP/GameStates/GameState.h>
#else
#include <RLGymCPP/Gamestates/GameState.h>
#endif
#include <corepacket_generated.h>
#include <array>
#include <cmath>
#include <string>

// Map RLBot pad indices to the observation's training order.
class BoostPadMapping {
public:
    static constexpr int COUNT = RLGC::CommonValues::BOOST_LOCATIONS_AMOUNT;

    bool Build(const rlbot::flat::FieldInfo* field) {
        ready_ = false;
        error_.clear();
        const auto* pads = field ? field->boost_pads() : nullptr;
        if (!pads || pads->size() != COUNT)
            return Fail("FieldInfo must contain exactly 34 Soccar boost pads");
        std::array<bool, COUNT> used{};
        for (int training = 0; training < COUNT; ++training) {
            const auto& expected = RLGC::CommonValues::BOOST_LOCATIONS[training];
            int match = -1;
            for (int packet = 0; packet < COUNT; ++packet) {
                const auto* pad = pads->Get(packet);
                const auto* pos = pad ? pad->location() : nullptr;
                if (!pos || !std::isfinite(pos->x()) || !std::isfinite(pos->y()))
                    return Fail("FieldInfo contains an invalid boost-pad location");
                const float dx = pos->x() - expected.x, dy = pos->y() - expected.y;
                if (dx * dx + dy * dy > 25.f)
                    continue;
                if (match != -1)
                    return Fail("FieldInfo contains ambiguous boost-pad locations");
                match = packet;
            }
            if (match < 0 || used[match])
                return Fail("FieldInfo does not uniquely match the trained Soccar field");
            const bool expectedBig = training == 3 || training == 4 || training == 15 ||
                training == 18 || training == 29 || training == 30;
            if (pads->Get(match)->is_full_boost() != expectedBig)
                return Fail("FieldInfo boost-pad size does not match training");
            used[match] = true;
            packetIndex_[training] = match;
        }
        ready_ = true;
        return true;
    }

    bool Apply(const rlbot::flat::GamePacket* packet, RLGC::GameState& state) const {
        const auto* pads = packet ? packet->boost_pads() : nullptr;
        if (!ready_ || !pads || pads->size() != COUNT)
            return false;
        for (int i = 0; i < COUNT; ++i) {
            const auto* pad = pads->Get(packetIndex_[i]);
            if (!pad || !std::isfinite(pad->timer()))
                return false;
        }
        for (int i = 0; i < COUNT; ++i) {
            const auto* pad = pads->Get(packetIndex_[i]);
            const int inverse = COUNT - 1 - i;
            state.boostPads[i] = pad->is_active();
            state.boostPadTimers[i] = pad->timer(); // Seconds until respawn.
            state.boostPadsInv[inverse] = state.boostPads[i];
            state.boostPadTimersInv[inverse] = state.boostPadTimers[i];
        }
        return true;
    }

    bool Ready() const { return ready_; }
    const std::string& Error() const { return error_; }
    int PacketIndex(int trainingIndex) const { return packetIndex_.at(trainingIndex); }

private:
    bool Fail(const char* message) { error_ = message; return false; }
    bool ready_ = false;
    std::array<int, COUNT> packetIndex_{};
    std::string error_;
};
