#include "RLBotClient.h"
#include <cstdlib>
#include <iostream>

#include <algorithm>
#include <array>
#include <cmath>

using namespace RLGC;

namespace
{
    Vec ToVec(const rlbot::flat::Vector3 rlbotVec) {
        return Vec(rlbotVec.x(), rlbotVec.y(), rlbotVec.z());
    }

    PhysState ToPhysObj(const rlbot::flat::Physics* phys) {
        PhysState obj = {};
        obj.pos = ToVec(phys->location());

        Angle ang = Angle(phys->rotation().yaw(), phys->rotation().pitch(), phys->rotation().roll());
        obj.rotMat = ang.ToRotMat();

        obj.vel = ToVec(phys->velocity());
        obj.angVel = ToVec(phys->angular_velocity());
        return obj;
    }


    RLGC::GalactusOBS::ExternalBallPredSamples ExtractBallPredSamples(
        const rlbot::flat::BallPrediction* prediction, float now, const PhysState& fallback)
    {
        RLGC::GalactusOBS::ExternalBallPredSamples samples{};
        samples.fill(fallback);
        if (!prediction || !prediction->slices() || prediction->slices()->size() == 0)
            return samples;
        constexpr std::array<float, 5> offsets{0.25f, 0.5f, 1.0f, 1.5f, 2.0f};
        const auto* slices = prediction->slices();
        size_t cursor = 0;
        for (size_t i = 0; i < offsets.size(); ++i) {
            const float target = now + offsets[i];
            while (cursor + 1 < slices->size() && slices->Get(cursor + 1)->game_seconds() <= target)
                ++cursor;
            size_t best = cursor;
            if (cursor + 1 < slices->size() &&
                std::fabs(slices->Get(cursor + 1)->game_seconds() - target) <
                std::fabs(slices->Get(cursor)->game_seconds() - target))
                best = cursor + 1;
            samples[i] = ToPhysObj(&slices->Get(best)->physics());
        }
        return samples;
    }

    Player ToPlayer(const rlbot::flat::PlayerInfo* playerInfo, float dtSec, PlayerTimingState& timing)
    {
        Player pd = {};

        static_cast<PhysState&>(pd) = ToPhysObj(playerInfo->physics());

        pd.carId = playerInfo->player_id();
        pd.team = (Team)playerInfo->team();

        pd.boost = playerInfo->boost();

        pd.isOnGround = (playerInfo->air_state() == rlbot::flat::AirState::OnGround);
        pd.isJumping = (playerInfo->air_state() == rlbot::flat::AirState::Jumping);
        pd.isFlipping = (playerInfo->air_state() == rlbot::flat::AirState::Dodging);
        pd.hasJumped = playerInfo->has_jumped();
        pd.hasDoubleJumped = playerInfo->has_double_jumped();
        pd.hasFlipped = playerInfo->has_dodged();
        pd.isDemoed = playerInfo->demolished_timeout() >= 0.f;

        if (pd.isOnGround) {
            timing.airTime = 0.f;
        }
        else {
            timing.airTime += dtSec;
        }

        pd.airTime = timing.airTime;
        // The deployed policy estimates flip availability from total airtime.
        pd.airTimeSinceJump = pd.hasJumped ? timing.airTime : 0.f;

        return pd;
    }

    GameState ToGameState(rlbot::flat::GamePacket const* packet, float dtSec, std::vector<PlayerTimingState>& playerTiming, const BoostPadMapping& boostPadMapping) {
        GameState gs = {};

        auto players = packet->players();
        if (players) {
            const int n = (int)players->size();
            if ((int)playerTiming.size() < n)
                playerTiming.resize(n);

            gs.players.reserve(n);
            for (int i = 0; i < n; i++) {
                auto player = ToPlayer(players->Get(i), dtSec, playerTiming[i]);
                player.index = i;
                gs.players.push_back(player);
            }
        }

        static_cast<PhysState&>(gs.ball) = ToPhysObj(packet->balls()->Get(0)->physics());

        if (!boostPadMapping.Apply(packet, gs)) {
            std::cerr << "Invalid or unmapped RLBot boost-pad packet; refusing incorrect observations.\n";
            std::exit(EXIT_FAILURE);
        }

        gs.deltaTime = dtSec;
        gs.lastTickCount = packet->match_info()->frame_num();
        return gs;
    }
} // anonymous namespace

RLBotBot::RLBotBot(std::unordered_set<unsigned> indices_,
    unsigned const team_,
    std::string name_,
    std::shared_ptr<const SharedBotContext> ctx) noexcept
    : rlbot::Bot(std::move(indices_), team_, std::move(name_))
    , ctx_(std::move(ctx))
{
    std::set<unsigned> sorted(std::begin(indices), std::end(indices));
    for (auto const& index : sorted)
        std::printf("Team %u Index %u: %s created\n", team_, index, name_.c_str());
}

RLBotBot::~RLBotBot() noexcept {}

void RLBotBot::initialize(rlbot::flat::ControllableTeamInfo const* controllable,
    rlbot::flat::FieldInfo const* field, rlbot::flat::MatchConfiguration const* match) noexcept {
    rlbot::Bot::initialize(controllable, field, match);
    if (!boostPadMapping_.Build(field)) {
        std::cerr << "Cannot initialize boost-pad mapping: " << boostPadMapping_.Error() << "\n";
        std::exit(EXIT_FAILURE);
    }
}

void RLBotBot::update(rlbot::flat::GamePacket const* packet,
    rlbot::flat::BallPrediction const* ballPrediction_) noexcept
{
    if (!boostPadMapping_.Ready()) return;

    if (!packet || !packet->match_info() || !packet->balls() || packet->balls()->size() == 0) {
        return;
    }

    const float curTime = packet->match_info()->seconds_elapsed();
    const float deltaTime = curTime - prevTime;
    prevTime = curTime;
    const float frameDt = std::clamp(deltaTime, 0.f, 1.f / 15.f);
    ticks += std::max(0, static_cast<int>(std::round(deltaTime * 120.f)));

    // Inference runs on the callback after the tick-skip boundary.
    GameState gs = ToGameState(packet, frameDt, m_playerTiming, boostPadMapping_);
    auto* galactus = dynamic_cast<GalactusOBS*>(ctx_->obs.get());
    if (galactus)
        galactus->SetExternalBallPredSamples(ExtractBallPredSamples(
            ballPrediction_, curTime, static_cast<const PhysState&>(gs.ball)));

    std::vector<unsigned> activeIndices;
    for (unsigned index : indices)
        if (index < gs.players.size())
            activeIndices.push_back(index);
    std::sort(activeIndices.begin(), activeIndices.end());

    for (unsigned index : activeIndices)
        gs.players[index].prevAction = m_botState[index].controls;

    if (updateAction && !activeIndices.empty()) {
        const auto actions = ctx_->inferUnit->BatchInferActions(activeIndices, gs, true);
        for (size_t row = 0; row < activeIndices.size(); ++row)
            m_botState[activeIndices[row]].action = actions[row];
    }

    for (unsigned index : activeIndices) {
        auto& st = m_botState[index];
        if (ticks >= ctx_->params.actionDelay || ticks == -1)
            st.controls = st.action;
        const auto& c = st.controls;
        setOutput(index, {
            c.throttle, c.steer, c.pitch, c.yaw, c.roll,
            c.jump > 0.5f, c.boost > 0.5f, c.handbrake > 0.5f, false,
        });
    }

    updateAction = false;
    if (ticks >= ctx_->params.tickSkip || ticks == -1) {
        ticks = 0;
        updateAction = true;
    }
}
