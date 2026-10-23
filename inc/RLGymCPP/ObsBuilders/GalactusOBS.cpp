#include "GalactusOBS.h"
#include <algorithm>
#include <array>
#include <RLGymCPP/GameStates/StateUtil.h>
#include <RocketSim/src/Sim/BallPredTracker/BallPredTracker.h>

namespace {
	constexpr uint64_t PRED_UPDATE_INTERVAL_TICKS = 32;  // Refresh every 32 sim ticks (~0.267s at 120Hz)
	constexpr int PLAYER_OBS_SIZE = 29;
	constexpr int MAX_STACK_PLAYERS = 8;
	constexpr float BIG_BOOST_PAD_DISTANCE_GATE = 1500.f;

	constexpr std::array<int, 6> BIG_BOOST_PAD_INDICES = {
		3, 4, 15, 18, 29, 30
	};

	constexpr std::array<size_t, 5> BALL_PRED_SAMPLE_TICKS = {
		30, 60, 120, 180, 240
	};

	// Cached predictions must cover the sample horizon plus their maximum age.
	constexpr size_t BALL_PRED_TICKS = BALL_PRED_SAMPLE_TICKS.back() + PRED_UPDATE_INTERVAL_TICKS + 1;

	inline int GetObsReserveSize(int maxPlayers, int actionAmount) {
		return
			9 + // Ball pos, vel, ang vel
			actionAmount +
			RLGC::CommonValues::BOOST_LOCATIONS_AMOUNT +
			3 + // Relative location of nearest active big boost pad within range
			PLAYER_OBS_SIZE + // Self
			PLAYER_OBS_SIZE * (maxPlayers - 1) + // Teammates
			PLAYER_OBS_SIZE * maxPlayers + // Opponents
			static_cast<int>(BALL_PRED_SAMPLE_TICKS.size()) * 6; // Ball prediction pos and vel samples
	}

	inline void AddEmptyPlayerObs(RLGC::FList& obs) {
		obs.insert(obs.end(), PLAYER_OBS_SIZE, 0.f);
	}

	inline void AddFloat(float*& obs, float val) {
		*obs++ = val;
	}

	inline void AddVec(float*& obs, const Vec& val) {
		*obs++ = val.x;
		*obs++ = val.y;
		*obs++ = val.z;
	}

	inline void AddEmptyPlayerObs(float*& obs) {
		for (int i = 0; i < PLAYER_OBS_SIZE; i++)
			*obs++ = 0.f;
	}

	inline void AddNearestActiveBigBoostPad(float*& obs, const RLGC::Player& player, bool inv, const std::vector<bool>& pads) {
		PhysState playerPhys = RLGC::InvertPhys(player, inv);
		Vec nearestRelative = {};
		float nearestDistSq = BIG_BOOST_PAD_DISTANCE_GATE * BIG_BOOST_PAD_DISTANCE_GATE;

		for (int padIdx : BIG_BOOST_PAD_INDICES) {
			if (padIdx < 0 || padIdx >= pads.size() || !pads[padIdx])
				continue;

			Vec padPos = RLGC::CommonValues::BOOST_LOCATIONS[padIdx];
			Vec relative = padPos - playerPhys.pos;
			float distSq = playerPhys.pos.DistSq2D(padPos);
			if (distSq <= nearestDistSq) {
				nearestDistSq = distSq;
				nearestRelative = relative;
			}
		}

		AddVec(obs, nearestRelative * RLGC::GalactusOBS::POS_COEF);
	}
}

RLGC::GalactusOBS::~GalactusOBS() {
	if (ballPredTracker) {
		delete ballPredTracker;
	}
}

void RLGC::GalactusOBS::SetExternalBallPredSamples(const ExternalBallPredSamples& samples) {
	externalBallPredSamples = samples;
	hasExternalBallPredSamples = true;
}

void RLGC::GalactusOBS::ClearExternalBallPredSamples() {
	hasExternalBallPredSamples = false;
}

void RLGC::GalactusOBS::Reset(const GameState& initialState) {
	if (initialState.lastArena) {
		GetPredictionData(initialState, true, true);
		lastPredUpdateTick = initialState.lastTickCount;
	}
}

const std::vector<BallState>* RLGC::GalactusOBS::GetPredictionData(const GameState& state, bool updateIfStale, bool forceUpdate) {
	if (!state.lastArena)
		return nullptr;

	if (!ballPredTracker) {
		ballPredTracker = new RocketSim::BallPredTracker(state.lastArena, BALL_PRED_TICKS);
	} else if (forceUpdate || updateIfStale) {
		ballPredTracker->UpdatePredFromArena(state.lastArena);
	}

	return &ballPredTracker->predData;
}

void RLGC::GalactusOBS::AddBallPredToObs(FList& obs, const GameState& state, bool inv) {
	if (hasExternalBallPredSamples) {
		for (PhysState predBall : externalBallPredSamples) {
			if (inv)
				predBall = predBall.GetInvertedY();

			obs += predBall.pos * POS_COEF;
			obs += predBall.vel * VEL_COEF;
		}
		return;
	}

	const std::vector<BallState>* predData = GetPredictionData(state, false);
	if (!predData) {
		for (size_t i = 0; i < BALL_PRED_SAMPLE_TICKS.size(); i++) {
			obs += Vec(0, 0, 0);
			obs += Vec(0, 0, 0);
		}
		return;
	}

	if (state.lastTickCount - lastPredUpdateTick >= PRED_UPDATE_INTERVAL_TICKS) {
		predData = GetPredictionData(state, true);
		lastPredUpdateTick = state.lastTickCount;
	}

	size_t predAgeTicks = (state.lastTickCount > lastPredUpdateTick) ?
		(size_t)(state.lastTickCount - lastPredUpdateTick) : 0;

	for (size_t tickIndex : BALL_PRED_SAMPLE_TICKS) {
		size_t sampleIndex = tickIndex + predAgeTicks;
		if (sampleIndex < predData->size()) {
			BallState predBall = (*predData)[sampleIndex];

			if (inv) {
				PhysState invertedPhys = predBall.GetInvertedY();
				predBall.pos = invertedPhys.pos;
				predBall.rotMat = invertedPhys.rotMat;
				predBall.vel = invertedPhys.vel;
				predBall.angVel = invertedPhys.angVel;
			}

			obs += predBall.pos * POS_COEF;
			obs += predBall.vel * VEL_COEF;
		} else {
			obs += Vec(0, 0, 0);
			obs += Vec(0, 0, 0);
		}
	}
}

void RLGC::GalactusOBS::AddBallPredToObs(float*& obs, const GameState& state, bool inv) {
	if (hasExternalBallPredSamples) {
		for (PhysState predBall : externalBallPredSamples) {
			if (inv)
				predBall = predBall.GetInvertedY();

			AddVec(obs, predBall.pos * POS_COEF);
			AddVec(obs, predBall.vel * VEL_COEF);
		}
		return;
	}

	const std::vector<BallState>* predData = GetPredictionData(state, false);
	if (!predData) {
		for (size_t i = 0; i < BALL_PRED_SAMPLE_TICKS.size(); i++) {
			AddVec(obs, Vec(0, 0, 0));
			AddVec(obs, Vec(0, 0, 0));
		}
		return;
	}

	if (state.lastTickCount - lastPredUpdateTick >= PRED_UPDATE_INTERVAL_TICKS) {
		predData = GetPredictionData(state, true);
		lastPredUpdateTick = state.lastTickCount;
	}

	size_t predAgeTicks = (state.lastTickCount > lastPredUpdateTick) ?
		(size_t)(state.lastTickCount - lastPredUpdateTick) : 0;

	for (size_t tickIndex : BALL_PRED_SAMPLE_TICKS) {
		size_t sampleIndex = tickIndex + predAgeTicks;
		if (sampleIndex < predData->size()) {
			BallState predBall = (*predData)[sampleIndex];

			if (inv) {
				PhysState invertedPhys = predBall.GetInvertedY();
				predBall.pos = invertedPhys.pos;
				predBall.rotMat = invertedPhys.rotMat;
				predBall.vel = invertedPhys.vel;
				predBall.angVel = invertedPhys.angVel;
			}

			AddVec(obs, predBall.pos * POS_COEF);
			AddVec(obs, predBall.vel * VEL_COEF);
		} else {
			AddVec(obs, Vec(0, 0, 0));
			AddVec(obs, Vec(0, 0, 0));
		}
	}
}

void RLGC::GalactusOBS::AddPlayerToObs(FList& obs, const Player& player, bool inv, const PhysState& ball) {
	auto phys = InvertPhys(player, inv);

	obs += phys.pos * POS_COEF;
	obs += phys.rotMat.forward;
	obs += phys.rotMat.up;
	obs += phys.vel * VEL_COEF;
	obs += phys.angVel * ANG_VEL_COEF;
	obs += phys.rotMat.Dot(phys.angVel) * ANG_VEL_COEF;

	obs += phys.rotMat.Dot(ball.pos - phys.pos) * POS_COEF;
	obs += phys.rotMat.Dot(ball.vel - phys.vel) * VEL_COEF;

	obs += player.boost / 100;
	obs += player.isOnGround;
	obs += player.HasFlipOrJump();
	obs += player.isDemoed;
	obs += player.hasJumped;
}

void RLGC::GalactusOBS::AddPlayerToObs(float*& obs, const Player& player, bool inv, const PhysState& ball) {
	auto phys = InvertPhys(player, inv);

	AddVec(obs, phys.pos * POS_COEF);
	AddVec(obs, phys.rotMat.forward);
	AddVec(obs, phys.rotMat.up);
	AddVec(obs, phys.vel * VEL_COEF);
	AddVec(obs, phys.angVel * ANG_VEL_COEF);
	AddVec(obs, phys.rotMat.Dot(phys.angVel) * ANG_VEL_COEF);

	AddVec(obs, phys.rotMat.Dot(ball.pos - phys.pos) * POS_COEF);
	AddVec(obs, phys.rotMat.Dot(ball.vel - phys.vel) * VEL_COEF);

	AddFloat(obs, player.boost / 100);
	AddFloat(obs, player.isOnGround);
	AddFloat(obs, player.HasFlipOrJump());
	AddFloat(obs, player.isDemoed);
	AddFloat(obs, player.hasJumped);
}

void RLGC::GalactusOBS::BuildObsInto(float* obs, const Player& player, const GameState& state) {
	if (maxPlayers > MAX_STACK_PLAYERS)
		RG_ERR_CLOSE("GalactusOBS: maxPlayers is too high for stack player slots, maximum is " << MAX_STACK_PLAYERS);

	float* out = obs;

	bool inv = player.team == Team::ORANGE;

	auto ball = InvertPhys(state.ball, inv);
	auto& pads = state.GetBoostPads(inv);
	auto& padTimers = state.GetBoostPadTimers(inv);

	AddVec(out, ball.pos * POS_COEF);
	AddVec(out, ball.vel * VEL_COEF);
	AddVec(out, ball.angVel * ANG_VEL_COEF);

	for (int i = 0; i < player.prevAction.ELEM_AMOUNT; i++)
		AddFloat(out, player.prevAction[i]);

	for (int i = 0; i < CommonValues::BOOST_LOCATIONS_AMOUNT; i++) {
		if (pads[i]) {
			AddFloat(out, 1.f);
		} else {
			AddFloat(out, 1.f / (1.f + padTimers[i]));
		}
	}

	AddNearestActiveBigBoostPad(out, player, inv, pads);

	AddPlayerToObs(out, player, inv, ball);

	std::array<const Player*, MAX_STACK_PLAYERS> teammates = {};
	std::array<const Player*, MAX_STACK_PLAYERS> opponents = {};
	int teammateCount = 0;
	int opponentCount = 0;

	for (auto& otherPlayer : state.players) {
		if (otherPlayer.carId == player.carId)
			continue;

		if (otherPlayer.team == player.team) {
			if (teammateCount >= maxPlayers - 1)
				RG_ERR_CLOSE("GalactusOBS: Too many teammates for Obs, maximum is " << (maxPlayers - 1));

			teammates[teammateCount++] = &otherPlayer;
		} else {
			if (opponentCount >= maxPlayers)
				RG_ERR_CLOSE("GalactusOBS: Too many opponents for Obs, maximum is " << maxPlayers);

			opponents[opponentCount++] = &otherPlayer;
		}
	}

	int teammateTargetCount = maxPlayers - 1;
	int opponentTargetCount = maxPlayers;

	std::shuffle(teammates.begin(), teammates.begin() + teammateTargetCount, ::Math::GetRandEngine());
	std::shuffle(opponents.begin(), opponents.begin() + opponentTargetCount, ::Math::GetRandEngine());

	for (int i = 0; i < teammateTargetCount; i++) {
		if (teammates[i]) {
			AddPlayerToObs(out, *teammates[i], inv, ball);
		} else {
			AddEmptyPlayerObs(out);
		}
	}

	for (int i = 0; i < opponentTargetCount; i++) {
		if (opponents[i]) {
			AddPlayerToObs(out, *opponents[i], inv, ball);
		} else {
			AddEmptyPlayerObs(out);
		}
	}

	AddBallPredToObs(out, state, inv);

	RG_ASSERT(out - obs == GetObsReserveSize(maxPlayers, player.prevAction.ELEM_AMOUNT));
}

RLGC::FList RLGC::GalactusOBS::BuildObs(const Player& player, const GameState& state) {
	FList result(GetObsReserveSize(maxPlayers, player.prevAction.ELEM_AMOUNT));
	BuildObsInto(result.data(), player, state);
	return result;
}
