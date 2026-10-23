#pragma once
#include "ObsBuilder.h"
#include <array>

namespace RocketSim {
	struct BallPredTracker;
}

namespace RLGC {
	// Padded player slots, boost timers, and five ball-prediction samples.
	class GalactusOBS : public ObsBuilder {
	public:
		static constexpr size_t BALL_PRED_SAMPLE_COUNT = 5;
		using ExternalBallPredSamples = std::array<PhysState, BALL_PRED_SAMPLE_COUNT>;

		constexpr static float
			POS_COEF = 1 / 5000.f,
			VEL_COEF = 1 / 2300.f,
			ANG_VEL_COEF = 1 / 3.f;

		int maxPlayers; // Max players per team

		// Used when predictions come from a simulation arena.
		RocketSim::BallPredTracker* ballPredTracker = nullptr;

		uint64_t lastPredUpdateTick = 0;  // Last sim tick when ball prediction was refreshed
		bool hasExternalBallPredSamples = false;
		ExternalBallPredSamples externalBallPredSamples = {};

		GalactusOBS(int maxPlayers = 3) : maxPlayers(maxPlayers) {}

		~GalactusOBS();

		virtual void Reset(const GameState& initialState) override;
		void SetExternalBallPredSamples(const ExternalBallPredSamples& samples);
		void ClearExternalBallPredSamples();
		const std::vector<BallState>* GetPredictionData(const GameState& state, bool updateIfStale, bool forceUpdate = false);
		virtual void AddPlayerToObs(FList& obs, const Player& player, bool inv, const PhysState& ball);
		virtual void AddBallPredToObs(FList& obs, const GameState& state, bool inv);
		void AddPlayerToObs(float*& obs, const Player& player, bool inv, const PhysState& ball);
		void AddBallPredToObs(float*& obs, const GameState& state, bool inv);

		virtual FList BuildObs(const Player& player, const GameState& state) override;
		virtual void BuildObsInto(float* obs, const Player& player, const GameState& state) override;
	};
}
