package environ

import (
	"math/rand"

	"vostok/pkg/types"
)

// NodeMovement пересчитывает координаты узла на основе вектора скорости.
// Обновляет RealCoord узла (истинное положение).
func NodeMovement(node *types.Node, velocity types.Velocity, dt float64) {
	if node.IsAnchor {
		return
	}
	node.RealCoord.X += velocity.X * dt
	node.RealCoord.Y += velocity.Y * dt
	node.RealCoord.Z += velocity.Z * dt
}

// ErrorShift обновляет Uncertainty узла по квадратичному закону роста ошибки.
// Модель: e(t) = d + k*t^2, где d — начальная ошибка GPS, k — коэффициент качества датчика.
// Эта формула физически корректнее: ошибка ускорения требует двойного интегрирования.
func ErrorShift(node *types.Node, growthRate float64, rng *rand.Rand) {
	if node.IsAnchor {
		return
	}

	node.TickCounter++
	t := float64(node.TickCounter)
	// e(t) = initialError + growthRate * t^2
	newUncertainty := node.Uncertainty + growthRate*t*t
	node.Uncertainty = newUncertainty

	// Размытие BelievedCoord на один шаг случайного блуждания с текущим σ
	dx := (rng.Float64()*2 - 1.0) * node.Uncertainty
	dy := (rng.Float64()*2 - 1.0) * node.Uncertainty
	dz := (rng.Float64()*2 - 1.0) * node.Uncertainty

	node.BelievedCoord.X += dx
	node.BelievedCoord.Y += dy
	node.BelievedCoord.Z += dz
}

// StepTrueMotion сдвигает RealCoord всех неанкерных узлов по случайному блужданию
// с шагом motionSigma (типичный сценарий броуновского движения объектов).
func StepTrueMotion(nodes []*types.Node, motionSigma float64, rng *rand.Rand) {
	for _, node := range nodes {
		if node.IsAnchor {
			continue
		}

		dx := (rng.Float64()*2 - 1.0) * motionSigma
		dy := (rng.Float64()*2 - 1.0) * motionSigma
		dz := (rng.Float64()*2 - 1.0) * motionSigma

		node.RealCoord.X += dx
		node.RealCoord.Y += dy
		node.RealCoord.Z += dz
	}
}

// GrowUncertainty обновляет Uncertainty всех неанкерных узлов и размывает их BelievedCoord.
func GrowUncertainty(nodes []*types.Node, growthRate float64, rng *rand.Rand) {
	for _, node := range nodes {
		ErrorShift(node, growthRate, rng)
	}
}

// RefreshInitialCoord переносит BelievedCoord в InitialCoord перед очередным
// тактом оптимизации градиентным спуском (новый "зашумленный якорь" для регуляризации).
func RefreshInitialCoord(nodes []*types.Node) {
	for _, node := range nodes {
		if node.IsAnchor {
			continue
		}
		node.InitialCoord = node.BelievedCoord
	}
}
