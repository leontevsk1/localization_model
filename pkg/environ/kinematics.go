package environ

import (
	"math/rand"

	"vostok/pkg/types"
)

// NodeMovement пересчитывает координаты узла на основе вектора скорости.
// Обновляет RealCoord узла (истинное положение).
func NodeMovement(node *types.Node, velocity types.Velocity, dt float64) {
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
	// Приращение e(t) - e(t-1) = k*(2t-1), телескопически даёт e(t) = d + k*t^2
	delta := growthRate * (2*t - 1)
	node.Uncertainty += delta

	// Размытие BelievedCoord на величину приращения ошибки за тик,
	// чтобы суммарный дрейф следовал квадратичному закону, а не обгонял его
	dx := (rng.Float64()*2 - 1.0) * delta
	dy := (rng.Float64()*2 - 1.0) * delta
	dz := (rng.Float64()*2 - 1.0) * delta

	node.BelievedCoord.X += dx
	node.BelievedCoord.Y += dy
	node.BelievedCoord.Z += dz
}

// StepSwarmMotion сдвигает весь рой по общему командному вектору скорости.
// RealCoord — истинное исполнение команды. BelievedCoord и CurrentCoord
// сдвигаются на ту же величину: счисление пути по известной команде
// (predict-шаг перед коррекцией дальномерами). Анкеры знают себя точно.
func StepSwarmMotion(nodes []*types.Node, velocity types.Velocity, dt float64) {
	for _, node := range nodes {
		NodeMovement(node, velocity, dt)

		if node.IsAnchor {
			node.InitialCoord = node.RealCoord
			node.CurrentCoord = node.RealCoord
			node.BelievedCoord = node.RealCoord
			continue
		}

		node.BelievedCoord.X += velocity.X * dt
		node.BelievedCoord.Y += velocity.Y * dt
		node.BelievedCoord.Z += velocity.Z * dt

		node.CurrentCoord.X += velocity.X * dt
		node.CurrentCoord.Y += velocity.Y * dt
		node.CurrentCoord.Z += velocity.Z * dt
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
