package types

import "math"

type Config struct {
	Network struct {
		Nodes    int     `toml:"nodes"`
		SpaceMin float64 `toml:"space_min"`
		SpaceMax float64 `toml:"space_max"`
		GpsErr   float64 `toml:"gps_err"`
		DistErr  float64 `toml:"dist_err"`
	} `toml:"network"`

	Hyperparams struct {
		Alpha        float64 `toml:"alpha"`
		Eps          float64 `toml:"eps"`
		MaxIter      int     `toml:"max_iter"`
		EkfIter      int     `toml:"ekf_iter"`
		AnchorsCount int     `toml:"anchors_count"`

		Q float64 `toml:"q"`
		R float64 `toml:"r"`
	} `toml:"hyperparams"`

	Topology struct {
		K int `toml:"k"`
	} `toml:"topology"`

	Dynamics struct {
		Ticks           int     `toml:"ticks"`
		Dt              float64 `toml:"dt"`
		SwarmSpeed      float64 `toml:"swarm_speed"`
		GrowthRate      float64 `toml:"growth_rate"`
		GdItersPerTick  int     `toml:"gd_iters_per_tick"`
		EkfItersPerTick int     `toml:"ekf_iters_per_tick"`
	} `toml:"dynamics"`
}

type Measurement struct {
	From, To int
	Value    float64
}

// Point описывает вектор
type Point struct {
	X, Y, Z float64
}

type Velocity struct {
	X, Y, Z float64
}

// Node описывает объект сети
type Node struct {
	ID            int
	InitialCoord  Point   // текущий зашумленный якорь для регуляризации
	CurrentCoord  Point   // результат GD+EKF
	RealCoord     Point   // истинное положение
	BelievedCoord Point   // то, что узел думает о себе (дрейфует со временем)
	Uncertainty   float64 // текущее σ ошибки BelievedCoord относительно RealCoord
	TickCounter   int     // счётчик тиков для отслеживания времени в квадратичной модели
	IsAnchor      bool
}

// Distance считает евклидово расстояние между двумя точками
func Distance(p1, p2 Point) float64 {
	return math.Sqrt(math.Pow(p1.X-p2.X, 2) + math.Pow(p1.Y-p2.Y, 2) + math.Pow(p1.Z-p2.Z, 2))
}
