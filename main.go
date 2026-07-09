package main

import (
	"flag"
	"fmt"
	"math"
	"math/rand"
	"time"
)

func main() {
	configPath := flag.String("config", "", "Путь к TOML-файлу конфигурации сценария")

	// Стандартные CLI флаги (как значения по умолчанию)
	numNodes := flag.Int("nodes", 4, "Количество объектов (узлов) в сети")
	spaceMin := flag.Float64("space-min", 0.0, "Минимальная граница пространства")
	spaceMax := flag.Float64("space-max", 100.0, "Максимальная граница пространства")
	gpsErr := flag.Float64("gps-err", 3.0, "Максимальная абсолютная ошибка GPS (мнимых координат)")
	distErr := flag.Float64("dist-err", 0.5, "Максимальная абсолютная ошибка дальномеров")

	alpha := flag.Float64("alpha", 0.02, "Скорость обучения градиентного спуска")
	lambda := flag.Float64("lambda", 0.15, "Вес мягкой регуляризации")
	epsilon := flag.Float64("eps", 1e-5, "Критерий останова градиентного спуска")
	maxIter := flag.Int("max-iter", 2000, "Лимит итераций градиентного спуска")
	ekfIter := flag.Int("ekf-iter", 5, "Количество итераций EKF")
	ekfQFlag := flag.Float64("q", 0.001, "Шум процесса EKF")
	ekfRFlag := flag.Float64("r", 0.1, "Шум измерений EKF")
	anchorsCountFlag := flag.Int("anchors", 0, "Количество анкерных узлов с точными координатами")
	flag.Parse()

	var (
		n int
		sMin, sMax, gErr, dErr float64
		gAlpha, gLambda, gEps float64
		gMaxIter, eIter int
		anchorsCount int

		ekfQ float64
		ekfR float64
	)
	// Если передан конфиг — читаем его, иначе берем CLI флаги
	if *configPath != "" {
		fmt.Printf("Загрузка сценария из файла: %s\n", *configPath)
		cfg, err := LoadConfigFromFile(*configPath)
		if err != nil {
			fmt.Printf("Ошибка загрузки конфигурации: %v\n", err)
			return
		}
		n = cfg.Network.Nodes
		sMin = cfg.Network.SpaceMin
		sMax = cfg.Network.SpaceMax
		gErr = cfg.Network.GpsErr
		dErr = cfg.Network.DistErr
		anchorsCount = cfg.Hyperparams.AnchorsCount
		gAlpha = cfg.Hyperparams.Alpha
		gLambda = cfg.Hyperparams.Lambda
		gEps = cfg.Hyperparams.Eps
		gMaxIter = cfg.Hyperparams.MaxIter
		eIter = cfg.Hyperparams.EkfIter
		ekfQ = cfg.Hyperparams.Q
		ekfR = cfg.Hyperparams.R
	} else {
		n = *numNodes
		sMin = *spaceMin
		sMax = *spaceMax
		gErr = *gpsErr
		dErr = *distErr
		anchorsCount = *anchorsCountFlag
		gAlpha = *alpha
		gLambda = *lambda
		gEps = *epsilon
		gMaxIter = *maxIter
		eIter = *ekfIter
		ekfQ = *ekfQFlag
		ekfR = *ekfRFlag
	}

	if n < 3 {
		fmt.Println("Ошибка: требуется минимум 3 узла.")
		return
	}

	rng := rand.New(rand.NewSource(time.Now().UnixNano()))

	fmt.Printf("--- Выполнение сценария ---\n")
	fmt.Printf("Узлов: %d | Пространство: [%.1f, %.1f] | Шум GPS: ±%.2f м | Шум дальномеров: ±%.2f м\n\n",
		n, sMin, sMax, gErr, dErr)

	nodes, realCoords, distances := buildMap(n, sMin, sMax, gErr, dErr, anchorsCount, rng)

	// [Выполнение алгоритмов]
	fmt.Println("Состояние ДО оптимизации:")
	printGradientMetrics(nodes, realCoords, distances)

	fmt.Println("\n--- Запуск Градиентного спуска ---")
	RunGradientDescent(nodes, distances, gAlpha, gLambda, gEps, gMaxIter)
	minDist := math.Inf(1)
	for i := 0; i < n; i++ {
		for j := i + 1; j < n; j++ {
			d := Distance(nodes[i].CurrentCoord, nodes[j].CurrentCoord)
			if d < minDist {
				minDist = d
			}
		}
	}
	fmt.Printf("Минимальное расстояние между узлами перед EKF: %.6f\n", minDist)
	fmt.Println("\n--- Запуск Расширенного фильтра Калмана (EKF) ---")
	RunEKF(nodes, distances, eIter, ekfQ, ekfR)

	fmt.Println("\nСостояние ПОСЛЕ оптимизации:")
	printGradientMetrics(nodes, realCoords, distances)

	movable := make([]int, 0, len(nodes))
	for i, node := range nodes {
		if !node.IsAnchor {
			movable = append(movable, i)
		}
	}

	if err := AnalyzeEKFHistory("ekf_history.csv", nodes, movable, realCoords, distances); err != nil {
		fmt.Printf("Ошибка при анализе статистики: %v\n", err)
	}
}
