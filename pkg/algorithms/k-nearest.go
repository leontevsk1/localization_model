package algorithms

import (
	"fmt"
	"sort"

	"vostok/pkg/types"
)

// BuildMeasurements строит асимметричный список направленных измерений.
// Для каждого подвижного узла i берутся k ближайших целей по distances[i][*].
// Анкеры не инициируют измерения (пропускаются как From).
func BuildMeasurements(nodes []*types.Node, distances [][]float64, k int) []types.Measurement {
	n := len(nodes)
	var measurements []types.Measurement

	for i := 0; i < n; i++ {
		if nodes[i].IsAnchor {
			continue
		}

		// Индексы всех узлов, кроме i
		candidates := make([]int, 0, n-1)
		for j := 0; j < n; j++ {
			if i != j {
				candidates = append(candidates, j)
			}
		}

		// Сортировка по расстоянию
		sort.Slice(candidates, func(a, b int) bool {
			return distances[i][candidates[a]] < distances[i][candidates[b]]
		})

		// Берём первые k (или всех, если k > n-1)
		limit := k
		if limit > len(candidates) {
			limit = len(candidates)
		}

		for j := 0; j < limit; j++ {
			target := candidates[j]
			measurements = append(measurements, types.Measurement{
				From:  i,
				To:    target,
				Value: distances[i][target],
			})
		}
	}

	return measurements
}

// CheckConnectivity проверяет, есть ли путь от каждого подвижного узла до какого-либо анкера
// в направленном графе исходящих измерений. Выводит предупреждение, если связность нарушена.
func CheckConnectivity(nodes []*types.Node, measurements []types.Measurement) {
	n := len(nodes)

	// Построение графа: для каждого узла - список узлов, на которые указывают исходящие измерения
	graph := make([][]int, n)
	for _, m := range measurements {
		graph[m.From] = append(graph[m.From], m.To)
	}

	// Для каждого подвижного узла проверяем BFS до анкера
	for i := 0; i < n; i++ {
		if nodes[i].IsAnchor {
			continue
		}

		visited := make([]bool, n)
		queue := []int{i}
		visited[i] = true
		found := false

		for len(queue) > 0 && !found {
			u := queue[0]
			queue = queue[1:]

			if nodes[u].IsAnchor {
				found = true
				break
			}

			for _, v := range graph[u] {
				if !visited[v] {
					visited[v] = true
					queue = append(queue, v)
					if nodes[v].IsAnchor {
						found = true
						break
					}
				}
			}
		}

		if !found {
			fmt.Printf("Предупреждение: узел %d не имеет пути к анкеру в направленном графе\n", i)
		}
	}
}
