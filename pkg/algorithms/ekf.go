package algorithms

import (
	"fmt"
	"math"

	"gonum.org/v1/gonum/mat"

	"vostok/pkg/io"
	"vostok/pkg/types"
)

// Анкерные узлы (node.IsAnchor == true) полностью исключены из вектора состояния: их координаты считаются точными и используются как неподвижные ориентиры при вычислении измерений и Якобиана для остальных узлов.
func RunEKF(nodes []*types.Node, measurements []types.Measurement, iterations int, q, r float64) {
	n := len(nodes)

	// Список подвижных узлов
	movable := make([]int, 0, n)
	nodeToState := make([]int, n)
	for i := range nodeToState {
		nodeToState[i] = -1 // -1 означает "это анкер, в состоянии его нет
	}
	for i, node := range nodes {
		if !node.IsAnchor {
			nodeToState[i] = len(movable)
			movable = append(movable, i)
		}
	}
	stateSize := len(movable) * 3

	if stateSize == 0 {
		fmt.Println("EKF пропущен: все узлы являются анкерами, оптимизировать нечего.")
		return
	}

	// Хранилище истории
	var ekfHistory [][]float64

	numMeas := len(measurements)

	// Инициализация вектора состояния X — только по подвижным узлам
	xData := make([]float64, stateSize)
	for k, nodeIdx := range movable {
		xData[k*3+0] = nodes[nodeIdx].CurrentCoord.X
		xData[k*3+1] = nodes[nodeIdx].CurrentCoord.Y
		xData[k*3+2] = nodes[nodeIdx].CurrentCoord.Z
	}
	X := mat.NewVecDense(stateSize, xData)

	// Инициализация ковариационной матрицы P
	P := mat.NewDense(stateSize, stateSize, nil)
	for i := 0; i < stateSize; i++ {
		P.Set(i, i, 1.0) // Начальная неопределенность
	}

	// Матрицы шума процесса Q и шума измерений R
	Q := mat.NewDense(stateSize, stateSize, nil)
	for i := 0; i < stateSize; i++ {
		Q.Set(i, i, q)
	}

	R := mat.NewDense(numMeas, numMeas, nil)
	for i := 0; i < numMeas; i++ {
		R.Set(i, i, r)
	}

	// Предварительное выделение памяти под рабочие матрицы
	H := mat.NewDense(numMeas, stateSize, nil) // Якобиан
	Z := mat.NewVecDense(numMeas, nil)         // Реальные измерения
	Zcalc := mat.NewVecDense(numMeas, nil)     // Расчетные измерения
	Y := mat.NewVecDense(numMeas, nil)         // Инновация (невязка)
	PHt := mat.NewDense(stateSize, numMeas, nil)
	S := mat.NewDense(numMeas, numMeas, nil)
	Sinv := mat.NewDense(numMeas, numMeas, nil) // Обратная матрица инноваций
	K := mat.NewDense(stateSize, numMeas, nil)  // Коэффициент Калмана

	I := mat.NewDense(stateSize, stateSize, nil) // Единичная матрица
	for i := 0; i < stateSize; i++ {
		I.Set(i, i, 1.0)
	}

	KX := mat.NewVecDense(stateSize, nil)
	KH := mat.NewDense(stateSize, stateSize, nil)
	IKH := mat.NewDense(stateSize, stateSize, nil)

	// Вспомогательная функция: получить текущие координаты узла — из вектора состояния X, если узел подвижен, либо из nodes[], если это анкер.
	coordOf := func(nodeIdx int) (x, y, z float64) {
		if s := nodeToState[nodeIdx]; s != -1 {
			return X.AtVec(s*3 + 0), X.AtVec(s*3 + 1), X.AtVec(s*3 + 2)
		}
		c := nodes[nodeIdx].CurrentCoord
		return c.X, c.Y, c.Z
	}

	// Основной цикл EKF
	for iter := 0; iter < iterations; iter++ {
		// ЭТАП ПРЕДСКАЗАНИЯ
		P.Add(P, Q)

		// ЭТАП КОРРЕКЦИИ
		for measIdx, m := range measurements {
			i, j := m.From, m.To
			xi, yi, zi := coordOf(i)
			xj, yj, zj := coordOf(j)

			dx, dy, dz := xi-xj, yi-yj, zi-zj
			dCalc := math.Sqrt(dx*dx + dy*dy + dz*dz)
			if dCalc == 0 {
				dCalc = 1e-6
			}

			Z.SetVec(measIdx, m.Value)
			Zcalc.SetVec(measIdx, dCalc)

			if si := nodeToState[i]; si != -1 {
				H.Set(measIdx, si*3+0, dx/dCalc)
				H.Set(measIdx, si*3+1, dy/dCalc)
				H.Set(measIdx, si*3+2, dz/dCalc)
			}
			if sj := nodeToState[j]; sj != -1 {
				H.Set(measIdx, sj*3+0, -dx/dCalc)
				H.Set(measIdx, sj*3+1, -dy/dCalc)
				H.Set(measIdx, sj*3+2, -dz/dCalc)
			}
		}

		// Вычисление инновации: Y = Z - Zcalc
		Y.SubVec(Z, Zcalc)

		// Вычисление ковариации инновации: S = H * P * H^T + R
		PHt.Mul(P, H.T())
		S.Mul(H, PHt)
		S.Add(S, R)

		// Обращение матрицы S
		err := Sinv.Inverse(S)
		if err != nil {
			fmt.Printf("EKF прерван на итерации %d: ошибка обращения матрицы S: %v\n", iter, err)
			break
		}

		// Вычисление коэффициента Калмана: K = P * H^T * Sinv
		K.Mul(PHt, Sinv)

		// Коррекция состояния: X = X + K * Y
		KX.MulVec(K, Y)
		X.AddVec(X, KX)

		// Обновление ковариации: P = (I - K * H) * P
		KH.Mul(K, H)
		IKH.Sub(I, KH)
		P.Mul(IKH, P)

		// Сохраняем текущее состояние подвижных узлов в историю
		historyRow := make([]float64, 0, 1+stateSize)
		historyRow = append(historyRow, float64(iter))
		for i := 0; i < stateSize; i++ {
			historyRow = append(historyRow, X.AtVec(i))
		}
		ekfHistory = append(ekfHistory, historyRow)
	}

	// Запись вычисленных координат обратно в структуру узлов (только для подвижных)
	for k, nodeIdx := range movable {
		nodes[nodeIdx].CurrentCoord.X = X.AtVec(k*3 + 0)
		nodes[nodeIdx].CurrentCoord.Y = X.AtVec(k*3 + 1)
		nodes[nodeIdx].CurrentCoord.Z = X.AtVec(k*3 + 2)
	}

	// Сохранение лога в корень проекта
	if err := io.SaveEKFHistory("ekf_history.csv", ekfHistory, len(movable)); err != nil {
		fmt.Printf("Ошибка при записи лога EKF: %v\n", err)
	} else {
		fmt.Println("Файл ekf_history.csv успешно сгенерирован.")
	}
}
