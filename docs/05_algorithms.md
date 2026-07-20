# Формальные алгоритмы модулей

---

## `pkg/types/types.go`

### `Distance(p1, p2 Point) float64`

```
1.  dx := p1.X - p2.X
2.  dy := p1.Y - p2.Y
3.  dz := p1.Z - p2.Z
4.  RETURN sqrt(dx*dx + dy*dy + dz*dz)
```

---

## `pkg/environ/env.go`

### `BuildMap(n, sMin, sMax, gErr, dErr, anchorsCount, rng) (nodes, realCoords, distances)`

```
 1.  realCoords := массив Point длины n
 2.  nodes := массив указателей Node длины n
 3.  i := 0
 4.  ЕСЛИ i >= n, ПЕРЕЙТИ К 17
 5.      rX := sMin + rng.Float64() * (sMax - sMin)
 6.      rY := sMin + rng.Float64() * (sMax - sMin)
 7.      rZ := sMin + rng.Float64() * (sMax - sMin)
 8.      realCoords[i] := Point{rX, rY, rZ}
 9.      isAnchor := (i < anchorsCount)
10.      dx := (rng.Float64()*2 - 1) * gErr
11.      dy := (rng.Float64()*2 - 1) * gErr
12.      dz := (rng.Float64()*2 - 1) * gErr
13.      ЕСЛИ isAnchor:
13a.         nodes[i] := Node{ID: i, InitialCoord: realCoords[i], CurrentCoord: realCoords[i],
                              RealCoord: realCoords[i], BelievedCoord: realCoords[i],
                              Uncertainty: 0, TickCounter: 0, IsAnchor: true}
13b.         i := i + 1; ПЕРЕЙТИ К 4
14.      ИНАЧЕ:
14a.         noisyCoord := Point{rX+dx, rY+dy, rZ+dz}
14b.         nodes[i] := Node{ID: i, InitialCoord: noisyCoord, CurrentCoord: noisyCoord,
                              RealCoord: realCoords[i], BelievedCoord: noisyCoord,
                              Uncertainty: gErr, TickCounter: 0, IsAnchor: false}
15.      i := i + 1
16.      ПЕРЕЙТИ К 4
17.  distances := buildDistanceMatrix(realCoords, dErr, rng)
18.  RETURN (nodes, realCoords, distances)
```

### `RecomputeDistances(nodes, distErr, rng) [][]float64`

```
1.  realCoords := массив Point длины len(nodes)
2.  ДЛЯ i ОТ 0 ДО len(nodes)-1:
2a.     realCoords[i] := nodes[i].RealCoord
3.  RETURN buildDistanceMatrix(realCoords, distErr, rng)
```

### `buildDistanceMatrix(coords, distErr, rng) [][]float64`

```
 1.  n := len(coords)
 2.  distances := матрица n×n, заполненная нулями
 3.  i := 0
 4.  ЕСЛИ i >= n, ПЕРЕЙТИ К 15
 5.      j := i + 1
 6.      ЕСЛИ j >= n, ПЕРЕЙТИ К 13
 7.          dx := coords[i].X - coords[j].X; dy := coords[i].Y - coords[j].Y; dz := coords[i].Z - coords[j].Z
 8.          trueDist := sqrt(dx*dx + dy*dy + dz*dz)
 9.          noise := (rng.Float64()*2 - 1) * distErr
10.          measDist := trueDist + noise
11.          ЕСЛИ measDist < 0: measDist := 0.001
12.          distances[i][j] := measDist; distances[j][i] := measDist
12a.         j := j + 1; ПЕРЕЙТИ К 6
13.      i := i + 1
14.      ПЕРЕЙТИ К 4
15.  RETURN distances
```

---

## `pkg/environ/kinematics.go`

### `NodeMovement(node, velocity, dt)`

```
1.  node.RealCoord.X := node.RealCoord.X + velocity.X * dt
2.  node.RealCoord.Y := node.RealCoord.Y + velocity.Y * dt
3.  node.RealCoord.Z := node.RealCoord.Z + velocity.Z * dt
4.  RETURN
```

### `ErrorShift(node, growthRate, rng)`

```
1.  ЕСЛИ node.IsAnchor: RETURN
2.  node.TickCounter := node.TickCounter + 1
3.  t := float64(node.TickCounter)
4.  delta := growthRate * (2*t - 1)
5.  node.Uncertainty := node.Uncertainty + delta
6.  dx := (rng.Float64()*2 - 1) * delta
7.  dy := (rng.Float64()*2 - 1) * delta
8.  dz := (rng.Float64()*2 - 1) * delta
9.  node.BelievedCoord.X := node.BelievedCoord.X + dx
10. node.BelievedCoord.Y := node.BelievedCoord.Y + dy
11. node.BelievedCoord.Z := node.BelievedCoord.Z + dz
12. RETURN
```

### `StepSwarmMotion(nodes, velocity, dt)`

```
1.  i := 0
2.  ЕСЛИ i >= len(nodes), ПЕРЕЙТИ К 10
3.      NodeMovement(nodes[i], velocity, dt)
4.      ЕСЛИ nodes[i].IsAnchor:
4a.         nodes[i].InitialCoord := nodes[i].RealCoord
4b.         nodes[i].CurrentCoord := nodes[i].RealCoord
4c.         nodes[i].BelievedCoord := nodes[i].RealCoord
4d.         i := i + 1; ПЕРЕЙТИ К 2
5.      nodes[i].BelievedCoord.X += velocity.X * dt
6.      nodes[i].BelievedCoord.Y += velocity.Y * dt
7.      nodes[i].BelievedCoord.Z += velocity.Z * dt
8.      nodes[i].CurrentCoord.X += velocity.X * dt
8a.     nodes[i].CurrentCoord.Y += velocity.Y * dt
8b.     nodes[i].CurrentCoord.Z += velocity.Z * dt
9.      i := i + 1
9a.     ПЕРЕЙТИ К 2
10. RETURN
```

### `GrowUncertainty(nodes, growthRate, rng)`

```
1.  ДЛЯ КАЖДОГО node В nodes: ErrorShift(node, growthRate, rng)
2.  RETURN
```

### `RefreshInitialCoord(nodes)`

```
1.  ДЛЯ КАЖДОГО node В nodes:
1a.     ЕСЛИ node.IsAnchor: ПРОПУСТИТЬ
1b.     node.InitialCoord := node.BelievedCoord
2.  RETURN
```

---

## `pkg/algorithms/k-nearest.go`

### `BuildMeasurements(nodes, distances, k) []Measurement`

```
 1.  n := len(nodes)
 2.  measurements := пустой список
 3.  i := 0
 4.  ЕСЛИ i >= n, ПЕРЕЙТИ К 15
 5.      ЕСЛИ nodes[i].IsAnchor: i := i+1; ПЕРЕЙТИ К 4
 6.      candidates := список всех j ≠ i, j ОТ 0 ДО n-1
 7.      ОТСОРТИРОВАТЬ candidates ПО ВОЗРАСТАНИЮ distances[i][candidates[·]]
 8.      limit := k
 9.      ЕСЛИ limit > len(candidates): limit := len(candidates)
10.      c := 0
11.      ЕСЛИ c >= limit, ПЕРЕЙТИ К 14
12.          target := candidates[c]
13.          ДОБАВИТЬ Measurement{From: i, To: target, Value: distances[i][target]} В measurements
13a.        c := c+1; ПЕРЕЙТИ К 11
14.      i := i + 1
14a.     ПЕРЕЙТИ К 4
15.  RETURN measurements
```

### `CheckConnectivity(nodes, measurements)`

```
 1.  n := len(nodes)
 2.  graph := массив n пустых списков
 3.  ДЛЯ КАЖДОГО m В measurements: ДОБАВИТЬ m.To В graph[m.From]
 4.  i := 0
 5.  ЕСЛИ i >= n, ПЕРЕЙТИ К 20
 6.      ЕСЛИ nodes[i].IsAnchor: i := i+1; ПЕРЕЙТИ К 5
 7.      visited := массив n флагов false
 8.      queue := [i]; visited[i] := true; found := false
 9.      ЕСЛИ queue пуст ИЛИ found = true, ПЕРЕЙТИ К 18
10.          u := queue[0]; queue := queue БЕЗ первого элемента
11.          ЕСЛИ nodes[u].IsAnchor: found := true; ПЕРЕЙТИ К 18
12.          ДЛЯ КАЖДОГО v В graph[u]:
13.              ЕСЛИ visited[v] = false:
14.                  visited[v] := true; ДОБАВИТЬ v В queue
15.                  ЕСЛИ nodes[v].IsAnchor: found := true; ПРЕРВАТЬ ЦИКЛ ПО v
16.          ПЕРЕЙТИ К 9
17.      (нет доп. действий)
18.      ЕСЛИ found = false: ВЫВЕСТИ "узел i не имеет пути к анкеру"
19.      i := i + 1; ПЕРЕЙТИ К 5
20.  RETURN
```

---

## `pkg/algorithms/gradient.go`

### `RunGradientDescent(nodes, measurements, alpha, distErr, epsilon, maxIter, appendLog) int`

```
 1.  n := len(nodes)
 2.  rangeVar := max(distErr*distErr, 1e-12)
 3.  priorWeight := массив n нулей
 4.  ДЛЯ КАЖДОГО i ОТ 0 ДО n-1:
 4a.     ЕСЛИ nodes[i].IsAnchor: ПРОПУСТИТЬ
 4b.     priorWeight[i] := rangeVar / max(nodes[i].Uncertainty², 1e-12)
 5.  ФУНКЦИЯ cost():
 5a.     c := 0
 5b.     ДЛЯ КАЖДОГО m В measurements: d := Distance(nodes[m.From].CurrentCoord, nodes[m.To].CurrentCoord); r := d - m.Value; c += r*r
 5c.     ДЛЯ i ОТ 0 ДО n-1: ЕСЛИ НЕ nodes[i].IsAnchor: c += priorWeight[i] * |CurrentCoord[i] - InitialCoord[i]|²
 5d.     RETURN c
 6.  logNode := индекс первого i, где nodes[i].IsAnchor = false (0, если такого нет)
 7.  csvData := [[заголовок: Iteration, Node{logNode}_X/Y/Z, RealX/Y/Z]]
 8.  alphaCur := alpha
 9.  prevCost := cost()
10.  snapshot := массив n точек
11.  stoppedAt := maxIter
12.  iter := 0
13.  ЕСЛИ iter >= maxIter, ПЕРЕЙТИ К 35
14.      gradients := массив n нулевых точек
15.      ДЛЯ КАЖДОГО m В measurements:
15a.        i, j := m.From, m.To
15b.        dCalc := Distance(CurrentCoord[i], CurrentCoord[j]); ЕСЛИ dCalc < 1e-9: dCalc := 1e-9
15c.        errRatio := 1 - m.Value/dCalc
15d.        (ux,uy,uz) := CurrentCoord[i] - CurrentCoord[j]
15e.        ЕСЛИ НЕ nodes[i].IsAnchor: gradients[i] += errRatio*(ux,uy,uz)
15f.        ЕСЛИ НЕ nodes[j].IsAnchor: gradients[j] -= errRatio*(ux,uy,uz)
16.      ДЛЯ i ОТ 0 ДО n-1:
16a.        ЕСЛИ nodes[i].IsAnchor: ПРОПУСТИТЬ
16b.        gradients[i] += priorWeight[i] * (CurrentCoord[i] - InitialCoord[i])
17.      ДЛЯ i ОТ 0 ДО n-1: snapshot[i] := CurrentCoord[i]
18.      maxShift := 0
19.      ДЛЯ i ОТ 0 ДО n-1:
19a.        ЕСЛИ nodes[i].IsAnchor: ПРОПУСТИТЬ
19b.        shift := alphaCur * gradients[i]
19c.        CurrentCoord[i] := CurrentCoord[i] - shift
19d.        shiftMag := |shift|; ЕСЛИ shiftMag > maxShift: maxShift := shiftMag
20.      newCost := cost()
21.      ЕСЛИ newCost — NaN ИЛИ Inf ИЛИ newCost > prevCost:
21a.        ДЛЯ i ОТ 0 ДО n-1: CurrentCoord[i] := snapshot[i]
21b.        alphaCur := alphaCur * 0.5
21c.        ЕСЛИ alphaCur < alpha*1e-12:
21d.            ВЫВЕСТИ "шаг выродился на итерации iter"; stoppedAt := iter; ПЕРЕЙТИ К 35
21e.        ПЕРЕЙТИ К 34 (следующая итерация, без записи в лог)
22.      prevCost := newCost
23.      alphaCur := min(alphaCur*1.1, alpha)
24.      ДОБАВИТЬ В csvData: [iter, CurrentCoord[logNode].X/Y/Z, RealCoord[logNode].X/Y/Z]
25.      ЕСЛИ maxShift < epsilon: stoppedAt := iter; ПЕРЕЙТИ К 35
34.     iter := iter + 1
34a.    ПЕРЕЙТИ К 13
35.  WriteHistoryToCSV("mod1_history.csv", csvData, appendLog)
36.  RETURN stoppedAt
```

---

## `pkg/algorithms/ekf.go`

### `RunEKF(nodes, measurements, iterations, q, r, P, appendLog) *mat.Dense`

```
 1.  n := len(nodes)
 2.  movable := пустой список; nodeToState := массив n значений -1
 3.  ДЛЯ i ОТ 0 ДО n-1: ЕСЛИ НЕ nodes[i].IsAnchor: nodeToState[i] := len(movable); ДОБАВИТЬ i В movable
 4.  stateSize := len(movable) * 3
 5.  ЕСЛИ stateSize = 0: ВЫВЕСТИ "все узлы анкеры"; RETURN nil
 6.  numRange := len(measurements); numMeas := numRange + stateSize
 7.  X := вектор длины stateSize, X[k*3+c] := CurrentCoord[movable[k]].{X,Y,Z}[c] ДЛЯ КАЖДОГО k, c
 8.  ЕСЛИ P = nil: P := единичная матрица stateSize×stateSize
 9.  Q := диагональная матрица stateSize×stateSize со значением q
10.  R := диагональная матрица numMeas×numMeas: R[i][i] := r ДЛЯ i < numRange;
         R[numRange+k*3+c][то же] := max(Uncertainty[movable[k]]², 1e-12) ДЛЯ КАЖДОГО k, c
11.  ВЫДЕЛИТЬ рабочие матрицы: H(numMeas×stateSize), Z, Zcalc, Y (векторы numMeas),
         PHt(stateSize×numMeas), S, Sinv(numMeas×numMeas), K(stateSize×numMeas),
         I(единичная stateSize×stateSize), KX(вектор stateSize), KH, IKH(stateSize×stateSize)
12.  ДЛЯ i ОТ 0 ДО stateSize-1: H[numRange+i][i] := 1  (постоянная часть якобиана для псевдо-GPS)
13.  ДЛЯ КАЖДОГО k, nodeIdx В movable: Z[numRange+k*3+{0,1,2}] := BelievedCoord[nodeIdx].{X,Y,Z}
14.  ФУНКЦИЯ coordOf(nodeIdx):
14a.     ЕСЛИ nodeToState[nodeIdx] ≠ -1: RETURN X[s*3+{0,1,2}], где s = nodeToState[nodeIdx]
14b.     ИНАЧЕ: RETURN nodes[nodeIdx].CurrentCoord
15.  iter := 0
16.  ЕСЛИ iter >= iterations, ПЕРЕЙТИ К 34
17.      P := P + Q                                    (этап предсказания)
18.      ДЛЯ КАЖДОГО (measIdx, m) В measurements:        (этап коррекции: дальности)
18a.        i, j := m.From, m.To
18b.        (xi,yi,zi) := coordOf(i); (xj,yj,zj) := coordOf(j)
18c.        (dx,dy,dz) := (xi-xj, yi-yj, zi-zj); dCalc := sqrt(dx²+dy²+dz²); ЕСЛИ dCalc=0: dCalc:=1e-6
18d.        Z[measIdx] := m.Value; Zcalc[measIdx] := dCalc
18e.        ЕСЛИ si := nodeToState[i] ≠ -1: H[measIdx][si*3+{0,1,2}] := (dx,dy,dz)/dCalc
18f.        ЕСЛИ sj := nodeToState[j] ≠ -1: H[measIdx][sj*3+{0,1,2}] := -(dx,dy,dz)/dCalc
19.      ДЛЯ i ОТ 0 ДО stateSize-1: Zcalc[numRange+i] := X[i]     (расчётные псевдо-GPS = состояние)
20.      Y := Z - Zcalc                                            (невязка)
21.      PHt := P * Hᵀ
22.      S := H*PHt + R
23.      Sinv := обратная(S)
24.      ЕСЛИ обращение НЕ удалось: ВЫВЕСТИ "ошибка обращения S на итерации iter"; ПЕРЕЙТИ К 34
25.      K := PHt * Sinv
26.      X := X + K*Y
27.      P := (I - K*H) * P
28.      historyRow := [iter] ++ X[0..stateSize-1] ++ RealCoord[movable[0]].{X,Y,Z}
29.      ДОБАВИТЬ historyRow В ekfHistory
33.     iter := iter + 1
33a.    ПЕРЕЙТИ К 16
34.  ДЛЯ КАЖДОГО k, nodeIdx В movable: CurrentCoord[nodeIdx].{X,Y,Z} := X[k*3+{0,1,2}]
35.  SaveEKFHistory("ekf_history.csv", ekfHistory, len(movable), appendLog)
36.  RETURN P
```

---

## `pkg/algorithms/kabsch.go`

### `KabschAlign(points, reference) []Point`

```
 1.  n := len(points)
 2.  ЕСЛИ n < 3: RETURN копия points (выравнивание невозможно/не нужно)
 3.  (pcX,pcY,pcZ) := среднее points; (rcX,rcY,rcZ) := среднее reference
 4.  H := матрица 3×3, нули
 5.  ДЛЯ i ОТ 0 ДО n-1:
 5a.     p := points[i] - (pcX,pcY,pcZ);  q := reference[i] - (rcX,rcY,rcZ)
 5b.     ДЛЯ r ОТ 0 ДО 2: ДЛЯ c ОТ 0 ДО 2: H[r][c] := H[r][c] + p[r]*q[c]
 6.  (U, V) := SVD(H)                              (H = U·Σ·Vᵀ)
 7.  ЕСЛИ разложение SVD не удалось: RETURN копия points
 8.  detSign := 1, ЕСЛИ det(V*Uᵀ) < 0: detSign := -1
 9.  D := diag(1, 1, detSign)
10.  R := V * D * Uᵀ                                 (оптимальный поворот без отражения)
11.  ДЛЯ i ОТ 0 ДО n-1:
11a.    p := points[i] - (pcX,pcY,pcZ)
11b.    aligned[i] := R*p + (rcX,rcY,rcZ)
12.  RETURN aligned
```

### `AlignMovableToReference(nodes, reference)`

```
1.  movable := индексы i, где nodes[i].IsAnchor = false
2.  points := CurrentCoord подвижных узлов; refs := reference по тем же индексам
3.  aligned := KabschAlign(points, refs)
4.  ДЛЯ КАЖДОГО k, idx В movable: nodes[idx].CurrentCoord := aligned[k]
5.  RETURN
```

---

## `pkg/analytics/metrics.go`

### `ComputeGlobalMetrics(nodes, realCoords, distances) (rmse, variance float64)`

```
 1.  n := len(nodes)
 2.  sumSqCoordDist := 0
 3.  ДЛЯ i ОТ 0 ДО n-1: sumSqCoordDist += |CurrentCoord[i] - realCoords[i]|²
 4.  rmseCoords := sqrt(sumSqCoordDist / n)
 5.  sumSqDistError := 0; count := 0
 6.  i := 0
 7.  ЕСЛИ i >= n, ПЕРЕЙТИ К 13
 8.      j := i+1
 9.      ЕСЛИ j >= n, ПЕРЕЙТИ К 12
10.          dCalc := Distance(CurrentCoord[i], CurrentCoord[j]); dMeas := distances[i][j]
10a.        sumSqDistError += (dCalc-dMeas)²; count += 1
10b.        j := j+1; ПЕРЕЙТИ К 9
11.      (нет доп. действий)
12.      i := i+1; ПЕРЕЙТИ К 7
13.  varianceDist := sumSqDistError / count
14.  RETURN (rmseCoords, varianceDist)
```

### `SummarizeEKFSpread(filename) (maxSpread, maxLabel, meanSpread float64, err error)`

```
 1.  ОТКРЫТЬ filename; ЕСЛИ ошибка: RETURN (0, "", 0, ошибка)
 2.  records := прочитать все строки CSV; ЕСЛИ ошибка: RETURN (0, "", 0, ошибка)
 3.  ЕСЛИ len(records) < 2: RETURN (0, "", 0, ошибка "недостаточно данных")
 4.  headers := records[0]; numCols := len(headers); numRows := len(records)-1
 5.  ЕСЛИ numCols < 2: RETURN (0, "", 0, ошибка "нет координатных колонок")
 6.  maxSpread := 0; maxLabel := headers[1]; sumSpread := 0; scannedCols := 0
 7.  c := 1
 8.  ЕСЛИ c >= numCols, ПЕРЕЙТИ К 18
 9.      ЕСЛИ headers[c] НАЧИНАЕТСЯ С "Real": c := c+1; ПЕРЕЙТИ К 8
10.      minVal := 0; maxVal := 0
11.      r := 0
12.      ЕСЛИ r >= numRows, ПЕРЕЙТИ К 16
13.          val := ПАРСИТЬ_FLOAT(records[r+1][c]); ЕСЛИ ошибка: RETURN (0,"",0, ошибка)
14.          ЕСЛИ r = 0: minVal := val; maxVal := val
15.          ИНАЧЕ: ЕСЛИ val < minVal: minVal := val; ЕСЛИ val > maxVal: maxVal := val
15a.        r := r+1; ПЕРЕЙТИ К 12
16.      spread := maxVal - minVal
16a.     sumSpread += spread; scannedCols += 1
16b.     ЕСЛИ spread > maxSpread: maxSpread := spread; maxLabel := headers[c]
17.      c := c+1; ПЕРЕЙТИ К 8
18.  ЕСЛИ scannedCols = 0: RETURN (0, "", 0, ошибка "нет координатных колонок")
19.  meanSpread := sumSpread / scannedCols
20.  RETURN (maxSpread, maxLabel, meanSpread, nil)
```

---

## `pkg/io/io.go`

### `LoadConfigFromFile(filepath) (*Config, error)`

```
1.  file := ПРОЧИТАТЬ_ФАЙЛ(filepath); ЕСЛИ ошибка: RETURN (nil, ошибка)
2.  cfg := Config{}
3.  РАЗОБРАТЬ TOML(file) В cfg; ЕСЛИ ошибка: RETURN (nil, ошибка)
4.  RETURN (&cfg, nil)
```

### `WriteHistoryToCSV(filename, recordFields, doAppend) error`

```
1.  ЕСЛИ doAppend: flags := СОЗДАТЬ|ЗАПИСЬ|ДОБАВЛЕНИЕ; rows := recordFields БЕЗ первой строки (заголовка)
2.  ИНАЧЕ: flags := СОЗДАТЬ|ЗАПИСЬ|УСЕЧЕНИЕ; rows := recordFields
3.  ОТКРЫТЬ filename С flags; ЕСЛИ ошибка: RETURN ошибка
4.  ДЛЯ КАЖДОЙ row В rows: ЗАПИСАТЬ_CSV(row); ЕСЛИ ошибка: RETURN ошибка
5.  СБРОСИТЬ_БУФЕР; ЗАКРЫТЬ ФАЙЛ
6.  RETURN nil
```

### `SaveEKFHistory(filename, history, nodeCount, doAppend) error`

```
 1.  ЕСЛИ doAppend: flags := СОЗДАТЬ|ЗАПИСЬ|ДОБАВЛЕНИЕ
 2.  ИНАЧЕ: flags := СОЗДАТЬ|ЗАПИСЬ|УСЕЧЕНИЕ
 3.  ОТКРЫТЬ filename С flags; ЕСЛИ ошибка: RETURN ошибка
 4.  ЕСЛИ НЕ doAppend:
 4a.     header := ["Iteration"]
 4b.     ДЛЯ i ОТ 0 ДО nodeCount-1: ДОБАВИТЬ "Node{i}_X", "Node{i}_Y", "Node{i}_Z" В header
 4c.     ДОБАВИТЬ "RealX", "RealY", "RealZ" В header
 4d.     ЗАПИСАТЬ_CSV(header); ЕСЛИ ошибка: RETURN ошибка
 5.  ДЛЯ КАЖДОЙ row В history:
 5a.     strRow := массив той же длины
 5b.     ДЛЯ i ОТ 0 ДО len(row)-1: ЕСЛИ i=0: strRow[i] := ФОРМАТ("%.0f", row[i]) ИНАЧЕ: strRow[i] := ФОРМАТ("%.6f", row[i])
 5c.     ЗАПИСАТЬ_CSV(strRow); ЕСЛИ ошибка: RETURN ошибка
 6.  СБРОСИТЬ_БУФЕР; ЗАКРЫТЬ ФАЙЛ
 7.  RETURN nil
```

---

## `cmd/main.go`

### `main()`

```
 1.  РАЗОБРАТЬ CLI-флаги (-config и остальные, см. docs/03_structure.md)
 2.  ЕСЛИ -config задан:
 2a.     cfg := LoadConfigFromFile(-config); ЕСЛИ ошибка: ВЫВЕСТИ И RETURN
 2b.     n, sMin, sMax, gErr, dErr, anchorsCount, gAlpha, gEps, gMaxIter, eIter, ekfQ, ekfR,
         topologyK, ticks, dt, swarmSpeed, growthRate, gdItersPerTick, ekfItersPerTick := ИЗ cfg
 3.  ИНАЧЕ: те же переменные := ИЗ CLI-флагов (значения по умолчанию)
 4.  ЕСЛИ n < 3: ВЫВЕСТИ "нужно минимум 3 узла"; RETURN
 5.  k := topologyK; ЕСЛИ CLI-флаг -k задан (>=0): k := флаг; ЕСЛИ k < 0: k := n-1
 6.  rng := новый ГПСЧ на основе текущего времени
 7.  ВЫВЕСТИ параметры сценария
 8.  (nodes, realCoords, distances) := environ.BuildMap(n, sMin, sMax, gErr, dErr, anchorsCount, rng)
 9.  velocity := случайный единичный вектор * swarmSpeed
10.  (rmseBefore, varBefore) := analytics.ComputeGlobalMetrics(nodes, realCoords, distances)
11.  measurements := algorithms.BuildMeasurements(nodes, distances, k)
12.  ЕСЛИ anchorsCount > 0: algorithms.CheckConnectivity(nodes, measurements)
13.  gdIters := algorithms.RunGradientDescent(nodes, measurements, gAlpha, dErr, gEps, gMaxIter, false)
14.  ekfP := algorithms.RunEKF(nodes, measurements, eIter, ekfQ, ekfR, nil, false)
15.  ЕСЛИ anchorsCount = 0:
15a.     believedRef := BelievedCoord всех узлов на данный момент
15b.     algorithms.AlignMovableToReference(nodes, believedRef)
16.  (rmseAfter, varAfter) := analytics.ComputeGlobalMetrics(nodes, realCoords, distances)
17.  ВЫВЕСТИ статистику статической коррекции (итерации, RMSE до/после, дисперсия до/после)
18.  analytics.SummarizeEKFSpread("ekf_history.csv") — ВЫВЕСТИ результат ИЛИ ошибку
19.  tickMetrics := [[заголовок: Tick, RawRMSE, FinalRMSE, ShapeRMSE, CompensationRatio]]
20.  tick := 0
21.  ЕСЛИ tick >= ticks, ПЕРЕЙТИ К 33
22.      environ.StepSwarmMotion(nodes, velocity, dt)
23.      environ.GrowUncertainty(nodes, growthRate, rng)
24.      environ.RefreshInitialCoord(nodes)
25.      distances := environ.RecomputeDistances(nodes, dErr, rng)
26.      predicted := CurrentCoord всех узлов на данный момент (снимок ДО переоптимизации)
27.      measurements := algorithms.BuildMeasurements(nodes, distances, k)
28.      algorithms.RunGradientDescent(nodes, measurements, gAlpha, dErr, gEps, gdItersPerTick, true)
29.      ekfP := algorithms.RunEKF(nodes, measurements, ekfItersPerTick, ekfQ, ekfR, ekfP, true)
30.      ЕСЛИ anchorsCount = 0: algorithms.AlignMovableToReference(nodes, predicted)
31.      rawRMSE := computeRawRMSE(nodes); finalRMSE := computeFinalRMSE(nodes); shapeRMSE := computeShapeRMSE(nodes)
31a.     compensationRatio := 0; ЕСЛИ rawRMSE > 0: compensationRatio := rawRMSE / finalRMSE
32.      ВЫВЕСТИ строку тика; ДОБАВИТЬ [tick, rawRMSE, finalRMSE, shapeRMSE, compensationRatio] В tickMetrics
32a.     tick := tick + 1; ПЕРЕЙТИ К 21
33.  ДЛЯ КАЖДОГО i: realCoords[i] := nodes[i].RealCoord   (обновление эталона после движения роя)
34.  (rmseFinal, varFinal) := analytics.ComputeGlobalMetrics(nodes, realCoords, distances)
35.  ВЫВЕСТИ итог динамики
36.  saveTickMetricsToCSV("tick_history.csv", tickMetrics)
37.  RETURN
```

### `computeRawRMSE(nodes) float64`

```
1.  sumSqDist := 0; count := 0
2.  ДЛЯ КАЖДОГО node В nodes:
2a.     ЕСЛИ node.IsAnchor: ПРОПУСТИТЬ
2b.     sumSqDist += |BelievedCoord - RealCoord|²; count += 1
3.  ЕСЛИ count = 0: RETURN 0
4.  RETURN sqrt(sumSqDist / count)
```

### `computeFinalRMSE(nodes) float64`

```
1.  Аналогично computeRawRMSE, но |CurrentCoord - RealCoord|² вместо |BelievedCoord - RealCoord|²
```

### `computeShapeRMSE(nodes) float64`

```
1.  current := CurrentCoord всех неанкерных узлов; real := RealCoord всех неанкерных узлов
2.  ЕСЛИ len(current) = 0: RETURN 0
3.  aligned := algorithms.KabschAlign(current, real)
4.  sumSqDist := 0
5.  ДЛЯ КАЖДОГО i: sumSqDist += |aligned[i] - real[i]|²
6.  RETURN sqrt(sumSqDist / len(aligned))
```

### `saveTickMetricsToCSV(filename, metrics) error`

```
1.  СОЗДАТЬ filename; ЕСЛИ ошибка: RETURN ошибка
2.  ДЛЯ КАЖДОЙ row В metrics:
2a.     strRow := row, КАЖДЫЙ ЭЛЕМЕНТ ПРЕОБРАЗОВАН В СТРОКУ
2b.     ЗАПИСАТЬ_CSV(strRow); ЕСЛИ ошибка: RETURN ошибка
3.  СБРОСИТЬ_БУФЕР; ЗАКРЫТЬ ФАЙЛ
4.  RETURN nil
```
