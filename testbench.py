#!/usr/bin/env python3
import os
import subprocess
import toml
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime
from pathlib import Path

CONFIG_DIR = "config"
MD_REPORT_PATH = "report.md"
GRADIENT_CSV = "mod1_history.csv"
EKF_CSV = "ekf_history.csv"
TICK_CSV = "tick_history.csv"
IMG_DIR = "report_images"


def get_git_commit():
    try:
        commit = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD']).decode('ascii').strip()
        return commit
    except Exception:
        return "Git-репозиторий не найден"


def run_go_simulation(config_path=None, cli_args=None):
    """Запускает симуляцию с конфигом или CLI аргументами."""
    # Логи GD/EKF накапливаются по тикам (append) — удаляем остатки
    # предыдущего прогона, чтобы график отражал только текущий запуск.
    for f in (GRADIENT_CSV, EKF_CSV, TICK_CSV):
        if os.path.exists(f):
            os.remove(f)

    cmd = ["go", "run", "./cmd/main.go"]

    if config_path:
        cmd.extend(["-config", config_path])
    if cli_args:
        cmd.extend(cli_args)

    print(f"Команда: {' '.join(cmd)}\n")
    result = subprocess.run(cmd, capture_output=True, text=True, cwd=".")

    if result.returncode != 0:
        print("❌ Ошибка выполнения:")
        print(result.stderr)
        return None

    print("✓ Симуляция успешно завершена\n")
    return result.stdout


def plot_metrics(run_id):
    """Строит графики для всех доступных CSV."""
    plots = {}

    # Градиентный спуск — CSV накапливается по всем тикам (append), поэтому
    # колонка Iteration скачет 0..N на каждом вызове. Ось X — сквозной номер
    # строки, иначе график читается как разрыв на каждой границе тика.
    # Имя узла в колонках не всегда Node0 — Go-логгер пишет первый подвижный
    # узел (Node0 может быть анкером и тогда был бы всегда плоским).
    if os.path.exists(GRADIENT_CSV):
        try:
            df = pd.read_csv(GRADIENT_CSV)
            node_label = df.columns[1].rsplit('_', 1)[0]  # напр. "Node2"
            x = range(len(df))
            plt.figure(figsize=(10, 5))
            plt.plot(x, df[f'{node_label}_X'], label='X', color='blue')
            plt.plot(x, df[f'{node_label}_Y'], label='Y', color='green')
            plt.plot(x, df[f'{node_label}_Z'], label='Z', color='red')
            plt.title(f'Сходимость GD ({node_label}), сквозная история по всем тикам')
            plt.xlabel('Шаг (накопительно)')
            plt.ylabel('Координата')
            plt.grid(True)
            plt.legend()

            img_path = os.path.join(IMG_DIR, f"gd_{run_id}.png")
            plt.savefig(img_path, dpi=100)
            plt.close()
            plots['gradient'] = img_path
        except Exception as e:
            print(f"⚠ Ошибка при построении графика GD: {e}")

    # EKF — та же логика: сквозной индекс вместо скачущей колонки Iteration.
    if os.path.exists(EKF_CSV):
        try:
            df = pd.read_csv(EKF_CSV)
            x = range(len(df))
            plt.figure(figsize=(10, 5))
            if len(df.columns) > 1:
                node_label = df.columns[1].rsplit('_', 1)[0]
                plt.plot(x, df[f'{node_label}_X'], label='X', color='blue')
                plt.plot(x, df[f'{node_label}_Y'], label='Y', color='green')
                plt.plot(x, df[f'{node_label}_Z'], label='Z', color='red')
            else:
                node_label = '?'
            plt.title(f'Траектория EKF ({node_label}), сквозная история по всем тикам')
            plt.xlabel('Шаг (накопительно)')
            plt.ylabel('Координата')
            plt.grid(True)
            plt.legend()

            img_path = os.path.join(IMG_DIR, f"ekf_{run_id}.png")
            plt.savefig(img_path, dpi=100)
            plt.close()
            plots['ekf'] = img_path
        except Exception as e:
            print(f"⚠ Ошибка при построении графика EKF: {e}")

    # Временная динамика (тиков). RawRMSE (инерциальный дрейф без коррекции)
    # и FinalRMSE (после GD+EKF) обычно отличаются на 2-3 порядка — на общей
    # линейной шкале FinalRMSE визуально сливается с нулём, что выглядит как
    # "система не работает", хотя это и есть цель компенсации. Разносим их
    # по отдельным панелям с независимым масштабом.
    if os.path.exists(TICK_CSV):
        try:
            df = pd.read_csv(TICK_CSV)
            fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 11))

            ax1.plot(df['Tick'], df['RawRMSE'], label='Raw RMSE (без коррекции)', marker='o', color='red')
            ax1.set_xlabel('Тик')
            ax1.set_ylabel('RMSE (м)')
            ax1.set_title('Сырая ошибка (инерциальный дрейф BelievedCoord)')
            ax1.grid(True)
            ax1.legend()

            ax2.plot(df['Tick'], df['FinalRMSE'], label='Final RMSE (после GD+EKF)', marker='s', color='green')
            ax2.set_xlabel('Тик')
            ax2.set_ylabel('RMSE (м)')
            ax2.set_title('Ошибка после коррекции дальномерами')
            ax2.grid(True)
            ax2.legend()

            ax3.plot(df['Tick'], df['CompensationRatio'], marker='^', color='blue')
            ax3.set_xlabel('Тик')
            ax3.set_ylabel('Compensation Ratio')
            ax3.set_title('Эффективность компенсации (RawRMSE / FinalRMSE)')
            ax3.grid(True)

            plt.tight_layout()
            img_path = os.path.join(IMG_DIR, f"ticks_{run_id}.png")
            plt.savefig(img_path, dpi=100)
            plt.close()
            plots['ticks'] = img_path
        except Exception as e:
            print(f"⚠ Ошибка при построении графика тиков: {e}")

    return plots


def generate_markdown(commit, config_path, cli_args, output, plots, timestamp):
    """Генерирует запись в report.md."""
    lines = []
    lines.append(f"## Тест ({timestamp})")
    lines.append(f"**Коммит:** `{commit}`")

    if config_path:
        lines.append(f"**Конфиг:** `{config_path}`")
    if cli_args:
        lines.append(f"**CLI флаги:** `{' '.join(cli_args)}`")

    lines.append("\n### Консоль")
    lines.append("```")
    lines.append(output)
    lines.append("```")

    for key, path in plots.items():
        lines.append(f"\n![{key}]({path})")

    lines.append("\n---\n")

    with open(MD_REPORT_PATH, "a", encoding="utf-8") as f:
        f.write("\n".join(lines))

    print(f"✓ Отчёт добавлен в {MD_REPORT_PATH}")


def find_configs(pattern=None):
    """Список конфигов."""
    configs = []
    for root, dirs, files in os.walk(CONFIG_DIR):
        for f in sorted(files):
            if f.endswith(".toml"):
                path = os.path.join(root, f)
                configs.append(path)

    if pattern:
        configs = [c for c in configs if pattern.lower() in c.lower()]

    return configs


def select_config_interactive():
    """Интерактивный выбор конфига."""
    configs = find_configs()

    if not configs:
        print("❌ Конфиги не найдены")
        return None

    print("\n📋 Доступные конфиги:")
    for i, cfg in enumerate(configs, 1):
        print(f"  {i}. {cfg}")

    while True:
        try:
            choice = input("\nВыберите номер (или Enter для пропуска): ").strip()
            if not choice:
                return None
            idx = int(choice) - 1
            if 0 <= idx < len(configs):
                return configs[idx]
            print("❌ Неверный выбор")
        except ValueError:
            print("❌ Введите число")


def input_cli_args():
    """Интерактивный ввод CLI аргументов."""
    print("\n⚙️  Параметры (Enter для пропуска):")
    args = []

    params = {
        "nodes": "Кол-во узлов (default: из конфига)",
        "space-min": "Min граница пространства",
        "space-max": "Max граница пространства",
        "gps-err": "Ошибка GPS (м)",
        "dist-err": "Ошибка дальномеров (м)",
        "k": "Число соседей k (default: n-1)",
        "alpha": "Learning rate",
        "lambda": "Регуляризация",
        "eps": "Epsilon (порог остановки)",
        "max-iter": "Макс итераций GD",
        "ekf-iter": "Итерации EKF",
        "q": "Шум процесса EKF",
        "r": "Шум измерений EKF",
    }

    for param, desc in params.items():
        val = input(f"  -{param}: {desc}\n    > ").strip()
        if val:
            args.extend([f"-{param}", val])

    return args


def menu_quick_test():
    """Быстрый тест со случайным конфигом."""
    configs = find_configs()
    if not configs:
        print("❌ Конфиги не найдены")
        return

    import random
    cfg = random.choice(configs)
    print(f"🎲 Выбран: {cfg}")

    output = run_go_simulation(cfg)
    if output is None:
        return

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    plots = plot_metrics(run_id)

    generate_markdown(
        get_git_commit(),
        cfg,
        None,
        output,
        plots,
        datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    )


def menu_single_config():
    """Тест одного конфига."""
    cfg = select_config_interactive()
    if not cfg:
        print("Отменено")
        return

    output = run_go_simulation(cfg)
    if output is None:
        return

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    plots = plot_metrics(run_id)

    generate_markdown(
        get_git_commit(),
        cfg,
        None,
        output,
        plots,
        datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    )


def menu_cli_only():
    """Тест с ручными CLI параметрами."""
    args = input_cli_args()
    if not args:
        print("Отменено")
        return

    output = run_go_simulation(cli_args=args)
    if output is None:
        return

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    plots = plot_metrics(run_id)

    generate_markdown(
        get_git_commit(),
        None,
        args,
        output,
        plots,
        datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    )


def menu_batch():
    """Пакетный запуск конфигов с параметром сравнения."""
    pattern = input("Фильтр конфигов (например 'anchors' или 'simple'): ").strip()
    configs = find_configs(pattern)

    if not configs:
        print("❌ Конфиги не найдены")
        return

    print(f"\n📦 Найдено {len(configs)} конфигов:")
    for cfg in configs:
        print(f"  - {cfg}")

    confirm = input("\nЗапустить все? (y/n): ").strip().lower()
    if confirm != 'y':
        return

    # Параметр K для сравнения топологий
    test_k = input("\nТестировать разные K? (Enter - нет, или список '2,3,4'): ").strip()
    k_values = []
    if test_k:
        try:
            k_values = [str(int(k.strip())) for k in test_k.split(',')]
        except ValueError:
            print("❌ Неверный формат K")
            return

    for cfg in configs:
        print(f"\n{'='*60}")
        print(f"Тестируем: {cfg}")

        if k_values:
            for k in k_values:
                print(f"\n  K={k}...")
                output = run_go_simulation(cfg, ["-k", k])
                if output is None:
                    continue

                run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
                plots = plot_metrics(run_id)
                generate_markdown(
                    get_git_commit(),
                    cfg,
                    ["-k", k],
                    output,
                    plots,
                    datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                )
        else:
            output = run_go_simulation(cfg)
            if output is None:
                continue

            run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
            plots = plot_metrics(run_id)
            generate_markdown(
                get_git_commit(),
                cfg,
                None,
                output,
                plots,
                datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            )

    print(f"\n✓ Тестирование завершено")


def main_menu():
    """Главное меню."""
    os.makedirs(IMG_DIR, exist_ok=True)

    while True:
        print(f"""
╔════════════════════════════════════════════════════════════════╗
║                    TESTBENCH - Vostok v2.0                     ║
║              Интерактивное тестирование сети локализации        ║
╚════════════════════════════════════════════════════════════════╝

1. 🎲 Случайный конфиг
2. 📋 Выбрать конфиг
3. ⚙️  Только CLI параметры
4. 📦 Пакетный запуск
5. 📊 Просмотр отчёта
0. ❌ Выход

Текущий коммит: {get_git_commit()}
""")

        choice = input("Выберите действие: ").strip()

        if choice == "1":
            menu_quick_test()
        elif choice == "2":
            menu_single_config()
        elif choice == "3":
            menu_cli_only()
        elif choice == "4":
            menu_batch()
        elif choice == "5":
            if os.path.exists(MD_REPORT_PATH):
                os.system(f"cat {MD_REPORT_PATH} | head -100")
            else:
                print(f"❌ {MD_REPORT_PATH} не найден")
        elif choice == "0":
            print("До свидания!")
            break
        else:
            print("❌ Неверный выбор")

        input("\nPress Enter для продолжения...")


if __name__ == "__main__":
    main_menu()
