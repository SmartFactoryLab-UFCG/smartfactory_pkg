#!/usr/bin/env python3
import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path


def pct(numerator: int, denominator: int) -> float:
    if denominator == 0:
        return math.nan
    return (100.0 * numerator) / denominator


def mean(values):
    if not values:
        return math.nan
    return sum(values) / len(values)


def format_pct(value: float) -> str:
    if math.isnan(value):
        return "N/A"
    return f"{value:.1f}"


def to_int(row, key):
    raw = (row.get(key, "") or "").strip()
    return 1 if raw in {"1", "true", "True", "y", "Y"} else 0


def analyze(rows):
    by_mode = defaultdict(list)
    for row in rows:
        by_mode[row["mode"]].append(row)

    report = []
    for mode in sorted(by_mode.keys()):
        mode_rows = by_mode[mode]
        n = len(mode_rows)

        initial_success = [to_int(r, "initial_grasp_success") for r in mode_rows]
        failure_detected = [to_int(r, "failure_detected") for r in mode_rows]
        silent_failure = [to_int(r, "silent_failure") for r in mode_rows]
        task_completed = [to_int(r, "task_completed") for r in mode_rows]
        interrupted = [to_int(r, "interrupted_execution") for r in mode_rows]
        recovered = [to_int(r, "recovered_execution") for r in mode_rows]
        durations = [float((r.get("duration_s", "0") or "0").strip() or 0) for r in mode_rows]

        failed_idx = [i for i, v in enumerate(initial_success) if v == 0]
        failed_count = len(failed_idx)
        detected_in_failed = sum(failure_detected[i] for i in failed_idx)
        silent_in_failed = sum(silent_failure[i] for i in failed_idx)

        consistency_warnings = []
        for i in failed_idx:
            if failure_detected[i] == 1 and silent_failure[i] == 1:
                consistency_warnings.append(
                    f"linha trial_id={mode_rows[i].get('trial_id', 'N/A')}: "
                    "failure_detected=1 e silent_failure=1 ao mesmo tempo."
                )

        report.append(
            {
                "mode": mode,
                "n_trials": n,
                "initial_grasp_success_rate": pct(sum(initial_success), n),
                "failure_detection_rate_failed_only": pct(detected_in_failed, failed_count),
                "silent_failure_rate_failed_only": pct(silent_in_failed, failed_count),
                "task_completion_rate": pct(sum(task_completed), n),
                "interrupted_execution_rate": pct(sum(interrupted), n),
                "recovered_execution_rate": pct(sum(recovered), n),
                "avg_duration_s": mean(durations),
                "failed_trials_count": failed_count,
                "consistency_warnings": consistency_warnings,
            }
        )

    return report


def render_markdown(report, source_csv: Path) -> str:
    lines = []
    lines.append("# Relatorio de Experimentos (Malha Aberta vs Fechada)")
    lines.append("")
    lines.append(f"Fonte: `{source_csv}`")
    lines.append("")
    lines.append(
        "| mode | n_trials | initial_grasp_success_rate(%) | failure_detection_rate_failed_only(%) | "
        "silent_failure_rate_failed_only(%) | task_completion_rate(%) | interrupted_execution_rate(%) | "
        "recovered_execution_rate(%) | avg_duration_s | failed_trials_count |"
    )
    lines.append(
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|"
    )

    for row in report:
        lines.append(
            f"| {row['mode']} | {row['n_trials']} | {format_pct(row['initial_grasp_success_rate'])} | "
            f"{format_pct(row['failure_detection_rate_failed_only'])} | "
            f"{format_pct(row['silent_failure_rate_failed_only'])} | "
            f"{format_pct(row['task_completion_rate'])} | "
            f"{format_pct(row['interrupted_execution_rate'])} | "
            f"{format_pct(row['recovered_execution_rate'])} | "
            f"{row['avg_duration_s']:.1f} | {row['failed_trials_count']} |"
        )

    has_warning = any(item["consistency_warnings"] for item in report)
    if has_warning:
        lines.append("")
        lines.append("## Avisos de Consistencia")
        for item in report:
            for warning in item["consistency_warnings"]:
                lines.append(f"- [{item['mode']}] {warning}")

    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description="Analisa CSV de experimentos open-loop vs closed-loop.")
    parser.add_argument("--input", required=True, help="CSV gerado por run_experiments.sh")
    parser.add_argument("--export-md", default="", help="Caminho para salvar relatorio em Markdown")
    args = parser.parse_args()

    source_csv = Path(args.input).resolve()
    if not source_csv.exists():
        raise SystemExit(f"Arquivo nao encontrado: {source_csv}")

    with source_csv.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        raise SystemExit("CSV sem dados de trial.")

    report = analyze(rows)
    markdown = render_markdown(report, source_csv)
    print(markdown)

    if args.export_md:
        out_path = Path(args.export_md).resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(markdown, encoding="utf-8")
        print(f"Relatorio salvo em: {out_path}")


if __name__ == "__main__":
    main()

