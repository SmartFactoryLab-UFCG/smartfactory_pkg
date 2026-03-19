#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DEFAULT="/workspaces/smartfactory_pkg/smartfactoring_ws"
OUTPUT_ROOT_DEFAULT="/workspaces/smartfactory_pkg/experiments/results"

WORKSPACE="$WORKSPACE_DEFAULT"
OUTPUT_ROOT="$OUTPUT_ROOT_DEFAULT"
MODE="both"
TRIALS=15
OPERATOR="${USER:-operator}"
SESSION_TAG="$(date -u +%Y%m%d_%H%M%S)"

CURRENT_RUNNER_PID=""

usage() {
  cat <<'EOF'
Uso:
  run_experiments.sh [opcoes]

Opcoes:
  --mode <open|closed|both>   Modo experimental (padrao: both)
  --trials <N>                Numero de repeticoes por modo (padrao: 15)
  --workspace <path>          Workspace ROS 2 (padrao: /workspaces/smartfactory_pkg/smartfactoring_ws)
  --output-root <path>        Pasta raiz dos resultados (padrao: /workspaces/smartfactory_pkg/experiments/results)
  --operator <nome>           Identificacao do operador (padrao: usuario atual)
  --session-tag <tag>         Tag da sessao (padrao: data UTC)
  -h, --help                  Mostra esta ajuda
EOF
}

cleanup_runner() {
  if [[ -n "${CURRENT_RUNNER_PID}" ]] && kill -0 "${CURRENT_RUNNER_PID}" 2>/dev/null; then
    kill -INT "${CURRENT_RUNNER_PID}" 2>/dev/null || true
    sleep 2
    kill -TERM "${CURRENT_RUNNER_PID}" 2>/dev/null || true
    sleep 1
    kill -KILL "${CURRENT_RUNNER_PID}" 2>/dev/null || true
  fi
}

trap cleanup_runner EXIT

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="${2:-}"
      shift 2
      ;;
    --trials)
      TRIALS="${2:-}"
      shift 2
      ;;
    --workspace)
      WORKSPACE="${2:-}"
      shift 2
      ;;
    --output-root)
      OUTPUT_ROOT="${2:-}"
      shift 2
      ;;
    --operator)
      OPERATOR="${2:-}"
      shift 2
      ;;
    --session-tag)
      SESSION_TAG="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Opcao desconhecida: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ "${MODE}" != "open" && "${MODE}" != "closed" && "${MODE}" != "both" ]]; then
  echo "Erro: --mode deve ser open, closed ou both." >&2
  exit 1
fi

if ! [[ "${TRIALS}" =~ ^[0-9]+$ ]] || [[ "${TRIALS}" -le 0 ]]; then
  echo "Erro: --trials deve ser inteiro positivo." >&2
  exit 1
fi

if [[ ! -d "${WORKSPACE}" ]]; then
  echo "Erro: workspace nao encontrado: ${WORKSPACE}" >&2
  exit 1
fi

if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "Erro: ${WORKSPACE}/install/setup.bash nao encontrado." >&2
  echo "Rode o build antes: colcon build --symlink-install" >&2
  exit 1
fi

SESSION_DIR="${OUTPUT_ROOT}/${SESSION_TAG}"
LOG_DIR="${SESSION_DIR}/logs"
CSV_PATH="${SESSION_DIR}/trials.csv"
SUMMARY_PATH="${SESSION_DIR}/session_info.txt"

mkdir -p "${LOG_DIR}"

cat > "${CSV_PATH}" <<'EOF'
trial_id,mode,operator,start_utc,end_utc,duration_s,initial_grasp_success,failure_detected,silent_failure,task_completed,interrupted_execution,recovered_execution,notes,log_file
EOF

{
  echo "session_tag=${SESSION_TAG}"
  echo "operator=${OPERATOR}"
  echo "workspace=${WORKSPACE}"
  echo "mode=${MODE}"
  echo "trials_per_mode=${TRIALS}"
  echo "created_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
} > "${SUMMARY_PATH}"

prompt_yn() {
  local prompt="$1"
  local default="$2"
  local answer
  while true; do
    read -r -p "${prompt} [y/n] (padrao: ${default}): " answer
    answer="${answer:-$default}"
    case "${answer}" in
      y|Y) echo "1"; return 0 ;;
      n|N) echo "0"; return 0 ;;
      *) echo "Digite y ou n." ;;
    esac
  done
}

mode_to_exec() {
  case "$1" in
    open) echo "behavior_pick_and_place" ;;
    closed) echo "pick_and_place" ;;
    *)
      echo "Modo invalido: $1" >&2
      exit 1
      ;;
  esac
}

default_yn_from_int() {
  if [[ "$1" -eq 1 ]]; then
    echo "y"
  else
    echo "n"
  fi
}

if [[ "${MODE}" == "both" ]]; then
  MODES=("open" "closed")
else
  MODES=("${MODE}")
fi

echo "Sessao criada em: ${SESSION_DIR}"
echo "Antes de iniciar: deixe o Terminal A com a cena ativa."
echo "Comando esperado no Terminal A:"
echo "  ros2 launch smart_factory_bringup smart_factory_scene.launch.py"
echo

trial_global=0
for mode_name in "${MODES[@]}"; do
  exec_name="$(mode_to_exec "${mode_name}")"
  echo "=== Iniciando modo: ${mode_name} (executavel: ${exec_name}) ==="

  for ((i=1; i<=TRIALS; i++)); do
    trial_global=$((trial_global + 1))
    trial_id="${mode_name}_$(printf "%03d" "${i}")"
    log_path="${LOG_DIR}/${trial_id}.log"

    echo
    echo "--- Trial ${i}/${TRIALS} [${mode_name}] ---"
    echo "1) Prepare a celula (objeto posicionado e estado inicial padronizado)."
    read -r -p "Pressione ENTER para iniciar este trial..."

    start_epoch="$(date +%s)"
    start_utc="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

    (
      set -e
      source /opt/ros/jazzy/setup.bash
      source "${WORKSPACE}/install/setup.bash"
      stdbuf -oL -eL ros2 run smartfactory_behavior_tree "${exec_name}"
    ) 2>&1 | tee "${log_path}" &
    CURRENT_RUNNER_PID=$!

    echo "Trial em execucao. Logs em: ${log_path}"
    read -r -p "Quando o trial terminar, pressione ENTER para encerrar o no..."

    cleanup_runner
    wait "${CURRENT_RUNNER_PID}" 2>/dev/null || true
    CURRENT_RUNNER_PID=""

    end_epoch="$(date +%s)"
    end_utc="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    duration_s=$((end_epoch - start_epoch))

    auto_failure_detected=0
    if rg -q "Peça caiu|Perdi a peça|Movimento falhou|Erro ao" "${log_path}"; then
      auto_failure_detected=1
    fi

    auto_task_completed=0
    if rg -q "Kinect confirmou id=0|UR10 atingiu a posição final|Movimento para esteira concluído com sucesso" "${log_path}"; then
      auto_task_completed=1
    fi

    failure_default="$(default_yn_from_int "${auto_failure_detected}")"
    completed_default="$(default_yn_from_int "${auto_task_completed}")"

    echo "Preencha os resultados do trial ${trial_id}:"
    initial_grasp_success="$(prompt_yn "Grasp inicial bem-sucedido?" "y")"
    failure_detected="$(prompt_yn "Houve deteccao de falha durante execucao?" "${failure_default}")"
    silent_failure="$(prompt_yn "Houve falha silenciosa (falhou sem detectar)?" "n")"
    task_completed="$(prompt_yn "Task concluida com sucesso?" "${completed_default}")"
    interrupted_execution="$(prompt_yn "Execucao interrompida?" "n")"
    recovered_execution="$(prompt_yn "Houve recuperacao apos falha?" "n")"
    read -r -p "Notas (opcional): " notes

    notes_escaped="${notes//\"/\"\"}"

    printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,"%s",%s\n' \
      "${trial_id}" \
      "${mode_name}" \
      "${OPERATOR}" \
      "${start_utc}" \
      "${end_utc}" \
      "${duration_s}" \
      "${initial_grasp_success}" \
      "${failure_detected}" \
      "${silent_failure}" \
      "${task_completed}" \
      "${interrupted_execution}" \
      "${recovered_execution}" \
      "${notes_escaped}" \
      "${log_path}" >> "${CSV_PATH}"

    echo "Trial ${trial_id} registrado."
  done
done

echo
echo "Coleta finalizada."
echo "CSV: ${CSV_PATH}"
echo "Logs: ${LOG_DIR}"
echo
echo "Proximo passo (analise):"
echo "  python3 /workspaces/smartfactory_pkg/experiments/analyze_results.py --input \"${CSV_PATH}\" --export-md \"${SESSION_DIR}/report.md\""

