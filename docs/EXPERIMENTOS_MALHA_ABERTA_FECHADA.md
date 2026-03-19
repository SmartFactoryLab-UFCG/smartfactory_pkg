# Experimentos: Malha Aberta vs Malha Fechada (smartfactory_pkg)

Este guia padroniza a repeticao dos experimentos para comparar:
- `open-loop` (baseline): `ros2 run smartfactory_behavior_tree behavior_pick_and_place`
- `closed-loop` (proposto): `ros2 run smartfactory_behavior_tree pick_and_place`

## 1) Objetivo e metrica

Comparar os modos de controle com as mesmas condicoes de percepcao e movimento, medindo:
- `Initial grasp success rate (%)`
- `Failure detection rate (%)` (calculado somente sobre tentativas com grasp inicial falho)
- `Silent failure rate (%)` (somente sobre tentativas com grasp inicial falho)
- `Task completion rate (%)`
- `Interrupted executions (%)`
- `Recovered executions (%)`
- `Avg duration (s)`

## 2) Pre-requisitos

- Workspace compilado.
- Cena principal operacional.
- Operador para reset da celula entre trials.

Comandos:

```bash
cd /workspaces/smartfactory_pkg/smartfactoring_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3) Terminal A: Cena fixa do experimento

Mantenha este terminal rodando durante toda a sessao:

```bash
ros2 launch smart_factory_bringup smart_factory_scene.launch.py
```

## 4) Coleta automatizada (script)

Os scripts criados ficam em:
- `/workspaces/smartfactory_pkg/experiments/run_experiments.sh`
- `/workspaces/smartfactory_pkg/experiments/analyze_results.py`

Dar permissao de execucao (uma vez):

```bash
chmod +x /workspaces/smartfactory_pkg/experiments/run_experiments.sh
chmod +x /workspaces/smartfactory_pkg/experiments/analyze_results.py
```

### Rodar sessao completa (aberta + fechada)

No Terminal B:

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/smartfactory_pkg/smartfactoring_ws/install/setup.bash
/workspaces/smartfactory_pkg/experiments/run_experiments.sh --mode both --trials 15 --operator "seu_nome"
```

Para banca (escala maior), use:

```bash
/workspaces/smartfactory_pkg/experiments/run_experiments.sh --mode both --trials 50 --operator "seu_nome"
```

Ou 100 repeticoes por modo:

```bash
/workspaces/smartfactory_pkg/experiments/run_experiments.sh --mode both --trials 100 --operator "seu_nome"
```

## 5) Fluxo detalhado de cada trial

Para cada trial, o script faz:
1. Pausa para voce preparar a celula (objeto, estado inicial, seguranca).
2. Inicia o no do modo (`open` ou `closed`) e grava log bruto.
3. Espera voce encerrar o trial (ENTER).
4. Faz perguntas de rotulagem do resultado.
5. Registra tudo em CSV.

Campos coletados por trial:
- `initial_grasp_success` (0/1)
- `failure_detected` (0/1)
- `silent_failure` (0/1)
- `task_completed` (0/1)
- `interrupted_execution` (0/1)
- `recovered_execution` (0/1)
- `duration_s`
- `notes`
- `log_file`

## 6) Analise dos resultados

Ao final, rode:

```bash
python3 /workspaces/smartfactory_pkg/experiments/analyze_results.py \
  --input /workspaces/smartfactory_pkg/experiments/results/<SESSION_TAG>/trials.csv \
  --export-md /workspaces/smartfactory_pkg/experiments/results/<SESSION_TAG>/report.md
```

O script imprime uma tabela consolidada por modo e salva o relatorio em Markdown.

## 7) Boas praticas para validade experimental

Mantenha constantes:
- mesma iluminacao;
- mesma configuracao de sensores;
- mesmo posicionamento inicial da peca;
- mesmas velocidades/parametros do UR10;
- mesmo procedimento de reset entre trials.

Recomendacao de ordem:
- executar em blocos alternados para reduzir vies temporal:
  - bloco 1: 10 open + 10 closed
  - bloco 2: 10 open + 10 closed
  - etc.

## 8) Estrutura de saida

Cada sessao cria:

`/workspaces/smartfactory_pkg/experiments/results/<SESSION_TAG>/`
- `trials.csv`
- `session_info.txt`
- `report.md` (apos analise)
- `logs/` (um `.log` por trial)

## 9) Troubleshooting rapido

- Erro `install/setup.bash nao encontrado`:
  - rode `colcon build --symlink-install` no workspace.
- No nao inicia:
  - confirme se o Terminal A esta ativo com `smart_factory_scene.launch.py`.
- Dado inconsistente em CSV:
  - use o campo `notes` no trial e corrija manualmente o CSV, se necessario.

