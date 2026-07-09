# PIPER-1-6

Repositório de desenvolvimento do ambiente de simulação, guiagem, navegação e integração com X-Plane para a aeronave **Piper J-3 Cub em escala 1/6**.

O projeto reúne modelos matemáticos não lineares, modelos Simulink, scripts MATLAB, algoritmos de guiagem por waypoints, filtros de navegação, modelos de sensores e rotinas de integração com o simulador X-Plane.

---

## Visão geral

O repositório está organizado em módulos independentes, mas integrados pelo script principal de inicialização. A estrutura atual contém aproximadamente:

- **103 arquivos listados**;
- **77 arquivos `.m`**;
- **9 arquivos `.md`**;
- modelos Simulink `.slx`;
- arquivos de parâmetros `.mat`;
- documentos técnicos `.pdf`;
- integração com X-Plane via XPlaneConnect.

O fluxo típico de uso é:

1. configurar o ambiente MATLAB;
2. executar o script de inicialização;
3. abrir ou executar o modelo Simulink desejado;
4. configurar waypoints, sensores ou filtros;
5. executar a simulação;
6. visualizar resultados por meio dos scripts de plotagem;
7. gerar documentação automática do repositório, quando necessário.

---

## Estrutura do repositório

```text
PIPER-1-6/
├── docs/
├── guiagem/
├── modelos/
│   └── Não Linear/
├── navigation/
│   ├── DBN/
│   ├── FK/
│   └── sensors/
├── plots/
├── xplane/
│   ├── Xplane_Interface/
│   └── XPlaneConnect-master/
├── .gitignore
├── inicializar_UI.m
├── gerar_relatorio_repositorio.py
├── relatorio_repositorio.pdf
└── README.md
```

> Observação: `gerar_relatorio_repositorio.py` e `relatorio_repositorio.pdf` correspondem ao mecanismo de documentação automática do projeto. Caso o relatório ainda não tenha sido gerado em uma cópia local do repositório, execute o script descrito na seção [Geração automática de relatório](#geração-automática-de-relatório).

---

## Módulos principais

### `docs/`

Contém documentos técnicos e referências utilizadas no desenvolvimento, como dissertações, materiais de aula e documentos compartilhados do trabalho.

Exemplos de arquivos identificados:

- `aula7_c.pdf`
- `Dissertacao_Mestrado_Sato (1).pdf`
- `dissertação_Marcelo.pdf`
- `Trabalho3_compartilhado.pdf`

---

### `guiagem/`

Módulo responsável pela guiagem por waypoints, controle de missão e interface visual de simulação.

Arquivos principais:

| Arquivo | Função |
|---|---|
| `NL_guidance.slx` | Modelo Simulink de guiagem LOS, piloto automático e planta não linear. |
| `gui_waypoints.m` | Interface gráfica para criação e edição de waypoints. |
| `inicializar.m` | Inicialização de parâmetros, controladores, waypoints e integração. |
| `scr_waypoints.m` | Definição textual de missão por waypoints. |
| `scr_aux_wp.m` | Rotina auxiliar para preparar e executar simulações por waypoint. |
| `analise_sim.m` | Análise pós-simulação. |
| `find_trim.m` | Busca alternativa de ponto de equilíbrio. |
| `trim_cost_fn.m` | Função custo usada em trimagem. |
| `plots_guidance.mlx` | Live Script para validação e plotagem da guiagem. |

A guiagem utiliza lógica **Line-of-Sight (LOS)**, recebendo posição atual, matriz de waypoints, raio de aceitação e estado inicial. As principais saídas são:

- `psi_ref`: proa desejada;
- `h_ref`: altitude desejada;
- `v_ref`: velocidade desejada;
- `wp_idx_monitor`: índice do waypoint ativo;
- `dist_monitor`: distância até o waypoint alvo.

A troca de waypoint ocorre quando a distância até o ponto alvo é menor ou igual a `R_accept`.

---

### `modelos/`

Contém os modelos matemáticos da aeronave. O subdiretório mais importante é `modelos/Não Linear/`, que reúne o modelo 6-DOF usado pelos modelos Simulink.

#### `modelos/Não Linear/`

Arquivos principais:

| Arquivo | Função |
|---|---|
| `sfunction_piper.m` | S-Function Level-1 para integração do modelo 6-DOF no Simulink. |
| `dyn_rigidbody.m` | Equações de movimento e derivadas dos 12 estados. |
| `obs_rigidbody.m` | Conversão dos estados em saídas observáveis. |
| `aerodynamics.m` | Cálculo de forças e momentos aerodinâmicos. |
| `aerodynamics2.m` | Versão alternativa do modelo aerodinâmico. |
| `propulsion.m` | Modelo de propulsão. |
| `equilibrium.m` | Definição do ponto de equilíbrio `Xe` e entrada de equilíbrio `Ue`. |
| `lin.m` | Linearização numérica em torno do equilíbrio. |
| `decoupling.m` | Separação longitudinal e látero-direcional. |
| `ISA.m` | Modelo de atmosfera padrão. |
| `modelo.slx` | Modelo Simulink da planta não linear. |
| `gerar_log.m` | Extração de variáveis e geração de gráficos/logs. |
| `plot_long.m` | Visualização de variáveis longitudinais. |

A S-Function `sfunction_piper.m` possui:

- **12 estados contínuos**: velocidades, taxas angulares, ângulos de Euler e posição NED;
- **4 entradas**: throttle, profundor, aileron e leme;
- **21 saídas**: variáveis aerodinâmicas, estados, posição, velocidades no corpo e derivadas.

O ponto de equilíbrio definido no projeto corresponde a voo reto e nivelado com:

```matlab
VT = 15;        % m/s
Theta = -0.121684;   % rad
Xe = [15; 0; -1.8343; 0; 0; 0; 0; -0.1217; 0; 0; 0; -100]
Ue = [0.4914; 0.0155; 0; 0]
```

---

### `navigation/`

Módulo dedicado à navegação inercial, estimação de estados, sensores e filtros.

Estrutura principal:

```text
navigation/
├── DBN/
├── FK/
├── sensors/
├── ins_init_block.m
└── README.md
```

#### `navigation/DBN/`

Contém o algoritmo de navegação baseado em DBN e sua inicialização:

- `DBN_solver.m`
- `init_DBN.m`

O DBN propaga atitude, velocidade e posição usando leituras de aceleração e giroscópio, com inicialização baseada no estado inicial do X-Plane.

#### `navigation/FK/`

Contém filtros de Kalman estendidos diretos e indiretos:

- `EKF_DI_solver.m`
- `EKF_DI_EM_solver.m`
- `EKF_INDI_solver.m`
- `EKF_INDI_EM_solver.m`
- `init_EKF_DI.m`
- `init_EKF_INDI.m`

Os filtros estimam posição, velocidade, atitude, aceleração em NED e estados internos associados a bias, escala e erros de medição.

#### `navigation/sensors/`

Modelos de sensores e inicialização:

- IMU `ICM20689`;
- magnetômetro `IST8310`;
- GPS/GNSS `NEOM8`;
- `ins_init_sensors.m`;
- `ins_telemetry.m`.

Os sensores incluem modelos de erro, ruído, bias, escala, desalinhamento, quantização, saturação e efeitos de temperatura.

---

### `plots/`

Contém scripts para visualização e comparação de resultados de simulação.

Arquivos típicos:

- `init_plots.m`
- `plot3d_voo.m`
- `plot3d_voo_DBN.m`
- `plot3d_voo_xplane.m`
- `plot_compare_all.m`
- `plot_compare_all2.m`
- `plot_compare_all3.m`
- `plot_compare_all4.m`
- `plot_compare_all5.m`

Esses scripts permitem gerar trajetórias 3D, gráficos de altitude, ângulos de Euler, velocidades, erros de estimadores e métricas comparativas entre X-Plane, DBN e EKFs.

---

### `xplane/`

Módulo de integração com o simulador X-Plane.

Estrutura principal:

```text
xplane/
├── Xplane_Interface/
└── XPlaneConnect-master/
```

#### `xplane/Xplane_Interface/`

Contém os scripts próprios de interface entre o projeto e o X-Plane:

- `ins_init_xplane.m`
- `ins_initial_state_xplane.m`
- `ins_read_xplane.m`
- `ins_send_xplane.m`
- `ins_delta_posi_calculation.m`
- `ins_wyps_insertion.m`

Também há uma pasta de testes com scripts e modelo Simulink para validação da comunicação.

#### `xplane/XPlaneConnect-master/`

Contém a biblioteca XPlaneConnect, incluindo funções MATLAB para comunicação UDP com o X-Plane, como:

- `openUDP.m`
- `closeUDP.m`
- `getDREFs.m`
- `getPOSI.m`
- `sendCTRL.m`
- `sendDREF.m`
- `sendPOSI.m`
- `pauseSim.m`
- `sendWYPT.m`

---

## Dependências

Requisitos principais:

- MATLAB R2024a ou superior;
- Simulink;
- Optimization Toolbox, se forem utilizadas rotinas de trimagem ou otimização;
- X-Plane, para simulações integradas;
- XPlaneConnect, já incluído no repositório em `xplane/XPlaneConnect-master/`;
- Python 3, para geração automática de relatório;
- biblioteca Python `reportlab`, para geração de PDF.

Para instalar a dependência Python do relatório:

```bash
pip install reportlab
```

Se estiver usando ambiente virtual:

```bash
python -m pip install reportlab
```

---

## Como executar a simulação principal

### 1. Abrir o MATLAB na raiz do repositório

Certifique-se de que a pasta atual do MATLAB seja a raiz do projeto `PIPER-1-6`.

### 2. Inicializar o ambiente

Execute:

```matlab
inicializar_UI
```

Esse script limpa o ambiente, adiciona caminhos essenciais, configura telemetria, executa `inicializar` e abre a interface de waypoints.

Alternativamente, para inicialização manual do módulo de guiagem:

```matlab
inicializar
```

### 3. Usar a interface de waypoints

Execute:

```matlab
gui_waypoints
```

Na interface, é possível:

- clicar no mapa para adicionar waypoints;
- editar coordenadas, altitude e velocidade;
- ajustar o raio de aceitação `R_accept`;
- definir tempo de simulação;
- executar automaticamente a simulação;
- visualizar os resultados ao final.

### 4. Executar missão por script

Também é possível configurar a missão diretamente em:

```matlab
guiagem/scr_waypoints.m
```

Depois, execute:

```matlab
scr_waypoints
```

---

## Convenções importantes

- O sistema de coordenadas utilizado é principalmente **NED**:
  - Norte;
  - Leste;
  - Down.
- A altitude positiva para cima é representada como `-xD`.
- O waypoint inicial é geralmente fixo na origem.
- O estado de equilíbrio da aeronave utiliza altitude de referência de aproximadamente 100 m.
- A S-Function `sfunction_piper.m` é compartilhada entre os modelos Simulink.
- As variáveis principais são carregadas no workspace base do MATLAB.

---

## Geração automática de relatório

O repositório inclui o script:

```text
gerar_relatorio_repositorio.py
```

Esse script gera automaticamente um relatório PDF do projeto a partir da raiz do repositório.

### O que o script faz

O script:

1. percorre a pasta raiz e todas as subpastas;
2. registra a estrutura de pastas e arquivos;
3. lista todos os arquivos encontrados por caminho relativo;
4. gera um resumo por extensão;
5. abre e copia o conteúdo apenas de arquivos `.m` e `.md`;
6. gera um único PDF final chamado:

```text
relatorio_repositorio.pdf
```

Arquivos como `.pdf`, `.slx`, `.mat`, `.mlx`, `.jar`, `.txt` e outros são apenas listados. O conteúdo desses arquivos não é lido nem copiado para o relatório.

### Como executar

Na raiz do repositório, execute:

```bash
python gerar_relatorio_repositorio.py
```

Ou, se estiver usando ambiente virtual:

```bash
.
venv\Scripts\activate
python gerar_relatorio_repositorio.py
```

No PowerShell, a ativação pode ser:

```powershell
.\venv\Scripts\Activate.ps1
python gerar_relatorio_repositorio.py
```

### Saída esperada

Após a execução, será criado:

```text
relatorio_repositorio.pdf
```

O PDF contém:

1. estrutura de pastas e arquivos;
2. lista ordenada de arquivos;
3. resumo por extensão;
4. conteúdo completo dos arquivos `.m` e `.md`.

---

## Relatórios e documentação

Além dos arquivos `README.md` presentes nos módulos, o projeto pode ser documentado automaticamente usando o relatório gerado pelo script Python. Recomenda-se gerar um novo relatório sempre que houver alterações significativas em:

- scripts MATLAB;
- modelos Simulink;
- documentação Markdown;
- estrutura de pastas;
- arquivos de entrada, parâmetros ou resultados relevantes.

---

## Observações de manutenção

- Manter os READMEs dos módulos sincronizados com alterações nos scripts principais.
- Atualizar este README sempre que um novo módulo for adicionado.
- Evitar versionar arquivos temporários do MATLAB, caches e outputs pesados de simulação.
- Se o relatório PDF for grande, avaliar se ele deve ser mantido no repositório ou gerado sob demanda.
- Conferir periodicamente os caminhos configurados em `inicializar.m` e `inicializar_UI.m`.

---

## Arquivos de entrada e resultados identificados

O relatório atual identifica, entre outros, os seguintes tipos de arquivo:

| Extensão | Função típica no projeto |
|---|---|
| `.m` | Scripts e funções MATLAB. |
| `.md` | Documentação dos módulos. |
| `.slx` | Modelos Simulink. |
| `.mat` | Parâmetros, trim e dados de modelo. |
| `.mlx` | Live Scripts MATLAB. |
| `.pdf` | Referências e documentos técnicos. |
| `.jar` | Biblioteca Java do XPlaneConnect. |
| `.txt` | Arquivos auxiliares e gravações. |

---

## Referência rápida

### Inicialização com interface

```matlab
inicializar_UI
```

### Interface de waypoints

```matlab
gui_waypoints
```

### Missão por script

```matlab
scr_waypoints
```

### Abrir modelo de guiagem

```matlab
open('guiagem/NL_guidance.slx')
```

### Gerar relatório do repositório

```bash
python gerar_relatorio_repositorio.py
```

---

## Status

Este README foi atualizado com base no relatório automático `relatorio_repositorio.pdf`, incluindo a documentação do script `gerar_relatorio_repositorio.py` e do fluxo de geração do relatório do repositório.
