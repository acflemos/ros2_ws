# ros2_ws/src — Workspace de Desenvolvimento robodog2

Este README é a **memória compartilhada** do projeto: explica o propósito de cada pasta, as regras de trabalho, as convenções adotadas e os próximos passos. Destina-se ao autor, ao assistente de IA (Claude) e a futuros leitores do projeto.

---

## Visão geral

Este workspace é o ambiente de desenvolvimento de uma família de pacotes ROS2 para o robô **ROSMASTER X3** da Yahboom, com foco em:

- **Simulação no Gazebo Fortress** (algo que o próprio fabricante não fornece)
- **Navegação autônoma** com Nav2, SLAM e comportamento inspirado em cão de vigilância
- **Documentação em português**, como base para um futuro projeto educativo de ROS2 para desenvolvedores lusófonos

O pacote principal e âncora do projeto é o **robodog2**, já publicado no GitHub. Os demais pacotes aqui são derivados, experimentos e estudos que evoluirão ao lado dele.

---

## Estrutura de pastas

```
ros2_ws/src/
│
├── robodog2/                  # PACOTE ÂNCORA — estável, publicado no GitHub
│                              # ⚠️  NÃO modificar a partir deste contexto
│                              # Modificações: abrir VSCode direto em robodog2/
│                              #               criar branch adequada antes de alterar
│
├── yahboomcar_bringup/        # Código original Yahboom — inicialização do hardware real
├── yahboomcar_description/    # Código original Yahboom — URDF/geometria do X3 e R2
├── yahboomcar_laser/          # Código original Yahboom — comportamentos LiDAR
├── yahboomcar_nav/            # Código original Yahboom — SLAM e Nav2 para hardware real
│                              # ⚠️  Estes pacotes são referência — modificar com cautela
│
└── robodog2_*/                # NOVOS PACOTES AUTORAIS (ainda a criar)
                               # Derivados do código Rosmaster, NVIDIA e outros
                               # Quando maduros: publicados no GitHub ao lado do robodog2
```

### Pasta irmã: `~/codigo_referencia/`

```
codigo_referencia/
├── Rosmaster-x3/              # Código-fonte completo do fabricante Yahboom
│   └── src/                   # Comentado em português (maio/2026)
├── robodog1/                  # Versão ROS1 Noetic — CONGELADA, apenas referência
└── ...zips de backup
```

> **Regra:** `codigo_referencia/` é **somente leitura**. Permitido apenas adicionar comentários e criar READMEs. Nunca modificar código existente.

---

## Convenção de nomenclatura

| Prefixo | Origem | Exemplo |
|---|---|---|
| `yahboomcar_` | Fabricante Yahboom (original) | `yahboomcar_bringup` |
| `robodog2_` | Autoral — derivado/inspirado | `robodog2_laser`, `robodog2_vision` |

O prefixo `robodog2_` em todos os pacotes autorais:
- Separa claramente o que é código autoral do código do fabricante
- Facilita a publicação futura no GitHub em conjunto com o robodog2
- Evita conflitos de nome com pacotes da comunidade ROS2

---

## Regras de trabalho

### robodog2 — pacote âncora
- **Nunca modificar** `robodog2/` quando trabalhando a partir deste contexto (`src/`)
- Modificações ao robodog2 → abrir Claude **dentro da pasta `robodog2/`** no VSCode
- Sempre criar **branch adequada** antes de qualquer alteração no robodog2

### codigo_referencia
- Apenas leitura e estudo
- Pode adicionar comentários em português e READMEs
- Nunca alterar código existente

### Novos pacotes `robodog2_*`
- Esta pasta `src/` é o sandbox: experimentos, testes, tentativas de integração
- Trabalho em andamento aqui não deve pressupor estabilidade
- Quando um pacote `robodog2_*` estiver maduro → publicar no GitHub

---

## Por que este projeto é relevante para a comunidade

O fabricante Yahboom **não fornece suporte a Gazebo** para o ROSMASTER X3. Os pacotes `yahboomcar_*` são 100% voltados para hardware real. O **robodog2 é provavelmente o único projeto público** que coloca o ROSMASTER X3 a funcionar em simulação no **Gazebo Fortress (ROS2 Humble)**, com:

- URDF de simulação com plugins Fortress (velocidade, odometria, LiDAR)
- Bridge ROS-Gazebo configurado
- Mundos SDF convertidos de Gazebo Classic para Fortress
- Nav2 + SLAM calibrado para o X3
- Navegação autônoma com comportamento de patrulha

---

## Próximos passos

### Imediatos
- [ ] Criar git local em `src/` para versionar pacotes Yahboom e futuros `robodog2_*`
- [ ] Adicionar destaque no README do `robodog2` sobre simulação Gazebo Fortress (contribuição inédita)
- [ ] Estudar e documentar os pacotes `yahboomcar_*` presentes neste workspace

### Médio prazo — estudo do código Rosmaster
- [ ] `yahboomcar_bringup` — entender o driver mecanum (`Mcnamu_driver_X3.py`) para eventual integração
- [ ] `yahboomcar_laser` — estudar lógica de desvio e rastreamento LiDAR
- [ ] Explorar pacotes extras em `codigo_referencia/Rosmaster-x3/src/` não instalados aqui:
  - `yahboomcar_msgs` — mensagens customizadas do fabricante
  - `yahboomcar_ctrl` — controle por joystick
  - `yahboomcar_visual` — visão computacional (QR, AR, cores)
  - `yahboomcar_mediapipe` — reconhecimento de gestos

### Longo prazo — projeto educativo
- [ ] Documentação completa em português de todo o stack ROS2
- [ ] Publicar pacotes `robodog2_*` maduros no GitHub ao lado do robodog2
- [ ] Estruturar material educativo: do zero ao robô autônomo em português

---

## Referências

- [robodog2 no GitHub](https://github.com/acflemos/robodog2) — pacote principal
- [robodog1](https://github.com/acflemos/robodog1) — versão ROS1, congelada
- [ROSMASTER X3 — Yahboom](https://github.com/YahboomTechnology/ROSMASTERX3)
- [Nav2](https://navigation.ros.org/)
- [Gazebo Fortress](https://gazebosim.org/docs/fortress)
- [ROS2 Humble](https://docs.ros.org/en/humble/)
