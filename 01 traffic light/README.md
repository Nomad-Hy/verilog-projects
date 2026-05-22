# 🚦 Traffic Light Controller — 차량 감지형 교차로 신호등

> Verilog HDL로 설계한 반응형(Adaptive) 교차로 신호등 FSM.  
> 단순 시간 순환이 아니라, 부도로 차량 감지 센서 입력에 따라 신호 흐름이 동적으로 바뀐다.

![status](https://img.shields.io/badge/status-verified-success)
![language](https://img.shields.io/badge/language-Verilog-blue)
![simulator](https://img.shields.io/badge/simulator-EDA%20Playground-orange)

---

## 1. Overview

실제 교차로는 모든 도로에 동일한 시간을 배분하지 않는다. 통행량이 많은
주도로(main road) 는 우선권을 갖고, 부도로(side road) 는 차량이
있을 때만 신호를 받는다. 본 프로젝트는 이 정책을 4-state FSM으로 구현했다.

| 구분 | 설명 |
|:---|:---|
| 설계 목표 | 차량 감지 기반 반응형 신호 제어 |
| 구현 언어 | Verilog HDL (RTL) |
| FSM 유형 | Moore Machine (출력 = 상태) |
| 검증 환경 | EDA Playground / ModelSim |
| 검증 방식 | 시나리오 기반 6-case 테스트벤치 |

---

## 2. Design Specification

### 2.1 신호 정책

| 도로 | 정책 |
|:---|:---|
| 주도로 (Main) | 기본 우선권. 부도로에 차량이 없으면 녹색을 무한 유지 |
| 부도로 (Side) | 차량 감지 시에만 진입. 차량이 빠지면 최대 시간 전이라도 조기 종료 |

### 2.2 타이밍 파라미터

| 항목 | 값 | 비고 |
|:---|:---:|:---|
| 녹색 신호 시간 | 25 cycle | `T_GREEN` |
| 황색 신호 시간 | 4 cycle | `T_YELLOW` |

> 시뮬레이션 편의를 위해 cycle 단위로 설정. 실제 적용 시
> `클럭 주파수 × 목표 시간`으로 파라미터를 환산한다.
> 예) 50MHz, 녹색 25초 → `T_GREEN = 1,250,000,000`

### 2.3 입출력 포트

| 포트 | 방향 | 폭 | 설명 |
|:---|:---:|:---:|:---|
| `clk` | input | 1 | 시스템 클럭 |
| `rstn` | input | 1 | 비동기 active-low 리셋 |
| `vs` | input | 1 | 부도로 차량 감지 (Vehicle Sensor) |
| `light` | output | 6 | 신호등 출력 `{주도로[R,Y,G], 부도로[R,Y,G]}` |

---

## 3. FSM Architecture

### 3.1 State Diagram

```
            tg && vs                  tr
   ┌──────[ S00 ]──────────→[ S01 ]──────────┐
   │      주도로 녹           주도로 황         │
   │        ▲                                ▼
   │        │ tr                          [ S10 ]
   │        │                             부도로 녹
   │      [ S11 ]                            │
   │      부도로 황 ◄───────────────────────────┘
   └────────────────────  tg || !vs  ───────────
```

### 3.2 상태 정의

| State | 신호 상태 | `light[5:0]` | 지속 시간 |
|:---:|:---|:---:|:---|
| `S00` | 주도로 녹 / 부도로 적 | `001100` | 최소 25 cycle |
| `S01` | 주도로 황 / 부도로 적 | `010100` | 4 cycle (고정) |
| `S10` | 주도로 적 / 부도로 녹 | `100001` | 최대 25 cycle |
| `S11` | 주도로 적 / 부도로 황 | `100010` | 4 cycle (고정) |

### 3.3 전이 조건 — 핵심 설계 결정

| 전이 | 조건 | 논리 | 설계 의도 |
|:---:|:---|:---:|:---|
| S00→S01 | 25cycle 경과 AND 부도로 차량 있으면 | `tg && vs` | 차 없으면 주도로 계속 유지 |
| S01→S10 | 황색 4cycle 경과 | `tr` | 고정 |
| S10→S11 | 25cycle 경과 OR 부도로 차량 없으면 | `tg \|\| !vs` | 차 빠지면 즉시 종료 |
| S11→S00 | 황색 4cycle 경과 | `tr` | 고정 |

> **이 프로젝트의 핵심 — 비대칭 전이 조건**
>
> S00은 AND(`&&`), S10은 OR(`||`) 를 쓴다.
> "주도로 우선 + 부도로 반응형" 정책 전체를 만든다.
> - `S00: tg && vs` → 두 조건이 모두 참이어야 떠남 → 차 없으면 영원히 녹색
> - `S10: tg || !vs` → 둘 중 하나만 참이어도 떠남 → 차 빠지면 조기 종료

---

## �‑4. Implementation

### 4.1 모듈 구조 — 3-Block FSM

```
┌─────────────────┐   nst   ┌──────────────────┐
│ Next-State      │────────→│ State Register   │
│ Logic (comb)    │         │ (sequential)     │
└─────────────────┘◄────────└──────────────────┘
        │            st              │ st
        │ tg, tr                     ↓
        │                   ┌──────────────────┐
        ▼                   │ Counter          │
   (전이 판단)               │ (sequential)     │
                            └──────────────────┘
                                     │ st
                                     ▼
                            ┌──────────────────┐
                            │ Output Logic     │
                            │ (comb) → light   │
                            └──────────────────┘
```

### 4.2 RTL Code

```verilog
module traffic_light(
    input            vs,      // 부도로 차량 감지
    input            clk,
    input            rstn,    // 비동기 active-low 리셋
    output reg [5:0] light    // {주도로 RYG, 부도로 RYG}
);
    parameter st00=2'b00, st01=2'b01, st10=2'b10, st11=2'b11;
    parameter T_GREEN = 24;   // 25 cycle (0~24)
    parameter T_YELLOW = 3;   // 4 cycle  (0~3)

    reg [4:0] cnt;
    reg       tg, tr;
    reg [1:0] st, nst;

    // ── ① State Register + Timer Counter ──
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            st  <= st00;
            cnt <= 0;
        end
        else begin
            st <= nst;
            if (st != nst)             cnt <= 0;       // 상태 전이 → 카운터 리셋
            else if (cnt == T_GREEN)   cnt <= cnt;     // 최대치 도달 → 포화
            else                       cnt <= cnt + 1; // 시간 측정
        end
    end

    // ── ② Next-State Logic ──
    always @(*) begin
        nst = st;
        tg  = (cnt == T_GREEN);   // 녹색 시간 만료
        tr  = (cnt == T_YELLOW);  // 황색 시간 만료

        case (st)
            st00: if (tg && vs)  nst = st01;  // 주도로 우선 (AND)
            st01: if (tr)        nst = st10;
            st10: if (tg || !vs) nst = st11;  // 부도로 반응형 (OR)
            st11: if (tr)        nst = st00;
            default:             nst = st00;
        endcase
    end

    // ── ③ Output Logic (Moore) ──
    always @(*) begin
        case (st)
            st00: light = 6'b001100;
            st01: light = 6'b010100;
            st10: light = 6'b100001;
            st11: light = 6'b100010;
            default: light = 6'b000000;
        endcase
    end
endmodule
```

### 4.3 핵심 설계 결정 (Design Decisions)

#### ① 카운터 리셋 = 상태 전이 (`st != nst`)
카운터의 역할은 "현재 상태에 얼마나 머물렀는가"의 측정이다.
별도 리셋 신호 대신 상태 전이 발생 자체(`st != nst`)를 리셋
조건으로 삼아 로직을 단순화했다.

#### ② 카운터 포화 (Saturation)
```verilog
else if (cnt == T_GREEN) cnt <= cnt;
```
S00에서 부도로 차량이 없으면 25cycle이 지나도 전이하지 않는다.
이때 카운터가 계속 증가하면 `tg = (cnt == T_GREEN)` 조건이
한 순간만 참이 되었다가 깨진다. → 차량이 와도 전이 불가.

카운터를 최대치에서 포화시켜 `tg`를 안정적으로 유지함으로써,
차량 감지 즉시 전이가 가능하도록 보장했다.

#### ③ Reset / 카운터 로직 분리
Reset 갈래와 카운터 갱신 갈래를 `if-else`로 명확히 분리하여
동일 레지스터(`cnt`)에 대한 다중 할당 충돌을 방지했다.

---

## 5. Verification

### 5.1 검증 전략

단순 순환 동작뿐 아니라, 설계의 핵심인 차량 감지 반응 로직을
집중 검증하도록 6개 시나리오를 구성했다.

| # | 시나리오 | 검증 목표 | 결과 |
|:-:|:---|:---|:---:|
| 1 | 주도로 녹색 + 차량 없음 | 25cycle 초과해도 녹색 유지 | ✅ |
| 2 | 부도로 차량 등장 | 즉시 황색→부도로 전이 | ✅ |
| 3 | 부도로 녹색 중 차량 이탈 | 25cycle 전 조기 종료 | ✅ |
| 4 | 차량 상시 존재 | 4-state 정상 순환 | ✅ |
| 5 | 부도로 차량 지속 | 부도로 녹색 최대 시간 사용 | ✅ |
| 6 | 동작 중 리셋 | S00로 안전 복귀 | ✅ |



### 5.2 핵심 시나리오 파형 분석

Case 1 — 주도로 우선 (Main Road Priority)
```
vs=0 유지 │ cnt: 0→1→...→24 도달 후 24에서 포화
          │ st = S00 유지, light = 001100 변화 없음
→ 부도로 차량이 없는 한 주도로 녹색은 무한 유지된다. 
```

Case 3 — 부도로 조기 종료 (Side Road Early Exit)
```
st = S10 (부도로 녹) 진입 │ cnt = 5 시점에 vs=0 발생
                          │ 25cycle 만료 전이지만 즉시 S11로 전이
→ 차량이 빠지면 불필요한 대기 없이 신호가 전환된다. 
```

### 5.3 Testbench

시나리오 기반 자극(stimulus) 생성, `$monitor`로 상태·카운터·출력을
실시간 추적. 내부 신호(`u0.st`, `u0.cnt`)를 계층적 참조로 관찰하여
FSM 전이 흐름을 검증했다.

```

traffic_light/
├── traffic_light.sv      # RTL 설계
└── tb_traffic_light.sv   # 6-scenario testbench
```

---

## 6. Tools & Environment

| 항목 | 내용 |
|:---|:---|
| HDL | Verilog (IEEE 1364) |
| Simulator | EDA Playground (Aldec Riviera-PRO) |
| Waveform | EPWave |
| Verification | Scenario-based directed testbench |

---

## 7. What I Learned

- 타이머 기반 FSM 설계 — 카운터와 FSM을 결합한 시간 제어 회로
- 비대칭 전이 조건 — AND/OR 하나의 차이로 정책을 구현하는 RTL적 사고
- 카운터 포화의 필요성 — 전이 조건 신호의 안정적 유지
- 시나리오 기반 검증 — 정상 동작뿐 아니라 설계 의도(우선권/조기종료)를
  직접 겨냥한 테스트 케이스 구성

---


> 본 프로젝트는 RTL 설계 및 검증 역량 학습을 위한 개인 프로젝트입니다.
