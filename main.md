# DIY PCVR Brick — Hardware Specification

## Общее описание
Компактный "кирпичик" (VR‑блок сенсоров) для PCVR с USB‑подключением к ПК. Состоит из SLAM OV9281, hand‑tracking OV2659, IMU ICM20948, Arduino Pro Micro. **Один USB‑C кабель** (данные USB 3 + питание). **Все камеры ≥120fps**. Монтируется на любой VR‑каркас с помощью крабиков‑зажимов.

## v0.1 — MVP (прототип)
**Цель:** доказать компоновку и USB‑агрегацию.  
**Габариты:** 96(W) x 29(D) x 56(H) мм.  
**Масса:** ~80–100 г.  
**Питание:** внешний адаптер 5V/3A (DC‑jack).  
**Кабель:** 1x USB‑C 3.2 Gen1 (данные).  

### Сенсоры
- **2x OV9281 SLAM** (38x38mm):  
  Pos: L=(-22,-11.5,10), R=(22,-11.5,10).  
  Yaw=±28°, Pitch_down=5°.  
  FOV=130° HFOV / 95° VFOV, ≥120fps.  
- **1x OV2659 Hand** (60x8x4.5mm):  
  Pos=(0,-10.5,-20).  
  Yaw=0°, Pitch_down=20°.  
  FOV=120° HFOV / 85° VFOV, ≥120fps.  
- **ICM20948 IMU** (15.5x25.5x3mm):  
  Pos=(0,6.5,-2).  

### Логика
- **Arduino Pro Micro** (34.6x18.4x4mm): Pos=(0,8,16.5).  
- **USB Hub:** VIA VL817 (4 порта USB 3.1 Gen1).  
  Down1=OV9281L, Down2=OV9281R, Down3=OV2659, Down4=Pro Micro.  

**Ток общий:** ~950mA пик, питание извне.  

## v1.0 — Production Prototype
**Цель:** стабильная работа, единственный кабель.  
**Габариты:** 100x32x60mm (+margins).  
**Масса:** ~100–120 г.  
**Питание:** USB‑C 5V/3A (Type‑C pull‑down 5.1kΩ CC).  
**Кабель:** 1x USB‑C 3.2 Gen1 (данные + питание).  

### Сенсоры
- **2x OV9281 SLAM** (как v0.1).  
- **2x OV2659 Hand** (добавлена 2-я):  
  Pos1=(0,-10.5,-20), Pos2=(0,-10.5,-8).  
  Yaw=0°, Pitch_down=20°.  
  FOV=120° HFOV / 85° VFOV, ≥120fps каждая.  
- **ICM20948 IMU** (как v0.1).  

### Логика
- **Arduino Pro Micro** (как v0.1).  
- **USB Hub:** VIA VL817 (расширено до 5 портов).  
  Down1–2=OV9281, Down3–4=OV2659, Down5=Pro Micro.  
- **Type‑C:** 5.1kΩ CC1/CC2 pull‑down (UFP 3A).  
- **Защита:** TPS25940 eFuse на портах, TPD2EUSB44 ESD.  

**Ток общий:** ~1.2A пик (4 камеры), от ПК USB‑C.  

## v2.0 — Optimized Production
**Цель:** серия, минимальный размер.  
**Габариты:** 92x28x54mm.  
**Масса:** ~70 г.  
**Питание:** USB‑C 5V/5A (PD sink CYPD3177).  
**Кабель:** 1x USB‑C 3.2 Gen2 (10Gbps + PD).  

### Сенсоры
- **2x OV9281 SLAM** (оптимизировано).  
- **2x OV2659 Hand** (как v1.0).  
- **ICM20948 IMU** (интегрировано).  

### Логика
- **Custom USB 3.2 Gen2 hub** (RTS5420/FL5001).  
- **Integrated Type‑C/PD** (GL3590).  
- **Custom PCB** с трассировкой кабелей, теплоотводом.  

**Ток общий:** до 2.5A (full load).  

## USB‑Архитектура (все версии)
```
PC ── USB-C ── Type-C receptacle
            │
            ├── CC pull-down (v1:5.1kΩ 3A, v2:PD 5A)
            ├── VBUS → fuse → 5V_HUB → hub + eFuse → камеры/Pro Micro
            └── USB3 → VL817 upstream
VL817 downstream:
├── 2x OV9281 (micro-B)
├── 1–2x OV2659 (micro-B)  
└── Pro Micro (USB2 header)
```
**Защита:** TPD2EUSB44 ESD, TPS25940 eFuse.  
**Резерв:** cable zone 28x8x12mm.  

## Монтаж
Крабики‑зажимы на любой VR‑каркас. M2.5 стойки внутри. Передняя маска с окнами линз.  

**Требования к ПК:** USB‑C 3A+ порт (90% современных).  
**Риски:** ток 900mA (10% ПК), тепло hub.