# Phaser Pharaohs – High-Resolution FMCW Radar (Radar Challenge 2025)

## Overview  
This repository contains the open-source project files from **Team Phaser Pharaohs** for the **IEEE AESS Radar Challenge 2025**.  
We modified the **Analog Devices CN0566 Phaser** platform to achieve **5 cm range resolution** for **non-destructive testing (NDT)**.  

---

## Motivation  
- provide a **non-destructive, high-resolution radar imaging tool**.  

---

## Key Achievements  
- Expanded usable radar **bandwidth from 500 MHz → 3 GHz**  
- Achieved **5 cm range resolution** and resolved **7 cm target separations**  
- Designed and fabricated custom **Horn TX** and **Vivaldi RX** antennas  
- Implemented **near-field focused beamforming** for sub-meter imaging  
- Demonstrated imaging on **sandbox rods** and **RAAC samples**  

---

## Hardware Modifications  
Full details are in [`docs/Modifications.docx`](docs/Modifications.docx).  
Summary of changes:  

- **Filters**  
  - Replaced *LFCW-1062+* with *LFCW-1142+* (TX/RX path)  
  - Removed *BFCN-1052+* on TX path (override the filter)
  - Optional IF bandpass filter for SNR improvement  
- **Local Oscillator**: HMC735 VCO extended to **9.5–12.6 GHz**  
- **Antennas**  
  - **Vivaldi RX Array**  
    - Fabrication substrate: **Rogers RT/duroid 5880, 0.062” (1.575 mm)**  
    - Layout provided in **ODB++ format**  
  - **Horn TX Antennas** (12 dBi and 18 dBi)  
    - CAD models in **STL format**  
    - SMA connector: **MC002927**  
    - Inner pin length: **7 mm** inside waveguide (coax-to-waveguide transition)  
- **Routing**: RF rerouted to MMCX connectors for external antenna interfacing  

**Result:** Operation extended to **8–11 GHz** with ~5 cm resolution, optimized for high-resolution short-range imaging.  

---

## Repository Structure  
```
├── hardware/          
│   ├── Horn antennas/       # STL models for 12 dBi and 18 dBi horn antennas
│   └── vivaldi antenna/     # ODB++ files for Vivaldi PCB antenna
├── software/                # Python scripts for data acquisition & processing
├── experiments/             # Test datasets (sandbox rods, RAAC panels)
├── docs/                    # Challenge documentation and presentations
│   ├── PhaserPharaohs_Presentation.pptx
│   └── Modifications.docx
└── README.md
```

---

## Installation & Usage  

### Requirements  
- CN0566 Phaser kit (modified)  
- ADALM-PLUTO SDR  
- Python ≥ 3.9 with:  
  ```
  numpy
  matplotlib
  scipy
  pyadi-iio
  ```

### Example Run  
```bash
git clone https://github.com/<your-org>/phaser-pharaohs-radar.git
cd phaser-pharaohs-radar/software

# Acquire test data
python acquire_data.py

# Process and plot results
python process_results.py
```

---

## Results  
- **Sandbox test**: resolved two rods at **7–10 cm separation**  
- **RAAC sample**: imaged hidden steel reinforcement with focused beamforming  
- Achieved **6× resolution improvement** over stock Phaser configuration  

---

## Future Work  
- 3D reconstruction using antenna array + translational motion  
- SAR/ISAR extensions for larger structures  
- Applications in **bridges, tunnels, and aerospace composites**  

---

## References  
- [Analog Devices CN0566 Phaser](https://www.analog.com/cn0566)  
- [IEEE AESS Radar Challenge 2025](https://ieee-aess.org/radar-challenge)  
- [RadarConf 2025 – Krakow](https://radarconf2025.org/)  

---

## License  
Released under the **MIT License** in accordance with the Radar Challenge open-source requirement.  
