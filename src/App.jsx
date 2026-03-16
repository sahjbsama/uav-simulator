import { useState, useMemo, useCallback } from "react";
import {
  LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip,
  ResponsiveContainer, ReferenceLine, AreaChart, Area,
  RadarChart, Radar, PolarGrid, PolarAngleAxis, PolarRadiusAxis,
  Legend, PieChart, Pie, Cell
} from "recharts";

// ═══════════════════════════════════════════════════════════════
// PHYSICAL CONSTANTS
// ═══════════════════════════════════════════════════════════════
const g    = 9.80665;
const RHO0 = 1.225;
const T0   = 288.15;   // ISA sea-level temp K

// ─── ISA + NON-STANDARD ATMOSPHERE ─────────────────────────────
function rhoAlt(h, dT = 0) {
  // dT: temperature offset from ISA standard (°C or K, same delta)
  const T_base = T0 + dT;
  const L = 0.0065;
  if (h <= 11000) {
    const T = T_base - L * h;
    const P = 101325 * Math.pow(T / T_base, g / (L * 287.058));
    return P / (287.058 * T);
  }
  const T11 = T_base - L * 11000;
  const P11 = 101325 * Math.pow(T11 / T_base, g / (L * 287.058));
  const T   = T11;
  const P   = P11 * Math.exp(-g * (h - 11000) / (287.058 * T11));
  return P / (287.058 * T);
}

// ─── GLAUERT INDUCED VELOCITY (Newton-Raphson, 5 iter) ──────────
function calcVh(T, rho, A) {
  return Math.sqrt(T / (2 * rho * Math.max(A, 1e-9)));
}
function calcVi(V, vh) {
  if (V < 0.3) return vh;
  let vi = vh / Math.sqrt(1 + (V / (2 * vh)) ** 2);
  for (let n = 0; n < 5; n++) {
    const r  = Math.sqrt(V * V + vi * vi);
    const f  = vi * vi * r - vh * vh;
    const df = 2 * vi * r + vi * vi * vi / r;
    vi = Math.max(vi - f / Math.max(df, 1e-10), 0.001);
  }
  return Math.max(vi, 0.001);
}

// ─── GROUND EFFECT (Cheeseman-Bennett 1955 + IGE correction) ───
function calcGE(alt, prop_d) {
  if (alt > 2.5 * prop_d) return 1.0;
  const R  = prop_d / 2;
  const h  = Math.max(alt, 0.02);
  const hR = h / R;
  // Extended model: more accurate below h/R < 1
  if (hR < 1.0) {
    // Hayden 1976 curve fit (more accurate near ground)
    return Math.max(0.40, 1.0 - 0.115 * Math.pow(2 * hR, -0.57));
  }
  return 1.0 / (1 + (R / (4 * h)) ** 2);
}

// ─── BVI PENALTY (Blade-Vortex Interaction) ──────────────────────
// At advance ratio μ ≈ 0.1–0.2, rotor blades encounter their own vortices
// μ = V / (Ω·R) ≈ V / tip_speed
// Peak penalty at μ ≈ 0.12, resolves above μ ≈ 0.25
function calcBVI(V, tip_speed) {
  const mu = V / Math.max(tip_speed, 1);
  if (mu < 0.05 || mu > 0.30) return 1.0;
  // Bell-shaped penalty: peak +6% at μ=0.12
  const mu_peak = 0.12;
  const bvi = 1.0 + 0.06 * Math.exp(-Math.pow((mu - mu_peak) / 0.06, 2));
  return bvi;
}

// ─── DESCENT POWER (vortex ring state) ───────────────────────────
// Leishman: in descent, P can INCREASE due to vortex ring state
// Descent rate -1 to -4 m/s is dangerous zone
function descentFactor(climb_ms) {
  if (climb_ms >= 0) return 1.0;
  const vd = -climb_ms; // positive descent rate
  // Vortex ring state: max penalty at ~2 m/s descent (~0.5 vh)
  // Simplified: +20% power penalty between -0.5 and -3 m/s
  if (vd < 0.5) return 1.0;
  if (vd < 3.0) return 1.0 + 0.20 * Math.sin(Math.PI * (vd - 0.5) / 2.5);
  return 1.0; // autorotation regime, power reduces
}

// ─── FM vs DISK LOADING (empirical, Leishman) ────────────────────
// Higher disk loading → lower FM (tips operate at higher Re but vortex losses grow)
// FM_DL = FM_ref * (1 - 0.08 * log10(DL / DL_ref))
// DL_ref = 50 N/m² (typical consumer drone)
function fmDiskLoading(FM0, DL) {
  const DL_ref = 50;
  return FM0 * Math.max(0.85, 1 - 0.06 * Math.log10(Math.max(DL / DL_ref, 0.1)));
}

// ─── FM(V) + κ(V) ────────────────────────────────────────────────
function fmEff(FM0, V) {
  return FM0 * Math.max(0.55, 1 - 0.28 * Math.pow(Math.min(V, 25) / 20, 1.5));
}
function kappaV(V) {
  return 1.0 + 0.04 * Math.pow(V / 10, 1.5);
}

// ─── δ(V) PROFILE POWER ──────────────────────────────────────────
function deltaEff(d0, V) {
  return Math.min(d0 * (1 + 1.4 * Math.pow(V / 10, 2.0)), d0 * 4.0);
}

// ─── CdA(V) BLEND ────────────────────────────────────────────────
// Wind effect: headwind increases effective airspeed
function cdaEff(CdA_h, CdA_f, V) {
  const w = 1 - Math.exp(-V / 7);
  return CdA_h * (1 - w) + CdA_f * w;
}

// ─── WIND MODEL ──────────────────────────────────────────────────
// wind_speed (m/s), wind_dir (0=headwind, 90=crosswind, 180=tailwind)
// heading assumed into desired flight direction
function effectiveAirspeed(V_ground, wind_speed, wind_dir_deg) {
  const wd = wind_dir_deg * Math.PI / 180;
  // Component along flight path (headwind = positive)
  const V_wind_axial = wind_speed * Math.cos(wd);
  const V_air = Math.max(0, V_ground + V_wind_axial);
  // Crosswind component (increases body drag)
  const V_cross = wind_speed * Math.abs(Math.sin(wd));
  return { V_air, V_cross };
}

// ─── MOTOR MODEL (Kv, Rm, I0) ────────────────────────────────────
// Based on standard brushless motor equations:
//   Back-EMF: E = Kv_rpm * RPM / 1000  (in some conventions)
//   Actually: RPM = Kv * (V_batt - I*Rm - E_esc_drop)
//   Shaft power: P_shaft = (V - I*Rm) * I - V * I0
//   Efficiency: η = P_shaft / P_electrical
// Simplified: given P_total_mech, find η_motor
function motorEfficiency(drone, P_mech_W) {
  // P_mech per motor
  const n_m = drone.num_rotors;
  const P_per = P_mech_W / n_m;
  const V_nom = drone.cell_count * 3.7;
  const Kv    = drone.motor_kv || 1500;       // RPM/V
  const Rm    = drone.motor_rm || 0.08;        // Ohm, winding resistance
  const I0    = drone.motor_i0 || 0.5;         // A, no-load current
  // Estimate current from power (iterative would be more accurate)
  // I ≈ P_per / (V_nom * eta_guess)
  const I_est = Math.max(P_per / (V_nom * 0.80), I0 + 0.1);
  // Copper loss
  const P_copper = I_est * I_est * Rm;
  // No-load loss
  const P_noload = I0 * V_nom;
  // Efficiency
  const eta = Math.max(0.50, Math.min(0.96,
    (P_per - P_copper - P_noload) / Math.max(P_per, 1)
  ));
  return eta;
}

// ─── BATTERY VOLTAGE SAG MODEL ────────────────────────────────────
// V_sag = V_nominal - I * R_internal
// R_internal ≈ (1/C_Ah) * r_cell_factor  (Rint grows at high C-rate)
// Temperature effect: Rint increases by ~1.5% per °C below 25°C
function batteryVoltage(drone, P_W, temp_C = 25) {
  const V_nom   = drone.cell_count * 3.7;
  const I       = P_W / Math.max(V_nom, 0.1);
  const C_Ah    = drone.battery_Wh / V_nom;
  // Internal resistance per cell (typical LiPo: 5-15 mΩ/Ah)
  const r_cell  = (drone.bat_ir_mohm_per_Ah || 8) / 1000;  // Ω/Ah
  const R_int   = r_cell / C_Ah;  // total internal resistance
  // Temperature derating of Rint (+2% per °C below 25°C)
  const T_factor = 1 + Math.max(0, (25 - temp_C)) * 0.02;
  const R_eff    = R_int * T_factor;
  const V_sag    = I * R_eff;
  const V_actual = Math.max(V_nom * 0.6, V_nom - V_sag);
  // Temperature also derate capacity
  const cap_derate = temp_C < 10 ? 0.85 : temp_C < 20 ? 0.93 : 1.0;
  const sag_pct  = (V_sag / V_nom) * 100;
  return { V_actual, V_sag, sag_pct, cap_derate, I, R_eff };
}

// ─── ESC THERMAL MODEL ────────────────────────────────────────────
// ESC heats up with I²R loss; above threshold → derating
// P_esc_loss = I² * R_esc
// T_esc = T_ambient + P_loss * R_thermal
// Derating: if T > 80°C → reduce max power by 2%/°C
function escThermal(drone, P_W, temp_ambient = 25) {
  const V_nom   = drone.cell_count * 3.7;
  const I       = P_W / Math.max(V_nom, 0.1) / drone.num_rotors;
  const R_esc   = drone.esc_r_mohm ? drone.esc_r_mohm / 1000 : 0.005; // Ω
  const R_th    = 8;   // °C/W thermal resistance (typical 30A ESC, no airflow)
  const P_loss  = I * I * R_esc;
  const T_esc   = temp_ambient + P_loss * R_th;
  // Derating above 80°C
  const derate  = T_esc > 80 ? Math.max(0.70, 1 - (T_esc - 80) * 0.02) : 1.0;
  return { T_esc, derate, I, P_loss };
}

// ─── NOISE ESTIMATE (dBA) ─────────────────────────────────────────
// Based on disk loading and tip speed (simplified Ffowcs-Williams model)
// SPL ≈ 20*log10(Vtip) + 10*log10(DL) + K_drone + distance_corr
// K_drone: consumer ~85, racing ~95, enterprise ~88
function noiseEstimate(drone, mass_kg, rho, dist_m = 5) {
  const DL  = mass_kg * g / (drone.num_rotors * Math.PI * (drone.prop_d / 2) ** 2);
  const tip = drone.tip_speed || 160;
  const K   = drone.noise_K   || 88;
  const SPL_src = K + 15 * Math.log10(tip / 150) + 8 * Math.log10(DL / 50);
  // Distance attenuation: -20dB per decade
  const SPL = SPL_src - 20 * Math.log10(Math.max(dist_m, 1));
  return Math.max(30, SPL);
}

// ─── PEUKERT + HCR (enhanced) ─────────────────────────────────────
function peukertEnhanced(drone, P_W, temp_C = 25) {
  const bat    = batteryVoltage(drone, P_W, temp_C);
  const V_nom  = drone.cell_count * 3.7;
  const I      = bat.I;
  const C_Ah   = drone.battery_Wh / V_nom;
  const k      = drone.peukert_k;
  const ratio  = Math.min(Math.max(C_Ah / Math.max(I, 0.01), 0.05), 5.0);
  const pk     = ratio ** (k - 1);
  const Cr     = I / Math.max(C_Ah, 0.01);
  const hcr    = Cr > 20 ? 0.60 : Cr > 15 ? 0.72 : Cr > 10 ? 0.85 : 1.0;
  // Temperature derating of usable capacity
  const E_eff  = drone.battery_Wh * drone.dod * drone.eta_bat * pk * hcr * bat.cap_derate;
  return { E_eff, Cr, hcr, pk, sag_pct: bat.sag_pct, V_actual: bat.V_actual };
}

// ─── η_motor (load-dependent) ─────────────────────────────────────
function etaMotorEff(eta_base, P_frac) {
  return eta_base * Math.max(0.60, 0.65 + 0.35 * Math.sqrt(Math.min(P_frac, 1)));
}

// ═══════════════════════════════════════════════════════════════
// MAIN POWER MODEL (v8 — all effects integrated)
// ═══════════════════════════════════════════════════════════════
function calcPower(drone, V, mass, alt, climb, env = {}) {
  const {
    wind_speed = 0,
    wind_dir   = 0,    // 0=headwind
    temp_dT    = 0,    // ISA delta-T (°C)
    temp_C     = 25,   // ambient Celsius
    payload_kg = 0,    // extra payload
  } = env;

  const total_mass = mass + payload_kg;
  const rho   = rhoAlt(alt, temp_dT);
  const T     = total_mass * g;
  const A     = drone.num_rotors * Math.PI * (drone.prop_d / 2) ** 2;
  const DL    = T / A;

  // FM with disk loading correction (Leishman)
  const FM0_dl = fmDiskLoading(drone.FM, DL);

  // Wind: effective airspeed
  const { V_air, V_cross } = effectiveAirspeed(V, wind_speed, wind_dir);

  const vh    = calcVh(T, rho, A);
  const vi    = calcVi(V_air, vh);
  const ge    = calcGE(alt, drone.prop_d);
  const FM    = fmEff(FM0_dl, V_air);
  const kap   = kappaV(V_air);
  const bvi   = calcBVI(V_air, drone.tip_speed || 160);
  const desc  = descentFactor(climb);
  const dlt   = deltaEff(drone.delta_profile, V_air);

  // CdA with crosswind contribution
  const CdA_base   = cdaEff(drone.CdA_h, drone.CdA_f, V_air);
  const CdA_cross  = drone.CdA_f * 1.5 * (V_cross > 0 ? (V_cross / Math.max(V_air + 1, 1)) ** 2 : 0);
  const CdA        = CdA_base + CdA_cross;

  // Payload drag increase
  const CdA_pay = payload_kg > 0 ? payload_kg * 0.01 : 0;

  // Load fraction
  const P_frac = Math.min(0.40 + V_air / (drone.max_v * 2), 1.0);
  const eta_m  = etaMotorEff(drone.eta_motor, P_frac);
  const eta    = eta_m * drone.eta_esc;

  // ESC thermal derating
  const esc    = escThermal(drone, T * vi / eta * 1.2, temp_C);
  const eta_final = eta * esc.derate;

  // Hover reference (for profile drag scaling)
  const eta_hov  = etaMotorEff(drone.eta_motor, 0.40) * drone.eta_esc;
  const P_href   = T * vh / (FM0_dl * eta_hov);

  // Power components
  const P_ind  = (T * vi * kap * ge * bvi * desc) / (FM * eta_final);
  const P_pro  = dlt * P_href;
  const P_par  = V_air > 0.05 ? (0.5 * rho * (CdA + CdA_pay) * V_air ** 3) / eta_final : 0;
  // Climb: positive = climbing, negative = descending
  const P_clm  = climb > 0
    ? (T * climb) / eta_final
    : (T * climb) / eta_final * 0.4; // recovery in descent
  const P_sys  = drone.P_sys;

  // Motor model efficiency check
  const eta_motor_actual = motorEfficiency(drone, P_ind + P_pro + P_par);

  const P_total = Math.max(P_sys,
    P_ind + P_pro + P_par + Math.max(0, P_clm) + P_sys
  );

  // Derived metrics
  const mu = V_air / (drone.tip_speed || 160);
  const TW = T / (P_total / Math.max(V_air, 0.5) + 0.001);

  return {
    P_total, P_href, P_ind, P_pro, P_par, P_clm,
    vh, vi, ge, FM, kap, dlt, CdA, bvi, desc,
    rho, A, DL, eta: eta_final, eta_m, T,
    mu, bvi, esc_T: esc.T_esc, esc_derate: esc.derate,
    FM0_dl,
  };
}

// ═══════════════════════════════════════════════════════════════
// FLIGHT TIME (minutes) — enhanced with voltage sag + temp
// ═══════════════════════════════════════════════════════════════
function flightMin(drone, P_W, env = {}) {
  const { temp_C = 25 } = env;
  const pek = peukertEnhanced(drone, P_W, temp_C);
  return { t: (pek.E_eff / Math.max(P_W, 0.1)) * 60, ...pek };
}

// ─── NOISE ────────────────────────────────────────────────────────
function droneNoise(drone, mass_kg, rho) {
  return noiseEstimate(drone, mass_kg, rho);
}

function diskLoad(drone, mass) {
  return (mass * g) / (drone.num_rotors * Math.PI * (drone.prop_d / 2) ** 2);
}

// ═══════════════════════════════════════════════════════════════
// DRONE PRESETS (v8 — extended with motor + battery params)
// ═══════════════════════════════════════════════════════════════
const DRONES = {
  mini4pro: {
    name:"DJI Mini 4 Pro", label:"Consumer · 249g",
    color:"#00ff99",       gen:"Consumer",
    num_rotors:4, prop_d:0.149, tip_speed:148,
    mass_kg:0.249, battery_Wh:34, cell_count:2,
    eta_motor:0.72, eta_esc:0.94, eta_bat:0.97,
    FM:0.62, delta_profile:0.12,
    CdA_h:0.022, CdA_f:0.004,
    P_sys:34.0, dod:0.95, peukert_k:1.04,
    motor_kv:1800, motor_rm:0.12, motor_i0:0.4,
    bat_ir_mohm_per_Ah:10, esc_r_mohm:6,
    noise_K:82, max_v:21, max_alt:4000,
    real_hover_min:30, real_cruise_v:10, real_cruise_min:40,
  },
  fpv5inch: {
    name:"5\" FPV Racing", label:"Racing · 650g · 6S",
    color:"#ff5555",       gen:"Racing",
    num_rotors:4, prop_d:0.127, tip_speed:220,
    mass_kg:0.650, battery_Wh:33.3, cell_count:6,
    eta_motor:0.72, eta_esc:0.93, eta_bat:0.92,
    FM:0.48, delta_profile:0.18,
    CdA_h:0.070, CdA_f:0.010,
    P_sys:20.0, dod:0.80, peukert_k:1.16,
    motor_kv:2450, motor_rm:0.055, motor_i0:0.8,
    bat_ir_mohm_per_Ah:14, esc_r_mohm:3,
    noise_K:97, max_v:35, max_alt:1500,
    real_hover_min:5, real_cruise_v:20, real_cruise_min:null,
  },
  matrice300: {
    name:"DJI Matrice 300", label:"Enterprise · 6.3kg",
    color:"#ffbb33",        gen:"Enterprise",
    num_rotors:6, prop_d:0.533, tip_speed:140,
    mass_kg:6.3, battery_Wh:474, cell_count:12,
    eta_motor:0.91, eta_esc:0.97, eta_bat:0.97,
    FM:0.82, delta_profile:0.10,
    CdA_h:0.28, CdA_f:0.04,
    P_sys:5.0, dod:0.92, peukert_k:1.05,
    motor_kv:380, motor_rm:0.035, motor_i0:1.2,
    bat_ir_mohm_per_Ah:5, esc_r_mohm:2,
    noise_K:86, max_v:23, max_alt:5000,
    real_hover_min:55, real_cruise_v:14, real_cruise_min:null,
  },
  combat7: {
    name:"7\" Combat FPV", label:"Tactical · 1.2kg · 6S",
    color:"#ff8833",       gen:"Tactical",
    num_rotors:4, prop_d:0.178, tip_speed:185,
    mass_kg:1.2, battery_Wh:89, cell_count:6,
    eta_motor:0.70, eta_esc:0.95, eta_bat:0.93,
    FM:0.50, delta_profile:0.16,
    CdA_h:0.095, CdA_f:0.013,
    P_sys:16.0, dod:0.80, peukert_k:1.12,
    motor_kv:1500, motor_rm:0.08, motor_i0:0.6,
    bat_ir_mohm_per_Ah:12, esc_r_mohm:4,
    noise_K:93, max_v:30, max_alt:2000,
    real_hover_min:10, real_cruise_v:15, real_cruise_min:null,
  },
};

const CUSTOM_DEF = {
  name:"Custom Design", label:"User Config",
  color:"#bb66ff",      gen:"Custom",
  num_rotors:4, prop_d:0.200, tip_speed:170,
  mass_kg:1.0, battery_Wh:200, cell_count:4,
  eta_motor:0.78, eta_esc:0.94, eta_bat:0.95,
  FM:0.62, delta_profile:0.14,
  CdA_h:0.10, CdA_f:0.020,
  P_sys:15.0, dod:0.85, peukert_k:1.08,
  motor_kv:1200, motor_rm:0.10, motor_i0:0.5,
  bat_ir_mohm_per_Ah:9, esc_r_mohm:5,
  noise_K:88, max_v:25, max_alt:3000,
  real_hover_min:null, real_cruise_v:12, real_cruise_min:null,
};

// ─── UI HELPERS ─────────────────────────────────────────────────
const TTip = ({ active, payload, label }) => {
  if (!active || !payload?.length) return null;
  return (
    <div style={{ background:"rgba(1,5,14,0.97)", border:"1px solid #ffffff14",
      padding:"9px 13px", borderRadius:4,
      fontFamily:"'Share Tech Mono',monospace", fontSize:10 }}>
      <div style={{ color:"#6688aa", marginBottom:4, fontWeight:"bold" }}>V = {label} m/s</div>
      {payload.map((p,i) => (
        <div key={i} style={{ color:p.color, marginBottom:2 }}>
          {p.name}:<span style={{ color:"#fff", fontWeight:"bold", marginLeft:3 }}>
            {typeof p.value==="number" ? p.value.toFixed(3) : p.value}
          </span>
        </div>
      ))}
    </div>
  );
};

const SC = ({ label, val, unit, color="#00ff99", sub, warn }) => (
  <div style={{ background:`${warn?warn:color}0e`, border:`1px solid ${warn?warn:color}38`,
    borderLeft:`4px solid ${warn?warn:color}`, padding:"9px 11px", borderRadius:3 }}>
    <div style={{ fontSize:8, color:"#5577aa", letterSpacing:2, marginBottom:2 }}>{label}</div>
    <div style={{ fontFamily:"'Orbitron',sans-serif", fontWeight:900, fontSize:19,
      color:warn||color, lineHeight:1 }}>
      {val}<span style={{ fontSize:9, color:"#334455", marginLeft:3 }}>{unit}</span>
    </div>
    {sub && <div style={{ fontSize:7, color:warn?"#aa6644":"#334455", marginTop:3, lineHeight:1.5 }}>{sub}</div>}
  </div>
);

const Sldr = ({ label, val, min, max, step=1, unit, set, color, sub }) => (
  <div style={{ background:"rgba(3,10,22,0.85)", border:"1px solid #0b1c2c",
    borderLeft:`4px solid ${color}`, padding:"9px 11px", borderRadius:3 }}>
    <div style={{ display:"flex", justifyContent:"space-between", marginBottom:4 }}>
      <span style={{ fontSize:8, color:"#4466aa", letterSpacing:2 }}>{label}</span>
      <span style={{ fontSize:13, color, fontFamily:"'Orbitron',sans-serif", fontWeight:900 }}>
        {step<1 ? val.toFixed(step<0.01?3:step<0.1?2:1) : val}
        <span style={{ fontSize:7, marginLeft:2, color:"#334455" }}>{unit}</span>
      </span>
    </div>
    <input type="range" min={min} max={max} step={step} value={val}
      onChange={e => set(+e.target.value)}
      style={{ width:"100%", accentColor:color, height:3 }} />
    {sub && <div style={{ fontSize:6, color:"#223344", marginTop:2, textAlign:"center" }}>{sub}</div>}
  </div>
);

// Gauge component (circular indicator)
const Gauge = ({ value, max, label, color, unit }) => {
  const pct = Math.min(value / max, 1);
  const r = 28, cx = 36, cy = 36;
  const circumference = 2 * Math.PI * r * 0.75;
  const offset = circumference * (1 - pct);
  const startAngle = 135, endAngle = 405;
  const pathD = (rx) => {
    const sa = (startAngle - 90) * Math.PI / 180;
    const ea = (endAngle - 90) * Math.PI / 180;
    const x1 = cx + rx * Math.cos(sa), y1 = cy + rx * Math.sin(sa);
    const x2 = cx + rx * Math.cos(ea), y2 = cy + rx * Math.sin(ea);
    return `M ${x1} ${y1} A ${rx} ${rx} 0 1 1 ${x2} ${y2}`;
  };
  return (
    <div style={{ textAlign:"center", width:72 }}>
      <svg width={72} height={56} viewBox="0 0 72 56">
        <path d={pathD(r)} fill="none" stroke="#0b1c2c" strokeWidth={5} strokeLinecap="round"/>
        <path d={pathD(r)} fill="none" stroke={color} strokeWidth={5} strokeLinecap="round"
          strokeDasharray={circumference} strokeDashoffset={offset}
          style={{ transition:"stroke-dashoffset .4s" }}/>
        <text x={cx} y={cy+2} fill={color} fontSize={11}
          fontFamily="'Orbitron',sans-serif" fontWeight={900} textAnchor="middle">{
          value > 99 ? Math.round(value) : value.toFixed(1)
        }</text>
        <text x={cx} y={cy+13} fill="#334455" fontSize={7} textAnchor="middle">{unit}</text>
      </svg>
      <div style={{ fontSize:7, color:"#334455", letterSpacing:1, marginTop:-6 }}>{label}</div>
    </div>
  );
};

// RotorDiag
const RotorDiag = ({ drone, pw }) => {
  const S=114, C=57, r=33, rr=Math.min(15, drone.prop_d*26+4);
  const pts = Array.from({length:drone.num_rotors},(_,i)=>{
    const a=2*Math.PI*i/drone.num_rotors-Math.PI/4;
    return{x:C+r*Math.cos(a),y:C+r*Math.sin(a)};
  });
  const th=Math.min(26,pw.P_total/600*10);
  const geA=pw.ge<0.99;
  const bviWarn=pw.bvi>1.03;
  return (
    <svg viewBox={`0 0 ${S} ${S}`} width={S} height={S}
      style={{filter:"drop-shadow(0 0 4px #00ff9918)"}}>
      {pts.map((p,i)=><line key={i} x1={C} y1={C} x2={p.x} y2={p.y} stroke="#112018" strokeWidth={2}/>)}
      <rect x={C-5} y={C-5} width={10} height={10} rx={2}
        fill="#070e08" stroke={geA?"#ffbb33":bviWarn?"#ff8833":"#00ff99"} strokeWidth={1.5}/>
      {pts.map((p,i)=>(
        <g key={i}>
          <circle cx={p.x} cy={p.y} r={rr} fill="none" stroke="#00ff9922"
            strokeWidth={1} strokeDasharray="3 2"/>
          <circle cx={p.x} cy={p.y} r={2.2} fill="#00ff99" opacity={0.9}/>
        </g>
      ))}
      <line x1={C} y1={C} x2={C} y2={C-th} stroke="#00ff99" strokeWidth={2} strokeLinecap="round"/>
      <polygon points={`${C},${C-th-4} ${C-2},${C-th+2} ${C+2},${C-th+2}`} fill="#00ff99"/>
      <text x={C} y={S-2} fill={geA?"#ffbb33":bviWarn?"#ff8833":"#1a3a2a"} fontSize={5.5} textAnchor="middle">
        {geA?`IGE ${(pw.ge*100).toFixed(0)}%`:bviWarn?"BVI ZONE":"OGE"}
      </text>
    </svg>
  );
};

// ═══════════════════════════════════════════════════════════════
// CONFIG PANEL — copy/paste based (no blob, works in Claude.ai)
// ═══════════════════════════════════════════════════════════════
function ConfigPanel({ drone, dk, spd, alt, climb, payload,
  windSpd, windDir, tempC, tempDT, onImport }) {
  const [open,    setOpen]    = useState(false);
  const [mode,    setMode]    = useState("export"); // "export" | "import"
  const [text,    setText]    = useState("");
  const [status,  setStatus]  = useState("");

  const cfgObj = {
    drone: dk === "custom" ? drone : { _preset: dk },
    spd, alt, climb, payload, windSpd, windDir, tempC, tempDT,
  };

  const handleOpen = (m) => {
    setMode(m);
    setStatus("");
    if (m === "export") setText(JSON.stringify(cfgObj, null, 2));
    else setText("");
    setOpen(true);
  };

  const handleCopy = () => {
    try {
      navigator.clipboard.writeText(text);
      setStatus("✅ Copied to clipboard!");
    } catch {
      setStatus("Select all text above and copy manually (Ctrl+A, Ctrl+C)");
    }
  };

  const handleImport = () => {
    try {
      const parsed = JSON.parse(text);
      onImport(parsed);
      setStatus("✅ Config loaded!");
      setTimeout(() => setOpen(false), 800);
    } catch (e) {
      setStatus("❌ Invalid JSON — check format");
    }
  };

  return (
    <>
      <div style={{ display:"flex", gap:5 }}>
        <button className="btn" onClick={() => handleOpen("export")} style={{
          flex:1, background:"rgba(187,102,255,0.08)", border:"1px solid #bb66ff33",
          padding:"7px 6px", color:"#bb66ff", fontSize:7, letterSpacing:1, borderRadius:3 }}>
          ⬆ EXPORT
        </button>
        <button className="btn" onClick={() => handleOpen("import")} style={{
          flex:1, background:"rgba(51,221,255,0.06)", border:"1px solid #33ddff33",
          padding:"7px 6px", color:"#33ddff", fontSize:7, letterSpacing:1, borderRadius:3 }}>
          ⬇ IMPORT
        </button>
      </div>

      {open && (
        <div style={{ position:"fixed", inset:0, background:"rgba(0,4,12,0.88)",
          display:"flex", alignItems:"center", justifyContent:"center", zIndex:2000 }}>
          <div style={{ background:"#010a16", border:"1px solid #bb66ff44",
            borderTop:"3px solid #bb66ff", borderRadius:6, padding:"20px",
            width:"min(520px, 92vw)", fontFamily:"'Share Tech Mono',monospace" }}>

            <div style={{ display:"flex", justifyContent:"space-between", alignItems:"center", marginBottom:14 }}>
              <div style={{ fontSize:11, color:"#bb66ff", fontFamily:"'Orbitron',sans-serif", fontWeight:700 }}>
                {mode === "export" ? "⬆ EXPORT CONFIG" : "⬇ IMPORT CONFIG"}
              </div>
              <button className="btn" onClick={() => setOpen(false)} style={{
                background:"#bb66ff14", border:"1px solid #bb66ff33",
                padding:"4px 10px", color:"#bb66ff", fontSize:9, borderRadius:2 }}>✕</button>
            </div>

            {mode === "export" && (
              <div style={{ fontSize:8, color:"#4466aa", marginBottom:8, lineHeight:1.6 }}>
                Copy this JSON — paste it later into Import to restore your settings.
              </div>
            )}
            {mode === "import" && (
              <div style={{ fontSize:8, color:"#4466aa", marginBottom:8, lineHeight:1.6 }}>
                Paste a previously exported JSON config here, then click Load.
              </div>
            )}

            <textarea
              value={text}
              onChange={e => setText(e.target.value)}
              readOnly={mode === "export"}
              rows={12}
              style={{
                width:"100%", background:"#00060e", border:"1px solid #0b1c2c",
                color:"#aaccdd", fontFamily:"'Share Tech Mono',monospace",
                fontSize:9, padding:"10px", borderRadius:3,
                resize:"vertical", outline:"none", lineHeight:1.5,
              }}
            />

            {status && (
              <div style={{ fontSize:9, color: status.startsWith("✅") ? "#00ff99" : "#ff5555",
                marginTop:6, marginBottom:4 }}>{status}</div>
            )}

            <div style={{ display:"flex", gap:8, marginTop:10 }}>
              {mode === "export" && (
                <button className="btn" onClick={handleCopy} style={{
                  flex:1, background:"#bb66ff18", border:"1px solid #bb66ff44",
                  padding:"9px", color:"#bb66ff", fontSize:9,
                  letterSpacing:2, borderRadius:3 }}>
                  📋 COPY TO CLIPBOARD
                </button>
              )}
              {mode === "import" && (
                <button className="btn" onClick={handleImport} style={{
                  flex:1, background:"#33ddff18", border:"1px solid #33ddff44",
                  padding:"9px", color:"#33ddff", fontSize:9,
                  letterSpacing:2, borderRadius:3 }}>
                  ✓ LOAD CONFIG
                </button>
              )}
              <button className="btn" onClick={() => setOpen(false)} style={{
                background:"rgba(2,10,20,0.8)", border:"1px solid #0b1c2c",
                padding:"9px 16px", color:"#4466aa", fontSize:9, borderRadius:3 }}>
                CANCEL
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
}

// ═══════════════════════════════════════════════════════════════
// MAIN APP
// ═══════════════════════════════════════════════════════════════
export default function App() {
  const [dk,       setDk]      = useState("mini4pro");
  const [payload,  setPayload] = useState(0);
  const [alt,      setAlt]     = useState(10);
  const [climb,    setClimb]   = useState(0);
  const [spd,      setSpd]     = useState(0);
  const [windSpd,  setWindSpd] = useState(0);
  const [windDir,  setWindDir] = useState(0);
  const [tempC,    setTempC]   = useState(25);
  const [tempDT,   setTempDT]  = useState(0);
  const [tab,      setTab]     = useState("flightTime");
  const [allCmp,   setAllCmp]  = useState(false);
  const [custom,   setCustom]  = useState(CUSTOM_DEF);
  const [showCfg,  setShowCfg] = useState(false);
  const [noiseDist,setNoiseDist]=useState(5);

  const drone = dk==="custom" ? custom : DRONES[dk];
  const mass  = drone.mass_kg;

  const env = useMemo(()=>({
    wind_speed:windSpd, wind_dir:windDir,
    temp_dT:tempDT, temp_C:tempC,
    payload_kg:payload,
  }), [windSpd,windDir,tempDT,tempC,payload]);

  const pw    = useMemo(()=>calcPower(drone,spd,mass,alt,climb,env), [drone,spd,mass,alt,climb,env]);
  const ftRes = useMemo(()=>flightMin(drone,pw.P_total,env), [drone,pw.P_total,env]);
  const ft    = ftRes.t;

  const pw0    = useMemo(()=>calcPower(drone,0,mass,alt,0,env),          [drone,mass,alt,env]);
  const ft0Res = useMemo(()=>flightMin(drone,pw0.P_total,env),            [drone,pw0.P_total,env]);
  const ft0    = ft0Res.t;

  const pwCr   = useMemo(()=>calcPower(drone,drone.real_cruise_v||10,mass,alt,0,env),[drone,mass,alt,env]);
  const ftCr   = useMemo(()=>flightMin(drone,pwCr.P_total,env).t,        [drone,pwCr.P_total,env]);

  const DL     = useMemo(()=>diskLoad(drone,mass+payload),               [drone,mass,payload]);
  const noise  = useMemo(()=>droneNoise(drone,mass+payload,pw.rho),      [drone,mass,payload,pw.rho]);
  const batRes = useMemo(()=>batteryVoltage(drone,pw.P_total,tempC),     [drone,pw.P_total,tempC]);
  const escRes = useMemo(()=>escThermal(drone,pw.P_total,tempC),         [drone,pw.P_total,tempC]);
  const TW     = useMemo(()=>(mass+payload)*g / Math.max(pw.P_total/Math.max(spd,1),1),[mass,payload,pw.P_total,spd]);

  // Accuracy
  const hErr = drone.real_hover_min
    ? (ft0-drone.real_hover_min)/drone.real_hover_min*100 : null;
  const cErr = drone.real_cruise_min
    ? (ftCr-drone.real_cruise_min)/drone.real_cruise_min*100 : null;
  const hCol = hErr===null?"#33ddff"
    : Math.abs(hErr)<12?"#00ff99":Math.abs(hErr)<25?"#ffbb33":"#ff5555";

  // Optimal speed
  const {optV,optFT} = useMemo(()=>{
    let oV=0,oF=0;
    for(let v=0;v<=drone.max_v;v+=0.5){
      const f=flightMin(drone,calcPower(drone,v,mass,alt,0,env).P_total,env).t;
      if(f>oF){oF=f;oV=v;}
    }
    return{optV:oV,optFT:oF};
  },[drone,mass,alt,env]);

  // Chart data
  const chartData = useMemo(()=>{
    const N=55;
    return Array.from({length:N+1},(_,i)=>{
      const v=(i*drone.max_v)/N;
      const pw2=calcPower(drone,v,mass,alt,climb,env);
      const ft2=flightMin(drone,pw2.P_total,env).t;
      const row={
        v:+v.toFixed(2),
        P_total:+(pw2.P_total/1000).toFixed(3),
        P_ind:+(pw2.P_ind/1000).toFixed(3),
        P_pro:+(pw2.P_pro/1000).toFixed(3),
        P_par:+(pw2.P_par/1000).toFixed(3),
        flightTime:+ft2.toFixed(2),
        FM_v:+pw2.FM.toFixed(3),
        CdA_v:+pw2.CdA.toFixed(4),
      };
      if(allCmp){
        Object.entries(DRONES).forEach(([key,d])=>{
          if(key===dk)return;
          const vc=Math.min(v,d.max_v);
          const p2=calcPower(d,vc,d.mass_kg,alt,0,{temp_C:tempC,temp_dT:tempDT});
          row[`ft_${key}`]=+flightMin(d,p2.P_total,{temp_C:tempC}).t.toFixed(2);
          row[`pow_${key}`]=+(p2.P_total/1000).toFixed(3);
        });
      }
      return row;
    });
  },[drone,mass,alt,climb,env,allCmp,dk,tempC,tempDT]);

  // Power pie data
  const pieData = useMemo(()=>[
    {name:"Induced",  value:Math.max(0,pw.P_ind),  color:"#00ff99"},
    {name:"Profile",  value:Math.max(0,pw.P_pro),  color:"#33ddff"},
    {name:"Parasite", value:Math.max(0,pw.P_par),  color:"#ffbb33"},
    {name:"Climb",    value:Math.max(0,pw.P_clm),  color:"#ff8833"},
    {name:"Systems",  value:drone.P_sys,            color:"#bb66ff"},
  ].filter(d=>d.value>0.1),[pw,drone]);

  // Radar
  const radarData = useMemo(()=>{
    const metrics=["Efficiency","Endurance","Speed","Payload","Agility"];
    return metrics.map(m=>{
      const row={m};
      Object.entries(DRONES).forEach(([,d])=>{
        const pw2=calcPower(d,0,d.mass_kg,10,0,{});
        const ft2=flightMin(d,pw2.P_total,{}).t;
        const DL2=diskLoad(d,d.mass_kg);
        const s={
          Efficiency:Math.round((d.FM/0.85)*100),
          Endurance:Math.min(100,Math.round((ft2/60)*100)),
          Speed:Math.round((d.max_v/35)*100),
          Payload:Math.min(100,Math.round((d.mass_kg/7)*100)),
          Agility:Math.min(100,Math.max(5,Math.round((1-DL2/700)*100))),
        };
        row[d.gen]=Math.max(5,s[m]||0);
      });
      return row;
    });
  },[]);

  const pbrk=[
    {n:"Induced", v:+(pw.P_ind/1000).toFixed(2),c:"#00ff99"},
    {n:"Profile", v:+(pw.P_pro/1000).toFixed(2),c:"#33ddff"},
    {n:"Parasite",v:+(pw.P_par/1000).toFixed(2),c:"#ffbb33"},
    {n:"Climb",   v:+(Math.abs(pw.P_clm)/1000).toFixed(2),c:"#ff8833"},
    {n:"Systems", v:+(drone.P_sys/1000).toFixed(2),c:"#bb66ff"},
  ];

  const tabs=[
    {id:"flightTime",lb:"Flight Time",c:"#00ff99"},
    {id:"P_total",   lb:"Power kW",   c:"#ffbb33"},
    {id:"FM_v",      lb:"FM(V)",      c:"#33ddff"},
    {id:"CdA_v",     lb:"CdA(V) m²", c:"#ff8833"},
  ];
  const tc=tabs.find(t=>t.id===tab)?.c||"#00ff99";

  const cfgFields=[
    {k:"num_rotors",  lb:"Rotors",    min:3,   max:8,    step:1,     u:"",    c:"#00ff99"},
    {k:"prop_d",      lb:"Prop ⌀",   min:0.05,max:0.7,  step:0.005, u:"m",   c:"#00ff99"},
    {k:"mass_kg",     lb:"Mass",      min:0.1, max:20,   step:0.05,  u:"kg",  c:"#ffbb33"},
    {k:"battery_Wh",  lb:"Battery",   min:10,  max:5000, step:5,     u:"Wh",  c:"#ffbb33"},
    {k:"cell_count",  lb:"Cells (S)", min:2,   max:14,   step:1,     u:"S",   c:"#ffbb33"},
    {k:"FM",          lb:"FM hover",  min:0.35,max:0.88, step:0.01,  u:"",    c:"#33ddff"},
    {k:"CdA_h",       lb:"CdA hover", min:0.002,max:0.5, step:0.005, u:"m²",  c:"#33ddff"},
    {k:"CdA_f",       lb:"CdA fwd",   min:0.001,max:0.2, step:0.002, u:"m²",  c:"#33ddff"},
    {k:"P_sys",       lb:"P systems", min:1,   max:600,  step:1,     u:"W",   c:"#ff8833"},
    {k:"motor_kv",    lb:"Motor Kv",  min:100, max:5000, step:50,    u:"RPM/V",c:"#ff5555"},
    {k:"tip_speed",   lb:"Tip Speed", min:80,  max:300,  step:5,     u:"m/s", c:"#ff5555"},
  ];
  const setCf=useCallback((k,v)=>setCustom(p=>({...p,[k]:v})),[]);

  // Warnings
  const warnEsc  = escRes.T_esc > 75;
  const warnBVI  = pw.bvi > 1.03;
  const warnSag  = batRes.sag_pct > 8;
  const warnCold = tempC < 10;

  return (
    <div style={{minHeight:"100vh",background:"#00060e",color:"#bbd2e2",
      fontFamily:"'Share Tech Mono',monospace",overflow:"hidden",position:"relative"}}>
      <style>{`
        @import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700;900&family=Share+Tech+Mono&display=swap');
        *{box-sizing:border-box}
        ::-webkit-scrollbar{width:3px;background:#000}::-webkit-scrollbar-thumb{background:#ffffff12}
        input[type=range]{-webkit-appearance:none;height:3px;border-radius:2px;outline:none;cursor:pointer;width:100%}
        input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:12px;height:12px;border-radius:50%;border:2px solid #00060e;cursor:pointer}
        .btn{cursor:pointer;border:none;transition:all .13s;font-family:'Share Tech Mono',monospace}
        .btn:hover{filter:brightness(1.3);transform:translateY(-1px)}
        .card{cursor:pointer;transition:all .18s}.card:hover{filter:brightness(1.1)}
      `}</style>
      <div style={{position:"fixed",inset:0,pointerEvents:"none",
        backgroundImage:"linear-gradient(rgba(0,180,120,0.015) 1px,transparent 1px),linear-gradient(90deg,rgba(0,180,120,0.015) 1px,transparent 1px)",
        backgroundSize:"48px 48px"}}/>

      {/* HEADER */}
      <div style={{borderBottom:"1px solid #ffffff0c",padding:"11px 20px",
        background:"rgba(0,180,120,0.014)",display:"flex",alignItems:"center",
        justifyContent:"space-between",position:"relative",zIndex:10}}>
        <div style={{display:"flex",alignItems:"center",gap:12}}>
          <div style={{width:36,height:36,border:"2px solid #00ff99",display:"flex",
            alignItems:"center",justifyContent:"center",fontSize:15,color:"#00ff99"}}>⬡</div>
          <div>
            <div style={{fontSize:14,fontFamily:"'Orbitron',sans-serif",fontWeight:900,
              color:"#00ff99",letterSpacing:4,textShadow:"0 0 16px #00ff9930"}}>
              MULTIROTOR UAV — ADVANCED PHYSICS v8
            </div>
            <div style={{fontSize:7,color:"#1a3a2a",letterSpacing:2}}>
              MOTOR·BATTERY SAG·WIND·BVI·DESCENT·ESC THERMAL·NOISE·GROUND EFFECT·PEUKERT
            </div>
          </div>
        </div>
        <div style={{display:"flex",gap:16,alignItems:"center"}}>
          {/* Warning badges */}
          {warnEsc&&<div style={{fontSize:8,color:"#ff5555",border:"1px solid #ff555533",padding:"3px 8px",borderRadius:2}}>⚠ ESC HOT {escRes.T_esc.toFixed(0)}°C</div>}
          {warnBVI&&<div style={{fontSize:8,color:"#ff8833",border:"1px solid #ff883333",padding:"3px 8px",borderRadius:2}}>⚠ BVI ZONE</div>}
          {warnSag&&<div style={{fontSize:8,color:"#ffbb33",border:"1px solid #ffbb3333",padding:"3px 8px",borderRadius:2}}>⚠ V-SAG {batRes.sag_pct.toFixed(1)}%</div>}
          {warnCold&&<div style={{fontSize:8,color:"#33ddff",border:"1px solid #33ddff33",padding:"3px 8px",borderRadius:2}}>❄ COLD BAT</div>}
          <div style={{textAlign:"right",fontSize:8,lineHeight:1.8}}>
            <div style={{color:"#00ff99",fontFamily:"'Orbitron',sans-serif"}}>DL={DL.toFixed(1)}N/m²</div>
            <div style={{color:"#ffbb33"}}>μ={pw.mu.toFixed(3)} · κ={pw.kap.toFixed(3)}</div>
            <div style={{color:"#33ddff"}}>ρ={pw.rho.toFixed(4)}kg/m³</div>
          </div>
        </div>
      </div>

      <div style={{padding:"12px 20px",maxWidth:1500,margin:"0 auto",position:"relative",zIndex:10}}>

        {/* DRONE SELECTOR */}
        <div style={{marginBottom:10}}>
          <div style={{fontSize:8,color:"#1a3a2a",letterSpacing:3,marginBottom:5}}>▶ PLATFORM</div>
          <div style={{display:"grid",gridTemplateColumns:"repeat(5,1fr)",gap:5}}>
            {[...Object.entries(DRONES),["custom",custom]].map(([key,d])=>(
              <button key={key} className="btn"
                onClick={()=>{setDk(key);if(key==="custom")setShowCfg(true);}} style={{
                background:dk===key?`${d.color}14`:"rgba(2,10,20,0.7)",
                border:`1px solid ${dk===key?d.color+"44":"#0b1c2c"}`,
                borderTop:`3px solid ${dk===key?d.color:"#0b1c2c"}`,
                padding:"8px 7px",textAlign:"left",borderRadius:3}}>
                <div style={{fontSize:7,color:dk===key?d.color:"#1a3a2a",letterSpacing:2,marginBottom:2}}>
                  {d.gen?.toUpperCase()} {dk===key?"◉":"○"}
                </div>
                <div style={{fontSize:9,fontFamily:"'Orbitron',sans-serif",fontWeight:700,
                  color:dk===key?d.color:"#4466aa",marginBottom:1}}>{d.name}</div>
                <div style={{fontSize:6,color:"#1a3a2a"}}>{d.label}</div>
                {d.real_hover_min&&<div style={{fontSize:6,color:"#223344",marginTop:1}}>
                  ✦ {d.real_hover_min}min hover
                </div>}
              </button>
            ))}
          </div>
        </div>

        {/* 3-COLUMN MAIN LAYOUT */}
        <div style={{display:"grid",gridTemplateColumns:"220px 1fr 220px",gap:12,marginBottom:12}}>

          {/* ── LEFT: FLIGHT + ENV PARAMS ── */}
          <div style={{display:"flex",flexDirection:"column",gap:7}}>
            <div style={{fontSize:8,color:"#1a3a2a",letterSpacing:3}}>▶ FLIGHT</div>
            <Sldr label="AIRSPEED"    val={spd}     min={0}   max={drone.max_v}   step={0.5} unit="m/s"
              set={setSpd}     color="#00ff99" sub={`Opt ${optV.toFixed(1)}m/s→${optFT.toFixed(1)}min`}/>
            <Sldr label="PAYLOAD"     val={payload} min={0}   max={5}             step={0.05} unit="kg"
              set={setPayload} color="#ffbb33" sub={`Total: ${(mass+payload).toFixed(2)}kg`}/>
            <Sldr label="ALTITUDE"    val={alt}     min={0}   max={drone.max_alt} step={50}  unit="m"
              set={setAlt}     color="#33ddff" sub={`GE ${alt<2.5*drone.prop_d?"ACTIVE":"off"} · ρ=${pw.rho.toFixed(3)}`}/>
            <Sldr label="CLIMB RATE"  val={climb}   min={-5}  max={10}            step={0.5} unit="m/s"
              set={setClimb}   color="#ff8833" sub={climb<0?"⚠ VRS RISK >-2m/s":"level=0"}/>

            <div style={{fontSize:8,color:"#1a3a2a",letterSpacing:3,marginTop:4}}>▶ ENVIRONMENT</div>
            <Sldr label="WIND SPEED"  val={windSpd} min={0}   max={20}            step={0.5} unit="m/s"
              set={setWindSpd} color="#33ddff" sub="0=calm"/>
            <Sldr label="WIND DIR"    val={windDir} min={0}   max={180}           step={5}   unit="°"
              set={setWindDir} color="#33ddff"
              sub={windDir<30?"headwind":windDir>150?"tailwind":"crosswind"}/>
            <Sldr label="TEMPERATURE" val={tempC}   min={-20} max={50}            step={1}   unit="°C"
              set={setTempC}   color="#ff8833"
              sub={`ISA Δ${tempDT>=0?"+":""}${tempDT}°C · bat${tempC<10?"❄":"✓"}`}/>
            <Sldr label="ISA Δ TEMP"  val={tempDT}  min={-20} max={40}            step={1}   unit="°C"
              set={setTempDT}  color="#ff8833" sub="non-standard day correction"/>
            <Sldr label="NOISE DIST"  val={noiseDist} min={1} max={50}            step={1}   unit="m"
              set={setNoiseDist} color="#bb66ff" sub="SPL measurement distance"/>

            <button className="btn" onClick={()=>setAllCmp(!allCmp)} style={{
              background:allCmp?"rgba(51,221,255,0.09)":"rgba(2,10,20,0.6)",
              border:`1px solid ${allCmp?"#33ddff33":"#0b1c2c"}`,
              padding:"7px 10px",color:allCmp?"#33ddff":"#1a3a2a",
              fontSize:8,letterSpacing:2,borderRadius:3}}>
              {allCmp?"◉":"○"} COMPARISON
            </button>

            {/* Config panel */}
            <ConfigPanel
              drone={drone} dk={dk}
              spd={spd} alt={alt} climb={climb} payload={payload}
              windSpd={windSpd} windDir={windDir} tempC={tempC} tempDT={tempDT}
              onImport={(cfg)=>{
                if(cfg.spd!==undefined) setSpd(cfg.spd);
                if(cfg.alt!==undefined) setAlt(cfg.alt);
                if(cfg.climb!==undefined) setClimb(cfg.climb);
                if(cfg.payload!==undefined) setPayload(cfg.payload);
                if(cfg.windSpd!==undefined) setWindSpd(cfg.windSpd);
                if(cfg.windDir!==undefined) setWindDir(cfg.windDir);
                if(cfg.tempC!==undefined) setTempC(cfg.tempC);
                if(cfg.tempDT!==undefined) setTempDT(cfg.tempDT);
                if(cfg.drone){setDk("custom");setCustom(cfg.drone);}
              }}
            />
          </div>

          {/* ── CENTER: CHARTS ── */}
          <div style={{display:"flex",flexDirection:"column",gap:10}}>

            {/* Tabs */}
            <div style={{display:"flex",gap:5,flexWrap:"wrap",alignItems:"center"}}>
              <span style={{fontSize:7,color:"#1a3a2a",letterSpacing:2,marginRight:3}}>CHART:</span>
              {tabs.map(t=>(
                <button key={t.id} className="btn" onClick={()=>setTab(t.id)} style={{
                  background:tab===t.id?`${t.c}16`:"rgba(2,10,20,0.6)",
                  border:`1px solid ${tab===t.id?t.c+"44":"#0b1c2c"}`,
                  padding:"4px 10px",fontSize:8,letterSpacing:1,borderRadius:3,
                  color:tab===t.id?t.c:"#1a3a2a",cursor:"pointer",
                  fontFamily:"'Share Tech Mono',monospace",
                  fontWeight:tab===t.id?"bold":"normal"}}>{t.lb}</button>
              ))}
            </div>

            {/* Main chart */}
            <div style={{background:"rgba(1,6,16,0.92)",border:"1px solid #0b1c2c",
              borderTop:`3px solid ${tc}`,padding:"12px",borderRadius:3}}>
              <div style={{fontSize:8,color:"#4466aa",letterSpacing:3,marginBottom:7}}>
                ▶ {tabs.find(t=>t.id===tab)?.lb.toUpperCase()} vs AIRSPEED
                {tab==="flightTime"&&<span style={{color:"#00ff99",marginLeft:8}}>
                  ★ OPT {optV.toFixed(1)}m/s → {optFT.toFixed(1)}min
                </span>}
              </div>
              <ResponsiveContainer width="100%" height={190}>
                <LineChart data={chartData}>
                  <CartesianGrid strokeDasharray="2 6" stroke="#0b1c2c"/>
                  <XAxis dataKey="v" stroke="#162636" tick={{fontSize:7,fill:"#334455"}}
                    label={{value:"m/s",position:"insideBottom",offset:-3,fill:"#334455",fontSize:7}}/>
                  <YAxis stroke="#162636" tick={{fontSize:7,fill:"#334455"}} width={42}/>
                  <Tooltip content={<TTip/>}/>
                  <ReferenceLine x={spd} stroke="#ffffff0e" strokeDasharray="3 3"/>
                  {tab==="flightTime"&&<ReferenceLine x={optV} stroke={`${tc}44`}
                    strokeDasharray="4 2" label={{value:"OPT",fill:tc,fontSize:7}}/>}
                  <Line type="monotone" dataKey={tab}
                    stroke={drone.color} strokeWidth={3} dot={false} name={drone.name}/>
                  {allCmp&&Object.entries(DRONES).map(([key,d])=>
                    key!==dk&&(
                      <Line key={key} type="monotone"
                        dataKey={tab==="flightTime"?`ft_${key}`:`pow_${key}`}
                        stroke={d.color} strokeWidth={1.2} dot={false} name={d.name}
                        strokeOpacity={0.45} strokeDasharray="5 3"/>
                    )
                  )}
                </LineChart>
              </ResponsiveContainer>
            </div>

            {/* Power area chart */}
            <div style={{background:"rgba(1,6,16,0.92)",border:"1px solid #0b1c2c",
              borderTop:"3px solid #ffbb3344",padding:"11px",borderRadius:3}}>
              <div style={{fontSize:8,color:"#4466aa",letterSpacing:3,marginBottom:6}}>
                ▶ POWER COMPONENTS vs AIRSPEED
              </div>
              <ResponsiveContainer width="100%" height={145}>
                <AreaChart data={chartData}>
                  <defs>
                    {[["a","#00ff99"],["b","#33ddff"],["c","#ffbb33"],["d","#ff8833"]].map(([id,col])=>(
                      <linearGradient key={id} id={`g${id}`} x1="0" y1="0" x2="0" y2="1">
                        <stop offset="5%"  stopColor={col} stopOpacity={0.22}/>
                        <stop offset="95%" stopColor={col} stopOpacity={0.02}/>
                      </linearGradient>
                    ))}
                  </defs>
                  <CartesianGrid strokeDasharray="2 6" stroke="#0b1c2c"/>
                  <XAxis dataKey="v" stroke="#162636" tick={{fontSize:6,fill:"#334455"}}/>
                  <YAxis stroke="#162636" tick={{fontSize:6,fill:"#334455"}} width={38}/>
                  <Tooltip content={<TTip/>}/>
                  <ReferenceLine x={spd} stroke="#ffffff0c" strokeDasharray="3 3"/>
                  <Area type="monotone" dataKey="P_ind" stroke="#00ff99" fill="url(#ga)" strokeWidth={1.5} name="Induced"  dot={false}/>
                  <Area type="monotone" dataKey="P_pro"  stroke="#33ddff" fill="url(#gb)" strokeWidth={1.2} name="Profile"  dot={false}/>
                  <Area type="monotone" dataKey="P_par"  stroke="#ffbb33" fill="url(#gc)" strokeWidth={1.2} name="Parasite" dot={false}/>
                  <Area type="monotone" dataKey="P_clm"  stroke="#ff8833" fill="url(#gd)" strokeWidth={1}   name="Climb"    dot={false}/>
                  <Legend wrapperStyle={{fontSize:7,color:"#334455"}}/>
                </AreaChart>
              </ResponsiveContainer>
            </div>

            {/* Radar */}
            <div style={{background:"rgba(1,6,16,0.92)",border:"1px solid #0b1c2c",
              borderTop:"3px solid #bb66ff33",padding:"11px",borderRadius:3}}>
              <div style={{fontSize:8,color:"#4466aa",letterSpacing:3,marginBottom:6}}>
                ▶ DRONE CLASS COMPARISON
              </div>
              <ResponsiveContainer width="100%" height={160}>
                <RadarChart data={radarData}>
                  <PolarGrid stroke="#0b1c2c"/>
                  <PolarAngleAxis dataKey="m" tick={{fontSize:8,fill:"#334455"}}/>
                  <PolarRadiusAxis tick={{fontSize:5,fill:"#162636"}} domain={[0,100]}/>
                  {Object.values(DRONES).map(d=>(
                    <Radar key={d.gen} name={d.name} dataKey={d.gen}
                      stroke={d.color} fill={d.color} fillOpacity={0.08} strokeWidth={2}/>
                  ))}
                  <Legend wrapperStyle={{fontSize:7,color:"#334455"}}/>
                </RadarChart>
              </ResponsiveContainer>
            </div>
          </div>

          {/* ── RIGHT: LIVE DASHBOARD ── */}
          <div style={{display:"flex",flexDirection:"column",gap:7}}>
            <div style={{fontSize:8,color:"#1a3a2a",letterSpacing:3}}>▶ LIVE RESULTS</div>

            {/* Accuracy badge */}
            {hErr!==null&&(
              <div style={{background:`${hCol}0c`,border:`1px solid ${hCol}2a`,
                padding:"7px 10px",borderRadius:3}}>
                <div style={{fontSize:7,color:"#4466aa",letterSpacing:2,marginBottom:3}}>ACCURACY</div>
                <div style={{fontSize:11,color:hCol,fontWeight:"bold"}}>
                  {Math.abs(hErr)<12?"✅":Math.abs(hErr)<25?"⚠️":"❌"}{" "}
                  {hErr>0?"+":""}{hErr.toFixed(1)}% hover
                </div>
                {drone.real_cruise_min&&cErr!==null&&(
                  <div style={{fontSize:9,color:"#33ddff",fontWeight:"bold"}}>
                    {Math.abs(cErr)<12?"✅":"⚠️"} {cErr>0?"+":""}{cErr.toFixed(1)}% cruise
                  </div>
                )}
                <div style={{fontSize:7,color:"#334455",marginTop:2}}>
                  {ft0.toFixed(1)}min hover · {ftCr.toFixed(1)}min@{drone.real_cruise_v||10}m/s
                </div>
              </div>
            )}

            {/* Gauges row */}
            <div style={{background:"rgba(2,10,20,0.85)",border:"1px solid #0b1c2c",
              padding:"8px",borderRadius:3}}>
              <div style={{fontSize:7,color:"#4466aa",letterSpacing:2,marginBottom:6}}>PERFORMANCE GAUGES</div>
              <div style={{display:"flex",justifyContent:"space-around"}}>
                <Gauge value={ft0} max={60} label="Hover" unit="min" color={hCol}/>
                <Gauge value={pw.P_total/1000} max={Math.max(drone.battery_Wh/5,1)} label="Power" unit="kW" color={drone.color}/>
                <Gauge value={noiseEstimate(drone,mass+payload,pw.rho,noiseDist)}
                  max={110} label="Noise" unit="dB" color="#bb66ff"/>
              </div>
            </div>

            <SC label="Total Power"  val={(pw.P_total/1000).toFixed(2)} unit="kW"
              color={drone.color}
              sub={`Hover: ${(pw0.P_total/1000).toFixed(2)}kW · ESC η=${(pw.esc_derate*100).toFixed(0)}%`}
              warn={pw.esc_derate<0.95?"#ff5555":null}/>
            <SC label="Flight Time"  val={ft.toFixed(1)} unit="min" color={hCol}
              sub={`Opt: ${optFT.toFixed(1)}min @ ${optV.toFixed(1)}m/s`}/>
            <SC label="Battery V"    val={batRes.V_actual.toFixed(2)} unit="V"
              color={warnSag?"#ffbb33":"#33ddff"}
              sub={`Sag: ${batRes.sag_pct.toFixed(1)}% · I=${batRes.I.toFixed(1)}A · Cr=${ftRes.Cr?.toFixed(1)||"–"}C`}
              warn={warnSag?"#ffbb33":null}/>
            <SC label="FM(V) / κ(V)" val={pw.FM.toFixed(3)} unit=""  color="#33ddff"
              sub={`κ=${pw.kap.toFixed(3)} · δ=${pw.dlt.toFixed(3)} · BVI×${pw.bvi.toFixed(3)}`}
              warn={warnBVI?"#ff8833":null}/>
            <SC label="vi / vh"      val={pw.vi.toFixed(2)} unit="m/s" color="#ffbb33"
              sub={`vh=${pw.vh.toFixed(2)} · A=${pw.A.toFixed(3)}m² · GE=${pw.ge.toFixed(3)}`}/>
            <SC label="ESC Temp"     val={escRes.T_esc.toFixed(0)} unit="°C"
              color={warnEsc?"#ff5555":"#334455"}
              sub={`Derate: ${(pw.esc_derate*100).toFixed(0)}% · I/ESC=${escRes.I.toFixed(1)}A`}
              warn={warnEsc?"#ff5555":null}/>
            <SC label="Noise SPL"    val={noiseEstimate(drone,mass+payload,pw.rho,noiseDist).toFixed(1)}
              unit="dB(A)" color="#bb66ff"
              sub={`@ ${noiseDist}m · μ=${pw.mu.toFixed(3)} · tip=${drone.tip_speed||160}m/s`}/>

            {/* Power pie */}
            <div style={{background:"rgba(2,10,20,0.85)",border:"1px solid #0b1c2c",
              padding:"8px",borderRadius:3}}>
              <div style={{fontSize:7,color:"#4466aa",letterSpacing:2,marginBottom:4}}>POWER PIE</div>
              <ResponsiveContainer width="100%" height={100}>
                <PieChart>
                  <Pie data={pieData} cx="50%" cy="50%" innerRadius={28} outerRadius={44}
                    dataKey="value" paddingAngle={2}>
                    {pieData.map((e,i)=><Cell key={i} fill={e.color} opacity={0.85}/>)}
                  </Pie>
                  <Tooltip formatter={(v)=>`${(v/1000).toFixed(2)} kW`}
                    contentStyle={{background:"rgba(1,5,14,0.95)",border:"1px solid #ffffff12",
                      fontSize:9,fontFamily:"'Share Tech Mono',monospace"}}/>
                  <Legend wrapperStyle={{fontSize:6,color:"#334455"}}/>
                </PieChart>
              </ResponsiveContainer>
            </div>

            {/* Power breakdown bars */}
            <div style={{background:"rgba(2,10,20,0.85)",border:"1px solid #0b1c2c",
              padding:"8px 10px",borderRadius:3}}>
              <div style={{fontSize:7,color:"#4466aa",letterSpacing:2,marginBottom:6}}>POWER BREAKDOWN</div>
              {pbrk.map(b=>(
                <div key={b.n} style={{marginBottom:4}}>
                  <div style={{display:"flex",justifyContent:"space-between",fontSize:8,marginBottom:1}}>
                    <span style={{color:"#4466aa"}}>{b.n}</span>
                    <span style={{color:b.c,fontFamily:"'Orbitron',sans-serif",fontWeight:700,fontSize:9}}>{b.v}kW</span>
                  </div>
                  <div style={{height:3,background:"#0b1c2c",borderRadius:2}}>
                    <div style={{height:"100%",borderRadius:2,background:b.c,transition:"width .3s",
                      width:`${Math.min(100,(b.v/Math.max(pw.P_total/1000,0.01))*100).toFixed(0)}%`}}/>
                  </div>
                </div>
              ))}
            </div>

            {/* Rotor diagram + info */}
            <div style={{background:"rgba(2,10,20,0.85)",border:"1px solid #0b1c2c",
              padding:"8px",borderRadius:3,display:"flex",alignItems:"center",gap:8}}>
              <RotorDiag drone={drone} pw={pw}/>
              <div style={{fontSize:7,color:"#223344",lineHeight:2.0}}>
                <div style={{color:"#00ff99"}}>{drone.num_rotors}×⌀{(drone.prop_d*100).toFixed(0)}cm</div>
                <div>DL:{DL.toFixed(0)}N/m²</div>
                <div>FM₀→{pw.FM.toFixed(2)}</div>
                <div style={{color:pw.ge<0.99?"#ffbb33":"#334455"}}>
                  {pw.ge<0.99?`IGE${(pw.ge*100).toFixed(0)}%`:"OGE"}
                </div>
                <div style={{color:warnBVI?"#ff8833":"#334455"}}>
                  BVI×{pw.bvi.toFixed(2)}
                </div>
                <div style={{color:drone.color}}>{drone.cell_count}S·{drone.battery_Wh}Wh</div>
              </div>
            </div>
          </div>
        </div>

        {/* FLEET CARDS */}
        <div style={{background:"rgba(1,6,16,0.92)",border:"1px solid #0b1c2c",
          borderTop:"3px solid #33ddff18",padding:"12px 14px",borderRadius:3,marginBottom:10}}>
          <div style={{fontSize:8,color:"#4466aa",letterSpacing:3,marginBottom:9}}>
            ▶ FLEET — REAL DATA VALIDATION
          </div>
          <div style={{display:"grid",gridTemplateColumns:"repeat(4,1fr)",gap:7}}>
            {Object.entries(DRONES).map(([key,d])=>{
              const e0={}; // neutral env for fleet comparison
              const pw2=calcPower(d,0,d.mass_kg,10,0,e0);
              const ft2=flightMin(d,pw2.P_total,e0).t;
              const cv=d.real_cruise_v||10;
              const pw3=calcPower(d,cv,d.mass_kg,50,0,e0);
              const ft3=flightMin(d,pw3.P_total,e0).t;
              const DL2=diskLoad(d,d.mass_kg);
              const e2=d.real_hover_min?(ft2-d.real_hover_min)/d.real_hover_min*100:null;
              const ec=e2===null?"#33ddff":Math.abs(e2)<12?"#00ff99":Math.abs(e2)<25?"#ffbb33":"#ff5555";
              const n2=noiseEstimate(d,d.mass_kg,rhoAlt(0),5);
              return (
                <div key={key} className="card" onClick={()=>setDk(key)} style={{
                  background:dk===key?`${d.color}0b`:"rgba(2,8,20,0.7)",
                  border:`1px solid ${dk===key?d.color+"44":"#0b1c2c"}`,
                  padding:"10px",borderRadius:3}}>
                  <div style={{fontSize:7,color:d.color,letterSpacing:2,marginBottom:2}}>
                    {d.gen?.toUpperCase()} {dk===key?"◉":"○"}
                  </div>
                  <div style={{fontSize:10,fontFamily:"'Orbitron',sans-serif",fontWeight:700,
                    color:dk===key?d.color:"#4466aa",marginBottom:1}}>{d.name}</div>
                  <div style={{fontSize:6,color:"#1a3a2a",marginBottom:6}}>{d.label}</div>
                  <div style={{display:"grid",gridTemplateColumns:"1fr 1fr",gap:3}}>
                    {[
                      ["Mass",  d.mass_kg+"kg",     ""],
                      ["Bat.",  d.battery_Wh+"Wh",  ""],
                      ["Hover", ft2.toFixed(1),      "min"],
                      ["Cruis", ft3.toFixed(1),      "min"],
                      ["FM₀",  d.FM.toFixed(2),      ""],
                      ["dB",   n2.toFixed(0),         "dB@5m"],
                    ].map(([l,v,u])=>(
                      <div key={l} style={{background:"#00020b",padding:"3px 5px",borderRadius:2}}>
                        <div style={{fontSize:5,color:"#162636",letterSpacing:1}}>{l}</div>
                        <div style={{fontSize:9,color:d.color,
                          fontFamily:"'Orbitron',sans-serif",fontWeight:700}}>
                          {v}<span style={{fontSize:5,marginLeft:1,color:"#1a3a2a"}}>{u}</span>
                        </div>
                      </div>
                    ))}
                  </div>
                  {d.real_hover_min&&(
                    <div style={{marginTop:6,padding:"3px 6px",
                      background:`${ec}08`,border:`1px solid ${ec}1c`,borderRadius:2}}>
                      <div style={{fontSize:6,color:"#1a3a2a",marginBottom:1}}>REAL DATA</div>
                      <div style={{fontSize:8,color:ec,fontWeight:"bold"}}>
                        {Math.abs(e2)<12?"✅":"⚠️"} {d.real_hover_min}min · {e2>0?"+":""}{e2?.toFixed(0)}%
                      </div>
                    </div>
                  )}
                </div>
              );
            })}
          </div>
        </div>

        {/* CUSTOM CONFIG */}
        {showCfg&&(
          <div style={{background:"rgba(1,6,16,0.95)",border:"1px solid #bb66ff33",
            borderTop:"3px solid #bb66ff",padding:"12px 14px",borderRadius:3,marginBottom:10}}>
            <div style={{display:"flex",justifyContent:"space-between",alignItems:"center",marginBottom:9}}>
              <div style={{fontSize:8,color:"#bb66ff",letterSpacing:3}}>▶ CUSTOM DRONE CONFIG</div>
              <button className="btn" onClick={()=>setShowCfg(false)} style={{
                background:"#bb66ff14",border:"1px solid #bb66ff33",
                padding:"4px 9px",color:"#bb66ff",fontSize:8,borderRadius:2}}>✕</button>
            </div>
            <div style={{display:"grid",gridTemplateColumns:"repeat(4,1fr)",gap:6}}>
              {cfgFields.map(f=>(
                <Sldr key={f.k} label={f.lb} val={custom[f.k]}
                  min={f.min} max={f.max} step={f.step} unit={f.u} color={f.c}
                  set={v=>setCf(f.k,v)}/>
              ))}
            </div>
          </div>
        )}

        {/* PHYSICS MODELS */}
        <div style={{background:"rgba(1,6,16,0.92)",border:"1px solid #0b1c2c",
          borderTop:"3px solid #00ff9912",padding:"11px 14px",borderRadius:3}}>
          <div style={{fontSize:8,color:"#4466aa",letterSpacing:3,marginBottom:8}}>
            ▶ PHYSICS MODELS — v8
          </div>
          <div style={{display:"grid",gridTemplateColumns:"repeat(4,1fr)",gap:6}}>
            {[
              {t:"Glauert NR + κ(V)",  f:"vi²√(V²+vi²)=vh² + κ=1+0.04·(V/10)^1.5",   n:"Leishman §4 — wake interaction"},
              {t:"Motor Model",         f:"η=Pshaft/(I·V) · Kv, Rm, I0 params",         n:"Copper + iron losses estimated"},
              {t:"Battery Sag",         f:"V_sag=I·Rint · Rint=r/C_Ah·T_factor",        n:"IR drop + temperature derating"},
              {t:"Wind Model",          f:"V_eff=V+Vwind·cos(θ) + Vcross CdA penalty",  n:"Headwind/crosswind both modeled"},
              {t:"BVI Penalty",         f:"×(1+0.06·exp(-(μ-0.12)²/0.0036))",           n:"Blade-vortex interaction μ≈0.12"},
              {t:"Descent / VRS",       f:"P_descent×0.4 recovery / VRS bell-curve",    n:"Vortex ring state -0.5 to -3m/s"},
              {t:"ESC Thermal",         f:"T_esc=T_amb+I²R·Rth · derate@80°C",         n:"Overheat → power derating"},
              {t:"Noise SPL",           f:"K+15·log(Vtip/150)+8·log(DL/50)-20·log(d)",  n:"Ffowcs-Williams simplified"},
            ].map(f=>(
              <div key={f.t} style={{background:"#00010a",border:"1px solid #0b1c2c",
                padding:"7px 9px",borderRadius:3}}>
                <div style={{fontSize:7,color:"#00ff9966",letterSpacing:1,
                  marginBottom:2,fontWeight:"bold"}}>{f.t}</div>
                <div style={{fontSize:8,color:"#4466aa",fontStyle:"italic",
                  marginBottom:2,lineHeight:1.4}}>{f.f}</div>
                <div style={{fontSize:6,color:"#1a3a2a"}}>{f.n}</div>
              </div>
            ))}
          </div>
        </div>

        <div style={{marginTop:9,padding:"5px 0",borderTop:"1px solid #0b1c2c",
          display:"flex",justifyContent:"space-between",fontSize:6,color:"#162636",letterSpacing:2}}>
          <span>MULTIROTOR UAV ADVANCED PHYSICS SIMULATOR v8.0</span>
          <span>MOTOR · BAT-SAG · WIND · BVI · VRS · ESC-THERMAL · NOISE · GE · PEUKERT</span>
          <span>A={pw.A.toFixed(4)}m² · DL={DL.toFixed(1)}N/m²</span>
        </div>
      </div>
    </div>
  );
}
