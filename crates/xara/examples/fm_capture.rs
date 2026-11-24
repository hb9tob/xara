//! Lists what one direction of a VARA FM recording carries, to work out the autotune and ping
//! exchanges the air interface document only names (record 0 kinds 1, 2 and 3):
//!
//!     cargo run --release -p xara --example fm_capture -- A_to_B.wav narrow CALLA CALLB
//!
//! The recording is 48 kHz mono (one virtual cable, one direction). Printed in time order:
//! every burst of audio with its level (so the drive steps of an autotune show), whether it is
//! the 1500 Hz tune tone, every record 0 frame decoded with its fields and raw bytes, and every
//! control code heard with either station's codebook.

use std::process::ExitCode;
use xara::wav;
use xara_phy::fm::ctrl::{Codebook, Side};
use xara_phy::fm::frame::Record0;
use xara_phy::fm::level::{FmBandwidth, Record};
use xara_phy::fm::rx::receiver::{Event, Receiver};
use xara_testkit::hex;

const RATE: f64 = 48000.0;
const WIN: usize = 480;
const GATE_DBFS: f64 = -60.0;
const HANG: usize = 10;

fn db(power: f64) -> f64 {
    10.0 * power.max(1e-20).log10()
}

/// Share of a segment's power at 1500 Hz (Goertzel over the whole segment).
fn tone_share(x: &[f64]) -> f64 {
    let w = 2.0 * std::f64::consts::PI * 1500.0 / RATE;
    let (mut s1, mut s2) = (0.0, 0.0);
    for &v in x {
        let s = v + 2.0 * w.cos() * s1 - s2;
        (s2, s1) = (s1, s);
    }
    let p = s1 * s1 + s2 * s2 - 2.0 * w.cos() * s1 * s2;
    let total: f64 = x.iter().map(|v| v * v).sum();
    (2.0 * p / x.len() as f64 / total.max(1e-20)).min(1.0)
}

fn bursts(x: &[f64]) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    let (mut start, mut quiet) = (None, 0);
    for (i, w) in x.chunks(WIN).enumerate() {
        let loud = db(w.iter().map(|v| v * v).sum::<f64>() / w.len() as f64) > GATE_DBFS;
        match (start, loud) {
            (None, true) => (start, quiet) = (Some(i), 0),
            (Some(_), true) => quiet = 0,
            (Some(s), false) => {
                quiet += 1;
                if quiet > HANG {
                    out.push((s * WIN, (i - quiet + 1) * WIN));
                    start = None;
                }
            }
            (None, false) => {}
        }
    }
    if let Some(s) = start {
        out.push((s * WIN, x.len()));
    }
    out
}

fn main() -> ExitCode {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let [path, bw, call_a, call_b] = a.as_slice() else {
        eprintln!("usage: fm_capture FILE.wav narrow|wide CALL_A CALL_B");
        return ExitCode::FAILURE;
    };
    let bw = if bw == "wide" { FmBandwidth::Wide } else { FmBandwidth::Narrow };
    let w = match wav::read(path.as_ref()) {
        Ok(w) if w.rate == 48000 => w,
        Ok(w) => {
            eprintln!("{path}: {} Hz, needs 48000", w.rate);
            return ExitCode::FAILURE;
        }
        Err(e) => {
            eprintln!("{path}: {e}");
            return ExitCode::FAILURE;
        }
    };
    let x = &w.samples;
    let (ua, ub) = (call_a.to_uppercase(), call_b.to_uppercase());
    let mut lines: Vec<(f64, String)> = Vec::new();

    for (s, e) in bursts(x) {
        let seg = &x[s..e];
        let rms = db(seg.iter().map(|v| v * v).sum::<f64>() / seg.len() as f64);
        let peak = db(seg.iter().fold(0.0f64, |m, v| m.max(v * v)));
        let tone = tone_share(seg);
        let what = if tone > 0.9 { "  TUNE TONE 1500 Hz" } else { "" };
        lines.push((
            s as f64 / RATE,
            format!(
                "burst {:.3} s  rms {rms:6.1} dBFS  peak {peak:6.1} dBFS  1500 Hz {:3.0}%{what}",
                (e - s) as f64 / RATE,
                100.0 * tone
            ),
        ));
    }

    // One receiver per codebook orientation: the station that asks and the one that answers
    let codes: Vec<u32> = (1..=24).collect();
    for (up, down) in [(&ua, &ub), (&ub, &ua)] {
        let mut rx = Receiver::new(bw, 0, &[Record::Connect], 1);
        let cb = Codebook::for_link(up.as_bytes(), down.as_bytes());
        rx.set_ctrl(Side::Source, Some(cb), &codes);
        let mut ev: Vec<Event> = x.chunks(WIN).flat_map(|b| rx.feed(b)).collect();
        ev.extend(rx.flush());
        for e in ev {
            match e {
                Event::Ctrl(c) => lines.push((
                    c.t_end48() as f64 / RATE,
                    format!("ctrl code {:2} (codebook {up}->{down}, {} hits)", c.det.code, c.det.hits),
                )),
                Event::Frame(f) if up == &ua => {
                    let t = f.t_end48() as f64 / RATE;
                    let Some(r) = f.ok.then(|| Record0::parse(&f.bytes)).flatten() else {
                        lines.push((t, format!("record 0 frame, not decoded (snr {:.1} dB)", f.snr_db)));
                        continue;
                    };
                    lines.push((
                        t,
                        format!(
                            "record 0 {:?}: {} -> {} via [{} {}] hop {} reg {} wide {} | info 0x{:02x} flags 0x{:02x} control 0x{:02x} chain snr {} ({:.1} dB) | snr {:.1} dB\n           {}",
                            r.kind,
                            r.source,
                            r.target,
                            r.via1,
                            r.via2,
                            r.hop,
                            r.registered,
                            r.wide,
                            f.bytes[33],
                            f.bytes[34],
                            r.control,
                            r.chain_snr,
                            f64::from(r.chain_snr) / 7.0,
                            f.snr_db,
                            hex::encode(&f.bytes),
                        ),
                    ));
                }
                _ => {}
            }
        }
    }

    lines.sort_by(|a, b| a.0.total_cmp(&b.0));
    for (t, l) in lines {
        println!("{t:9.3}  {l}");
    }
    ExitCode::SUCCESS
}
