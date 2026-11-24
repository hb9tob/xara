//! Records a radio interface both ways at once on Windows, for working out an on-air exchange:
//! what comes in from the radio (the capture device) and, through WASAPI loopback, exactly what
//! a program such as VARA plays into it. Two 48 kHz mono WAVs on one clock: each packet goes in
//! at its QPC time, so the gaps loopback leaves while nothing plays come out as silence.
//!
//!     cargo run --release -p xara --example wasapi_rec -- "USB Audio CODEC" 300 OUT_DIR
//!
//! writes OUT_DIR\rx.wav and OUT_DIR\tx.wav (the capture and render devices whose names contain
//! the given text), for the given number of seconds.

#[cfg(windows)]
mod imp {
    use std::fs::File;
    use std::io::{self, BufWriter};
    use std::path::Path;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::thread;
    use std::time::{Duration, Instant};

    use windows::Win32::Devices::FunctionDiscovery::PKEY_Device_FriendlyName;
    use windows::Win32::Media::Audio::{
        AUDCLNT_BUFFERFLAGS_SILENT, AUDCLNT_SHAREMODE_SHARED, AUDCLNT_STREAMFLAGS_AUTOCONVERTPCM,
        AUDCLNT_STREAMFLAGS_LOOPBACK, AUDCLNT_STREAMFLAGS_SRC_DEFAULT_QUALITY, DEVICE_STATE_ACTIVE, EDataFlow,
        IAudioCaptureClient, IAudioClient, IMMDevice, IMMDeviceEnumerator, MMDeviceEnumerator, WAVE_FORMAT_PCM,
        WAVEFORMATEX, eCapture, eRender,
    };
    use windows::Win32::System::Com::{CLSCTX_ALL, COINIT_MULTITHREADED, CoCreateInstance, CoInitializeEx, STGM_READ};
    use xara::wav;

    const RATE: u32 = 48000;
    /// The shared-mode buffer asked for, in 100 ns units: 1 s, plenty for a 10 ms poll.
    const BUFFER_HNS: i64 = 10_000_000;
    /// A packet this late (20 ms) after the last one ended starts after a gap.
    const GAP: usize = RATE as usize / 50;

    fn err(what: &str) -> impl Fn(windows::core::Error) -> io::Error + '_ {
        move |e| io::Error::other(format!("{what}: {e}"))
    }

    fn find(flow: EDataFlow, name: &str) -> io::Result<(IMMDevice, String)> {
        // SAFETY: plain COM enumeration on live objects.
        unsafe {
            let _ = CoInitializeEx(None, COINIT_MULTITHREADED);
            let e: IMMDeviceEnumerator =
                CoCreateInstance(&MMDeviceEnumerator, None, CLSCTX_ALL).map_err(err("devices"))?;
            let c = e.EnumAudioEndpoints(flow, DEVICE_STATE_ACTIVE).map_err(err("devices"))?;
            let low = name.to_lowercase();
            let mut hits = Vec::new();
            for i in 0..c.GetCount().map_err(err("devices"))? {
                let d = c.Item(i).map_err(err("device"))?;
                let n = d.OpenPropertyStore(STGM_READ).and_then(|s| s.GetValue(&PKEY_Device_FriendlyName));
                let n = n.map(|v| v.to_string()).unwrap_or_default();
                if n.to_lowercase().contains(&low) {
                    hits.push((d, n));
                }
            }
            match hits.len() {
                1 => Ok(hits.pop().unwrap()),
                0 => Err(io::Error::other(format!("no device named like {name:?}"))),
                _ => Err(io::Error::other(format!(
                    "{name:?} fits several: {}",
                    hits.iter().map(|h| h.1.as_str()).collect::<Vec<_>>().join(", ")
                ))),
            }
        }
    }

    /// Packets as (QPC time in 100 ns, samples), until stop.
    fn record(flow: EDataFlow, name: String, stop: Arc<AtomicBool>) -> io::Result<Vec<(u64, Vec<i16>)>> {
        let (dev, full) = find(flow, &name)?;
        let loopback = flow == eRender;
        eprintln!("{}: {full}", if loopback { "tx (loopback)" } else { "rx" });
        let mut out = Vec::new();
        // SAFETY: COM calls on live objects; each buffer is copied and released before the next.
        unsafe {
            let client: IAudioClient = dev.Activate(CLSCTX_ALL, None).map_err(err("audio client"))?;
            let fmt = WAVEFORMATEX {
                wFormatTag: WAVE_FORMAT_PCM as u16,
                nChannels: 1,
                nSamplesPerSec: RATE,
                nAvgBytesPerSec: RATE * 2,
                nBlockAlign: 2,
                wBitsPerSample: 16,
                cbSize: 0,
            };
            let mut flags = AUDCLNT_STREAMFLAGS_AUTOCONVERTPCM | AUDCLNT_STREAMFLAGS_SRC_DEFAULT_QUALITY;
            if loopback {
                flags |= AUDCLNT_STREAMFLAGS_LOOPBACK;
            }
            client
                .Initialize(AUDCLNT_SHAREMODE_SHARED, flags, BUFFER_HNS, 0, &fmt, None)
                .map_err(err("audio format"))?;
            let cap: IAudioCaptureClient = client.GetService().map_err(err("capture client"))?;
            client.Start().map_err(err("start"))?;
            while !stop.load(Ordering::SeqCst) {
                thread::sleep(Duration::from_millis(10));
                while cap.GetNextPacketSize().map_err(err("packet"))? > 0 {
                    let (mut data, mut frames, mut fl, mut qpc) = (std::ptr::null_mut(), 0u32, 0u32, 0u64);
                    cap.GetBuffer(&mut data, &mut frames, &mut fl, None, Some(&mut qpc)).map_err(err("buffer"))?;
                    let x = if fl & AUDCLNT_BUFFERFLAGS_SILENT.0 as u32 != 0 || data.is_null() {
                        vec![0; frames as usize]
                    } else {
                        std::slice::from_raw_parts(data.cast::<i16>(), frames as usize).to_vec()
                    };
                    cap.ReleaseBuffer(frames).map_err(err("release"))?;
                    out.push((qpc, x));
                }
            }
            let _ = client.Stop();
        }
        Ok(out)
    }

    fn write(path: &Path, packets: &[(u64, Vec<i16>)], t0: u64, len: usize) -> io::Result<()> {
        // Packets follow on from each other; their QPC time only places the first one and
        // the restart after a gap (loopback delivers nothing while nothing plays), since
        // its jitter would otherwise tear the waveform apart.
        let mut x = vec![0i16; len];
        let mut at = 0usize;
        for (qpc, p) in packets {
            let due = ((qpc - t0) as u128 * u128::from(RATE) / 10_000_000) as usize;
            if due > at + GAP || at == 0 {
                at = due;
            }
            for (i, &v) in p.iter().enumerate() {
                if let Some(s) = x.get_mut(at + i) {
                    *s = v;
                }
            }
            at += p.len();
        }
        let mut w = wav::Writer::new(BufWriter::new(File::create(path)?), 1, RATE)?;
        w.write(&x)?;
        w.finish()?;
        Ok(())
    }

    pub fn main() -> io::Result<()> {
        let a: Vec<String> = std::env::args().skip(1).collect();
        let [name, secs, dir] = a.as_slice() else {
            return Err(io::Error::other("usage: wasapi_rec DEVICE SECONDS OUT_DIR"));
        };
        let secs: u64 = secs.parse().map_err(|_| io::Error::other("SECONDS: a number"))?;
        std::fs::create_dir_all(dir)?;
        let stop = Arc::new(AtomicBool::new(false));
        // Enter or Ctrl-C ends the recording early and still writes it
        let done = Arc::new(AtomicBool::new(false));
        let _ = xara::signals::trap(&xara::signals::STOP, done.clone(), || {});
        let enter = done.clone();
        thread::spawn(move || {
            // (no console, as when started hidden, reads nothing: that isn't Enter)
            if io::stdin().read_line(&mut String::new()).is_ok_and(|n| n > 0) {
                enter.store(true, Ordering::SeqCst);
            }
        });
        let threads: Vec<_> = [eCapture, eRender]
            .into_iter()
            .map(|flow| {
                let (n, s) = (name.clone(), stop.clone());
                thread::spawn(move || record(flow, n, s))
            })
            .collect();
        let start = Instant::now();
        eprintln!("recording at most {secs} s into {dir}; Enter (or Ctrl-C) stops it and saves");
        while start.elapsed() < Duration::from_secs(secs)
            && !done.load(Ordering::SeqCst)
            && !threads.iter().any(|t| t.is_finished())
        {
            thread::sleep(Duration::from_millis(200));
        }
        stop.store(true, Ordering::SeqCst);
        let [rx, tx] = <[_; 2]>::try_from(threads).ok().unwrap().map(|t| t.join().unwrap());
        let (rx, tx) = (rx?, tx?);
        let first = |p: &[(u64, Vec<i16>)]| p.first().map(|f| f.0);
        let t0 = [first(&rx), first(&tx)].into_iter().flatten().min().unwrap_or(0);
        let len = (start.elapsed().as_secs_f64() * f64::from(RATE)) as usize;
        write(&Path::new(dir).join("rx.wav"), &rx, t0, len)?;
        write(&Path::new(dir).join("tx.wav"), &tx, t0, len)?;
        let n = |p: &[(u64, Vec<i16>)]| p.iter().map(|q| q.1.len()).sum::<usize>() as f64 / f64::from(RATE);
        eprintln!(
            "rx {:.1} s of audio, tx {:.1} s of audio (loopback only runs while something plays)",
            n(&rx),
            n(&tx)
        );
        Ok(())
    }
}

fn main() -> std::process::ExitCode {
    #[cfg(windows)]
    if let Err(e) = imp::main() {
        eprintln!("wasapi_rec: {e}");
        return std::process::ExitCode::FAILURE;
    }
    #[cfg(not(windows))]
    eprintln!("wasapi_rec is for Windows");
    std::process::ExitCode::SUCCESS
}
