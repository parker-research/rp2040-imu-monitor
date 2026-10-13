//! Minimal hamlib `rotctld` client (default protocol, not extended mode).
//!
//!   p          -> "<az>\n<el>\n"          or "RPRT <n>\n"
//!   P az el    -> "RPRT <n>\n"
//!   S          -> "RPRT <n>\n"

use crate::error::{Fail, R};
use log::{debug, warn};
use std::io::{BufRead, BufReader, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

struct Conn {
    r: BufReader<TcpStream>,
    w: TcpStream,
}

pub struct Rotctl {
    addr: String,
    conn: Option<Conn>,
}

fn hamlib_err(code: i32) -> &'static str {
    match -code {
        1 => "invalid parameter (outside rotctld's configured min/max limits?)",
        2 => "invalid configuration",
        4 => "function not implemented",
        5 => "communication timed out",
        6 => "I/O error",
        7 => "internal hamlib error",
        8 => "protocol error",
        9 => "command rejected by rotator",
        11 => "feature not available",
        _ => "error",
    }
}

impl Rotctl {
    pub fn connect(addr: &str) -> R<Self> {
        let mut me = Rotctl { addr: addr.to_string(), conn: None };
        me.reconnect()?;
        Ok(me)
    }

    fn reconnect(&mut self) -> R<()> {
        self.conn = None;
        let sa = self
            .addr
            .to_socket_addrs()
            .map_err(|e| Fail::Rotator(format!("bad rotctld address {}: {e}", self.addr)))?
            .next()
            .ok_or_else(|| Fail::Rotator(format!("cannot resolve {}", self.addr)))?;
        let s = TcpStream::connect_timeout(&sa, Duration::from_secs(3))
            .map_err(|e| Fail::Rotator(format!("cannot connect to rotctld at {}: {e}", self.addr)))?;
        s.set_read_timeout(Some(Duration::from_secs(10))).ok();
        s.set_nodelay(true).ok();
        let w = s.try_clone().map_err(|e| Fail::Rotator(e.to_string()))?;
        self.conn = Some(Conn { r: BufReader::new(s), w });
        Ok(())
    }

    fn transact_once(&mut self, cmd: &str, data_lines: usize) -> Result<Vec<String>, std::io::Error> {
        let c = self.conn.as_mut().ok_or_else(|| std::io::Error::other("not connected"))?;
        c.w.write_all(cmd.as_bytes())?;
        c.w.write_all(b"\n")?;
        let mut out = Vec::new();
        loop {
            let mut line = String::new();
            if c.r.read_line(&mut line)? == 0 {
                return Err(std::io::Error::new(std::io::ErrorKind::UnexpectedEof, "rotctld closed connection"));
            }
            let t = line.trim().to_string();
            if t.starts_with("RPRT") {
                out.push(t);
                return Ok(out);
            }
            out.push(t);
            if out.len() == data_lines {
                return Ok(out);
            }
        }
    }

    /// Send a command, reconnecting once on I/O failure. Returns data lines.
    fn transact(&mut self, cmd: &str, data_lines: usize) -> R<Vec<String>> {
        debug!("rotctld <- {cmd}");
        let res = match self.transact_once(cmd, data_lines) {
            Ok(v) => v,
            Err(e) => {
                warn!("rotctld I/O error ({e}); reconnecting");
                self.reconnect()?;
                self.transact_once(cmd, data_lines)
                    .map_err(|e| Fail::Rotator(format!("rotctld I/O error: {e}")))?
            }
        };
        debug!("rotctld -> {res:?}");
        if let Some(last) = res.last() {
            if let Some(code) = last.strip_prefix("RPRT") {
                let code: i32 = code.trim().parse().unwrap_or(-8);
                if code != 0 {
                    let msg = format!("`{cmd}` -> RPRT {code} ({})", hamlib_err(code));
                    return Err(if code == -1 || code == -9 { Fail::Rejected(msg) } else { Fail::Rotator(msg) });
                }
            }
        }
        Ok(res)
    }

    pub fn get_pos(&mut self) -> R<(f64, f64)> {
        let v = self.transact("p", 2)?;
        let parse = |s: Option<&String>| -> R<f64> {
            s.and_then(|x| x.parse::<f64>().ok())
                .ok_or_else(|| Fail::Rotator(format!("unparseable get_pos reply: {v:?}")))
        };
        Ok((parse(v.first())?, parse(v.get(1))?))
    }

    pub fn set_pos(&mut self, az: f64, el: f64) -> R<()> {
        self.transact(&format!("P {az:.3} {el:.3}"), 0).map(|_| ())
    }

    /// Send a raw backend command (rotctld `w ...`). The controller may reboot
    /// without answering, so a timeout or closed connection is not an error.
    /// The connection is dropped afterwards so the next call reconnects.
    pub fn send_raw(&mut self, cmd: &str, wait: Duration) -> R<Vec<String>> {
        debug!("rotctld <- {cmd} (raw)");
        if self.conn.is_none() {
            self.reconnect()?;
        }
        let ok = {
            let c = self.conn.as_mut().unwrap();
            c.w.write_all(format!("{cmd}\n").as_bytes()).is_ok()
        };
        if !ok {
            self.reconnect()?;
            let c = self.conn.as_mut().unwrap();
            c.w.write_all(format!("{cmd}\n").as_bytes())
                .map_err(|e| Fail::Rotator(format!("cannot send `{cmd}`: {e}")))?;
        }
        let mut out = Vec::new();
        {
            let c = self.conn.as_mut().unwrap();
            c.r.get_ref().set_read_timeout(Some(wait)).ok();
            loop {
                let mut line = String::new();
                match c.r.read_line(&mut line) {
                    Ok(0) | Err(_) => break,
                    Ok(_) => {
                        let t = line.trim().to_string();
                        let done = t.starts_with("RPRT");
                        out.push(t);
                        if done {
                            break;
                        }
                    }
                }
            }
        }
        self.conn = None;
        debug!("rotctld -> {out:?}");
        Ok(out)
    }

    pub fn stop(&mut self) -> R<()> {
        self.transact("S", 0).map(|_| ())
    }
}

/// Best-effort out-of-band stop (used from the Ctrl-C handler).
pub fn emergency_stop(addr: &str) {
    if let Some(sa) = addr.to_socket_addrs().ok().and_then(|mut a| a.next()) {
        if let Ok(mut s) = TcpStream::connect_timeout(&sa, Duration::from_secs(2)) {
            let _ = s.write_all(b"S\n");
            let _ = s.flush();
        }
    }
}
