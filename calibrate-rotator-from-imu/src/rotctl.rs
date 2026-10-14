//! Minimal hamlib `rotctld` client (default protocol, not extended mode).
//!
//!   p          -> "<az>\n<el>\n"          or "RPRT <n>\n"
//!   P az el    -> "RPRT <n>\n"
//!   S          -> "RPRT <n>\n"
//!   w <raw>    -> backend-dependent: reply lines and/or "RPRT <n>\n", or nothing
//!
//! Moves go either as `P` or as raw G-code (`w G0 X.. Y..`), per [motion].

use crate::config::{MoveMethod, Motion};
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
    pub motion: Motion,
}

/// How long to wait for a reply to a raw `w` command before assuming the
/// backend sends none.
const RAW_REPLY_WAIT: Duration = Duration::from_millis(1500);
const READ_TIMEOUT: Duration = Duration::from_secs(10);

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
        let mut me = Rotctl { addr: addr.to_string(), conn: None, motion: Motion::default() };
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
        s.set_read_timeout(Some(READ_TIMEOUT)).ok();
        s.set_nodelay(true).ok();
        let w = s.try_clone().map_err(|e| Fail::Rotator(e.to_string()))?;
        self.conn = Some(Conn { r: BufReader::new(s), w });
        Ok(())
    }

    /// Discard anything left unread (e.g. a raw command's late reply) so it
    /// can't be mistaken for the answer to the next command.
    fn drain(c: &mut Conn) -> std::io::Result<()> {
        let n = c.r.buffer().len();
        if n > 0 {
            debug!("rotctld: discarding {n} stale bytes");
            c.r.consume(n);
        }
        c.r.get_ref().set_nonblocking(true)?;
        let mut buf = [0u8; 512];
        let res = loop {
            match std::io::Read::read(c.r.get_mut(), &mut buf) {
                Ok(0) => break Err(std::io::Error::new(std::io::ErrorKind::UnexpectedEof, "rotctld closed connection")),
                Ok(n) => debug!("rotctld: discarding {n} stale bytes"),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break Ok(()),
                Err(e) => break Err(e),
            }
        };
        c.r.get_ref().set_nonblocking(false)?;
        res
    }

    /// With `reply_wait`, a read timeout ends the reply instead of failing:
    /// some backends answer raw commands with nothing at all.
    fn transact_once(&mut self, cmd: &str, data_lines: usize, reply_wait: Option<Duration>) -> Result<Vec<String>, std::io::Error> {
        let c = self.conn.as_mut().ok_or_else(|| std::io::Error::other("not connected"))?;
        Self::drain(c)?;
        c.w.write_all(cmd.as_bytes())?;
        c.w.write_all(b"\n")?;
        c.r.get_ref().set_read_timeout(Some(reply_wait.unwrap_or(READ_TIMEOUT)))?;
        let res = Self::read_reply(c, data_lines, reply_wait.is_some());
        c.r.get_ref().set_read_timeout(Some(READ_TIMEOUT))?;
        res
    }

    fn read_reply(c: &mut Conn, data_lines: usize, timeout_ends: bool) -> Result<Vec<String>, std::io::Error> {
        let mut out = Vec::new();
        loop {
            let mut line = String::new();
            match c.r.read_line(&mut line) {
                Ok(0) => return Err(std::io::Error::new(std::io::ErrorKind::UnexpectedEof, "rotctld closed connection")),
                Ok(_) => {}
                Err(e) if timeout_ends && matches!(e.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut) => {
                    return Ok(out);
                }
                Err(e) => return Err(e),
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
        self.transact_wait(cmd, data_lines, None)
    }

    fn transact_wait(&mut self, cmd: &str, data_lines: usize, reply_wait: Option<Duration>) -> R<Vec<String>> {
        debug!("rotctld <- {cmd}");
        let res = match self.transact_once(cmd, data_lines, reply_wait) {
            Ok(v) => v,
            Err(e) => {
                warn!("rotctld I/O error ({e}); reconnecting");
                self.reconnect()?;
                self.transact_once(cmd, data_lines, reply_wait)
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

    /// Command an absolute position in the controller's reported frame (deg).
    pub fn set_pos(&mut self, az: f64, el: f64) -> R<()> {
        match self.motion.method {
            MoveMethod::Hamlib => self.transact(&format!("P {az:.3} {el:.3}"), 0).map(|_| ()),
            MoveMethod::Gcode => {
                let g = self.gcode_for(az, el);
                self.transact_wait(&format!("w {g}"), 0, Some(RAW_REPLY_WAIT)).map(|_| ())
            }
        }
    }

    pub fn gcode_for(&self, az: f64, el: f64) -> String {
        let k = self.motion.deg_per_gcode_unit;
        let u = |d: f64| format!("{:.4}", d / k + 0.0); // + 0.0 turns -0 into 0
        self.motion.gcode_template.replace("{az}", &u(az)).replace("{el}", &u(el))
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
