#!/usr/bin/env python3
"""Speaks the voice messages of the LXNAV V8 instrument with the macOS `say` command.

FlightGear's Nasal cannot start programs, so the instrument writes the text of a message into
/instrumentation/lxv8/voice/text (and increases .../seq). This helper reads these properties over FlightGear's telnet
interface (start FlightGear with --telnet=5401) and speaks the text with `say`, in the system voice, like FG9070 does
in the browser. While the helper runs it also writes .../voice/helper-time every poll, so the instrument knows that it
is there and uses it; without the helper the instrument plays the prerecorded clips of Sounds/lxv8-voice/.

    python3 tools/lxv8-speak.py [port]            (stop it with Ctrl-C)
"""
import re, socket, subprocess, sys, tempfile, time, os

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 5401
BASE = '/instrumentation/lxv8'

class Telnet:
    def __init__(self, port):
        self.s = socket.create_connection(('127.0.0.1', port), timeout=5)
        self.buf = b''
        self.s.settimeout(5)
    def read_until_prompt(self):
        while b'/> ' not in self.buf:
            d = self.s.recv(4096)
            if not d: raise ConnectionError('closed')
            self.buf += d
        out, _, self.buf = self.buf.partition(b'/> ')
        return out.decode('utf-8', 'replace')
    def cmd(self, c):
        self.s.sendall((c + '\r\n').encode())
        return self.read_until_prompt()
    def get(self, path):
        m = re.search(r"=\s*'(.*)'\s*\(", self.cmd('get ' + path), re.S)
        return m.group(1) if m else None
    def set(self, path, value):
        self.cmd('set %s %s' % (path, value))

def speak(text, volume):
    with tempfile.NamedTemporaryFile(suffix='.aiff', delete=False) as f:
        path = f.name
    try:
        subprocess.run(['say', '-o', path, text], check=True)           # no -v: the system voice, like the browser
        subprocess.run(['afplay', '-v', '%.2f' % max(0.05, min(1.0, volume)), path], check=False)
    finally:
        os.unlink(path)

def main():
    last_seq = None
    while True:
        try:
            t = Telnet(PORT)
            print('connected to FlightGear on port %d' % PORT)
            while True:
                t.set(BASE + '/voice/helper-time', '%.1f' % time.time())
                seq = t.get(BASE + '/voice/seq')
                if last_seq is None:
                    last_seq = seq                      # do not speak an old message
                elif seq != last_seq:
                    last_seq = seq
                    text = t.get(BASE + '/voice/text')
                    vol = float(t.get(BASE + '/audio/voice-volume') or 0.8)
                    if text and vol > 0:
                        print('say:', text)
                        speak(text, vol)
                time.sleep(0.15)
        except (OSError, ConnectionError):
            print('waiting for FlightGear (telnet port %d) ...' % PORT)
            last_seq = None
            time.sleep(2)
        except KeyboardInterrupt:
            print('bye')
            return

if __name__ == '__main__':
    main()
