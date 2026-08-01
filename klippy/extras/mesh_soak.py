# Beta tooling: run N bed meshes, capture force data, bundle for upload
#
# klippy/extras/mesh_soak.py
#
# [mesh_soak] in printer.cfg, then: MESH_SOAK [NUM=<n>] [BED_TEMP=<t>]
import io, json, logging, os, shutil, subprocess, time

README = """hx711s mesh soak results
========================
Printer build: %(version)s
Runs: %(runs)s

HOW TO UPLOAD
1. Go to %(pr_url)s
2. Drag this .tar.gz into a new comment
3. Add your printer model, bed temp, and anything unusual you saw
   (skips, retries, "Bad tap" messages, hotend blobs, etc.)

THANK YOU for testing!
"""

class MeshSoak:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.out_dir = config.get('output_dir', '/user-resource')
        self.pr_url = config.get('pr_url',
            '# NEXT PR #')
        self.hx = None
        self.capture = None      # file handle while soaking
        self.cbuf = []           # buffered sample lines
        self._bundle_timer = None
        self._bundle_proc = None
        self._bundle_ctx = None
        self.printer.register_event_handler('klippy:connect',
                                            self._connect)
        self.gcode.register_command(
            'MESH_SOAK', self.cmd_MESH_SOAK,
            desc="Run NUM bed meshes and bundle results for the PR")

    def _connect(self):
        # the sensor is not registered by name; reach it through the probe
        probe = self.printer.lookup_object('load_cell_probe', None)
        if probe is not None:
            try:
                self.hx = probe._load_cell.get_sensor()
            except Exception:
                self.hx = None
        if self.hx is not None and hasattr(self.hx, 'add_sample_listener'):
            self.hx.add_sample_listener(self._on_samples)

    # capture runs on klippy's thread; buffer and flush in chunks
    def _on_samples(self, batch):
        if self.capture is None:
            return
        self.cbuf.append(json.dumps({'data': batch}) + '\n')
        if len(self.cbuf) >= 40:
            # an OSError here (ENOSPC on /user-resource) would propagate to
            # the reactor and shut klippy down mid-probe; lose the data instead
            try:
                self.capture.writelines(self.cbuf)
            except OSError:
                logging.exception("mesh_soak: capture write failed, "
                                  "aborting capture")
                self.capture.close()
                self.capture = None
            self.cbuf = []

    def _settings_text(self):
        sa = self.printer.get_start_args()
        lines = ['software_version: %s' % sa.get('software_version')]
        # MCU firmware build IDs (SRCREV-PR), when present on the host
        for mcu in ('bed', 'toolhead'):
            try:
                with open('/lib/firmware/klipper-%s.bin.ver' % mcu) as f:
                    lines.append('klipper-%s.ver: %s' % (mcu, f.read().strip()))
            except OSError:
                pass
        if self.hx is not None:
            for k in ('torn_retries', 'stuck_ms', 'settle_ms',
                      'spike_sum_threshold', 'sps', 'recovered_count'):
                lines.append('%s: %s' % (k, getattr(self.hx, k, '?')))
        return '\n'.join(lines) + '\n'

    def cmd_MESH_SOAK(self, gcmd):
        n = gcmd.get_int('NUM', 5, minval=1, maxval=50)
        bed_temp = gcmd.get_int('BED_TEMP', 60, minval=0)
        if self.hx is None:
            gcmd.respond_info('MESH_SOAK: no hx711s sensor found - force '
                              'capture will be EMPTY (check sensor wiring)')
            logging.warning('MESH_SOAK: no hx711s sensor, capture disabled')
        ts = time.strftime('%Y%m%d-%H%M%S')
        # settings go to klippy.log too, so plain logs are useful on their own
        settings = self._settings_text()
        logging.info('MESH_SOAK start ts=%s n=%d bed_temp=%d\n%s',
                     ts, n, bed_temp, settings)
        cap_path = os.path.join(self.out_dir, 'mesh-soak-%s-force.jsonl' % ts)
        try:
            self.capture = open(cap_path, 'w')
        except OSError:
            cap_path = os.path.join('/tmp', os.path.basename(cap_path))
            try:
                self.capture = open(cap_path, 'w')
            except OSError as e:
                gcmd.respond_info('MESH_SOAK: cannot open capture file: %s' % e)
                return
        results = []
        try:
            for i in range(1, n + 1):
                if self.printer.is_shutdown():
                    gcmd.respond_info('MESH_SOAK: printer shutdown, stopping')
                    break
                t0 = time.time()
                try:
                    if bed_temp:
                        self.gcode.run_script_from_command(
                            'M140 S%d\nM190 S%d' % (bed_temp, bed_temp))
                    self.gcode.run_script_from_command('BED_MESH_CALIBRATE')
                    rc, err = 'ok', ''
                except Exception as e:
                    rc, err = 'FAIL', str(e)
                dur = time.time() - t0
                results.append((i, rc, dur, err))
                gcmd.respond_info('MESH_SOAK run %d/%d: %s (%.0fs) %s'
                                  % (i, n, rc, dur, err))
                logging.info('MESH_SOAK run %d/%d: %s %.1fs %s',
                             i, n, rc, dur, err)
                if bed_temp:
                    try:
                        self.gcode.run_script_from_command('M140 S0')
                    except Exception:
                        pass
        finally:
            # flush/close can raise ENOSPC too; never let it escape the
            # finally and mask the mesh results (or shut klippy down)
            try:
                if self.cbuf and self.capture is not None:
                    self.capture.writelines(self.cbuf)
                    self.cbuf = []
                if self.capture is not None:
                    self.capture.close()
            except OSError:
                logging.exception('MESH_SOAK: capture flush/close failed')
            self.capture = None
        if self._start_bundle(ts, settings, results, cap_path, gcmd):
            gcmd.respond_info(
                'MESH_SOAK: runs done, compressing bundle in background '
                '(reniced tar, klippy not blocked)...')
        else:
            gcmd.respond_info('MESH_SOAK: FAILED to stage bundle (see log)')

    # --- background bundling ---------------------------------------------
    # tar+gzip of a 40MB+ capture must never run on klippy's thread: the
    # child is reniced (busybox has no `nice`, so preexec_fn) and completion
    # is polled from a reactor timer, same pattern as gcode_shell_command.
    def _start_bundle(self, ts, settings, results, cap_path, gcmd):
        if self._bundle_proc is not None:
            logging.warning('MESH_SOAK: previous bundle still running, '
                            'not starting another (capture left at %s)',
                            cap_path)
            return False
        version = self.printer.get_start_args().get('software_version')
        rtxt = ''.join('run %d: %s %.1fs %s\n' % r for r in results)
        readme = README % {'version': version, 'runs': rtxt,
                           'pr_url': self.pr_url}
        name = 'mesh-results-%s' % ts
        for d in (self.out_dir, '/tmp'):
            work = os.path.join(d, name)
            out_path = work + '.tar.gz'
            try:
                os.makedirs(work, exist_ok=True)
                for fname, text in (('README.txt', readme),
                                    ('settings.txt', settings),
                                    ('results.txt', rtxt)):
                    with open(os.path.join(work, fname), 'w') as f:
                        f.write(text)
                if os.path.dirname(os.path.abspath(cap_path)) == \
                        os.path.abspath(d):
                    os.link(cap_path, os.path.join(work, 'force.jsonl'))
                else:
                    # blocking copy on klippy's thread; only hit on the /tmp
                    # fallback path. Do not cite this as a pattern.
                    shutil.copy(cap_path, os.path.join(work, 'force.jsonl'))
                log_file = self.printer.get_start_args().get('log_file')
                if log_file and os.path.exists(log_file):
                    shutil.copy(log_file, os.path.join(work, 'klippy.log'))
                # stderr to a file, not a PIPE: a PIPE drained only after
                # exit can deadlock the child if it ever gets chatty
                with open(os.path.join(work, 'tar.stderr'), 'w') as errf:
                    self._bundle_proc = subprocess.Popen(
                        ['tar', 'czf', out_path, '-C', d, name],
                        preexec_fn=lambda: os.nice(19),
                        stdout=subprocess.DEVNULL, stderr=errf)
                self._bundle_ctx = (work, out_path, cap_path)
                self._bundle_timer = self.reactor.register_timer(
                    self._bundle_poll, self.reactor.monotonic() + 1.0)
                return True
            except OSError as e:
                logging.warning('MESH_SOAK: cannot stage %s: %s', out_path, e)
                shutil.rmtree(work, ignore_errors=True)
        logging.warning('MESH_SOAK: bundle staging failed everywhere, '
                        'capture left at %s', cap_path)
        return False

    def _bundle_poll(self, eventtime):
        proc = self._bundle_proc
        if proc is None or self._bundle_ctx is None:
            return self.reactor.NEVER
        rc = proc.poll()
        if rc is None:
            return eventtime + 1.0
        work, out_path, cap_path = self._bundle_ctx
        try:
            with open(os.path.join(work, 'tar.stderr')) as f:
                err = f.read()
        except OSError:
            err = ''
        self._bundle_proc = self._bundle_ctx = self._bundle_timer = None
        if rc == 0:
            try:
                os.unlink(cap_path)
            except OSError:
                pass
            shutil.rmtree(work, ignore_errors=True)
            msg = ('MESH_SOAK complete: %s\n'
                   'Upload this file to the PR to contribute test data:\n%s'
                   % (out_path, self.pr_url))
        else:
            # failure: don't leave a 40MB landfill on the partition
            shutil.rmtree(work, ignore_errors=True)
            try:
                os.unlink(out_path)
            except OSError:
                pass
            msg = 'MESH_SOAK: tar failed rc=%d: %s' % (rc, err[-300:])
        logging.info(msg.replace('\n', ' | '))
        try:
            self.gcode.respond_info(msg)
        except Exception:
            pass
        return self.reactor.NEVER

def load_config(config):
    return MeshSoak(config)
