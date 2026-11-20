function MotorXploer_Lab21_AutoRun()
%MOTORXPLOER_LAB21_AUTORUN  Automated Lab 2.1 DC-motor PWM sweep on MotorXploer.slx
%
%   Put this file next to MotorXploer.slx and run from the Command Window:
%       >> MotorXploer_Lab21_AutoRun
%
%   For each frequency set -> frequency -> round (1..3) it starts the model
%   once, sweeps PWM duty 0:10:100 % (10 s per step), sets PWM Range = 0,
%   stops the model, slices exact 10 s windows of the 4 logged signals,
%   saves the round to a .mat file, then rests the motor for 300 s.
%
%   Everything runs from the single model MotorXploer.slx. The script turns
%   on signal logging for the Host Serial Rx outputs IN MEMORY ONLY; the
%   .slx file on disk is never saved or modified.
%
%   Safety: Ctrl+C or any error forces PWM Range = 0, streams it to the
%   board, stops the simulation, and saves whatever steps of the current
%   round were completed (*_PARTIAL.mat). If the model can no longer stream,
%   a PWM = 0 packet is written straight to the serial port as a fallback.
%
%   This is a function file rather than a plain script on purpose: onCleanup
%   only fires on Ctrl+C when it lives inside a function workspace.

%% ========================= CONFIGURATION (edit here) =========================
cfg = struct();

% --- Model & block paths ------------------------------------------------------
cfg.ModelName  = 'MotorXploer';
cfg.ModelDir   = fileparts(mfilename('fullpath'));   % folder holding MotorXploer.slx
cfg.BlkPWM     = 'MotorXploer/PWM Range';            % Constant: 0..65535 = 0..100 % duty
cfg.BlkFreq    = 'MotorXploer/Motor Frequency';      % Constant: PWM frequency [Hz]
cfg.BlkFreqSat = 'MotorXploer/Saturation1';          % Saturation after Motor Frequency
cfg.BlkRx      = 'MotorXploer/Host Serial Rx1';      % board -> host packet
cfg.BlkSetup   = 'MotorXploer/Host Serial Setup';    % holds the serial port name

% --- Logged signals (Host Serial Rx1 output ports; port 1 = READY, 6 = stepper echo)
cfg.Signals = struct( ...
    'field', {'omega',                  'loadcell',        'current',       'pwm_echo'}, ...
    'name',  {'Motor Angular Velocity', 'Load Cell (PA0)', 'Current (PA1)', 'PWM Range (echo)'}, ...
    'port',  {2,                        3,                 4,               5}, ...
    'unit',  {'rad/s',                  'ADC raw',         'ADC raw',       'PWM counts'});

% --- Test matrix --------------------------------------------------------------
SET1_FREQS_HZ = [5 50 500 5000 50000];   % shorten to [5 50 500 5000] if needed
cfg.FreqSets  = { SET1_FREQS_HZ, ...
                  [1 10 100 1000 10000], ...
                  [2 20 200 2000 20000] };
cfg.NumRounds    = 3;
cfg.DutyPct      = 0:10:100;              % 11 steps
cfg.PWMFullScale = 65535;                 % PWM = round(duty/100 * 65535)

% --- Timing [s] ---------------------------------------------------------------
cfg.StepDuration_s = 10;    % logged window per duty step
cfg.Cooldown_s     = 300;   % motor rest (PWM = 0) after every round
cfg.PreRoll_s      = 2;     % PWM = 0 after sim start so the serial link settles
cfg.StopFlush_s    = 0.5;   % PWM = 0 streamed this long before the sim is stopped
cfg.StartTimeout_s = 60;    % model compile + serial open

% --- Behaviour ----------------------------------------------------------------
% MotorXploer/Saturation1 clips Motor Frequency to [0 20000] Hz, so 50000 Hz
% would silently run at 20000 Hz. Choose what to do with such frequencies:
cfg.OverLimitAction = 'skip';   % 'skip' | 'clip' (run at limit, recorded) | 'error'
cfg.Resume          = true;     % skip rounds whose result file already exists
cfg.EchoTolerance   = 2;        % warn if PWM echo differs from command by more (counts)
cfg.OutDir          = fullfile(cfg.ModelDir, 'Lab21_Data');
%% ==============================================================================

% ---- Run state: a handle object, so the onCleanup safety handler still sees
% ---- the latest values after Ctrl+C has unwound this function's workspace.
S = containers.Map('KeyType', 'char', 'ValueType', 'any');
S('Ts')                = NaN;
S('outVar')            = 'out';
S('freqLimit')         = Inf;
S('inRound')           = false;
S('cur')               = struct();
S('stepLog')           = emptyStepLog();
S('boardMayBeDriving') = false;   % true while a non-zero PWM may be latched on the board
S('shutdownDone')      = false;
S('roundTimes')        = [];

cleanupObj = onCleanup(@() safeShutdown(cfg, S, 'Ctrl+C / exit'));

try
    preflight(cfg, S);
    [runs, nSkipped] = buildRunList(cfg, S('freqLimit'));
    nRuns = numel(runs);

    fprintf('\n==================== Lab 2.1 AutoRun: %s ====================\n', cfg.ModelName);
    fprintf('Rounds to run: %d   (skipped: %d)   Output: %s\n', nRuns, nSkipped, cfg.OutDir);
    if nRuns == 0
        fprintf('Nothing to run. Delete files in the output folder or set cfg.Resume = false.\n');
        safeShutdown(cfg, S, 'done');
        return
    end
    fprintf('Estimated total runtime: %s\n\n', fmtDur(remainingAfter(cfg, S, 0, nRuns)));

    tExp = tic;
    for iRun = 1:nRuns
        roundData = runRound(cfg, S, runs(iRun), iRun, nRuns);
        saveRound(cfg, runs(iRun), roundData);
        if iRun < nRuns
            cooldown(cfg, S, iRun, nRuns);
        end
    end
    fprintf('\nAll %d rounds finished in %s. Data in %s\n', nRuns, fmtDur(toc(tExp)), cfg.OutDir);
    safeShutdown(cfg, S, 'done');
catch ME
    liveLine([]);
    fprintf(2, '\n[ERROR] %s\n', ME.message);
    safeShutdown(cfg, S, 'error');
    rethrow(ME);
end
end

% ================================ Experiment flow ================================

function preflight(cfg, S)
mdl = cfg.ModelName;
if ~bdIsLoaded(mdl)
    slxFile = fullfile(cfg.ModelDir, [mdl '.slx']);
    if isfile(slxFile)
        load_system(slxFile);
    else
        load_system(mdl);   % fall back to the MATLAB path
    end
end
blocks = {cfg.BlkPWM, cfg.BlkFreq, cfg.BlkFreqSat, cfg.BlkRx, cfg.BlkSetup};
for b = 1:numel(blocks)
    if getSimulinkBlockHandle(blocks{b}) < 0
        error('Lab21:block', 'Block not found: %s (check the CONFIGURATION section).', blocks{b});
    end
end
if ~strcmp(get_param(mdl, 'SimulationStatus'), 'stopped')
    error('Lab21:busy', 'Model %s is not stopped. Stop it, then run this again.', mdl);
end

% Logging + real-time pacing (in memory only; the .slx is not saved)
set_param(mdl, 'SignalLogging', 'on', 'SignalLoggingName', 'logsout', ...
    'SignalLoggingSaveFormat', 'Dataset', 'ReturnWorkspaceOutputs', 'on', ...
    'LimitDataPoints', 'off', 'EnablePacing', 'on', 'PacingRate', 1, 'StopTime', 'inf');
S('outVar') = get_param(mdl, 'ReturnWorkspaceOutputsName');
Ts = str2double(get_param(mdl, 'FixedStep'));
if ~(isfinite(Ts) && Ts > 0)
    error('Lab21:ts', 'Could not read a numeric fixed step from %s.', mdl);
end
S('Ts') = Ts;

ph = get_param(cfg.BlkRx, 'PortHandles');
for s = 1:numel(cfg.Signals)
    sig = cfg.Signals(s);
    hPort = ph.Outport(sig.port);
    hLine = get_param(hPort, 'Line');
    if hLine < 0
        error('Lab21:line', 'Rx port %d (%s) is not connected.', sig.port, sig.name);
    end
    set_param(hLine, 'Name', sig.name);
    set_param(hPort, 'DataLogging', 'on', 'DataLoggingNameMode', 'SignalName');
end

S('freqLimit') = evalNum(get_param(cfg.BlkFreqSat, 'UpperLimit'));
setPWM(cfg, S, 0);
if ~isfolder(cfg.OutDir)
    mkdir(cfg.OutDir);
end
fprintf('Preflight OK: Ts = %g s, Motor Frequency limit = %g Hz, logging %d signals.\n', ...
    Ts, S('freqLimit'), numel(cfg.Signals));
end

function [runList, nSkip] = buildRunList(cfg, freqLimit)
runList = struct('setIdx', {}, 'freqCmd', {}, 'freqApplied', {}, 'round', {}, 'file', {});
nSkip = 0;
for si = 1:numel(cfg.FreqSets)
    for f = cfg.FreqSets{si}
        fApplied = f;
        if f > freqLimit
            switch lower(cfg.OverLimitAction)
                case 'skip'
                    warning('Lab21:freqLimit', ['Set %d: %g Hz exceeds the %g Hz limit of %s ' ...
                        '-> SKIPPED (change cfg.OverLimitAction to run it).'], si, f, freqLimit, cfg.BlkFreqSat);
                    nSkip = nSkip + cfg.NumRounds;
                    continue
                case 'clip'
                    fApplied = freqLimit;
                    warning('Lab21:freqLimit', 'Set %d: %g Hz will run at the %g Hz limit.', si, f, freqLimit);
                otherwise
                    error('Lab21:freqLimit', 'Set %d: %g Hz exceeds the %g Hz limit of %s.', ...
                        si, f, freqLimit, cfg.BlkFreqSat);
            end
        end
        for rr = 1:cfg.NumRounds
            fname = sprintf('Set%d_F%gHz_R%d.mat', si, f, rr);
            if cfg.Resume && isfile(fullfile(cfg.OutDir, fname))
                nSkip = nSkip + 1;
                continue
            end
            runList(end+1) = struct('setIdx', si, 'freqCmd', f, 'freqApplied', fApplied, ... %#ok<AGROW>
                'round', rr, 'file', fname);
        end
    end
end
end

function R = runRound(cfg, S, r, iR, nR)
tRound = tic;
S('cur') = r;
S('stepLog') = emptyStepLog();
nSteps = numel(cfg.DutyPct);

fprintf('--- [%d/%d] Set %d | %g Hz | Round %d/%d ---\n', ...
    iR, nR, r.setIdx, r.freqCmd, r.round, cfg.NumRounds);
setPWM(cfg, S, 0);
set_param(cfg.BlkFreq, 'Value', sprintf('%.10g', r.freqApplied));
startSim(cfg, S);
S('inRound') = true;
waitUntilSimTime(cfg, simNow(cfg) + cfg.PreRoll_s, '', 0);

for k = 1:nSteps
    duty = cfg.DutyPct(k);
    pwm  = round(duty / 100 * cfg.PWMFullScale);
    setPWM(cfg, S, pwm);
    t0 = simNow(cfg);
    hdr = sprintf('Set %d | %6g Hz | Round %d/%d | Duty %3d%% (PWM %5d) | step %2d/%d', ...
        r.setIdx, r.freqCmd, r.round, cfg.NumRounds, duty, pwm, k, nSteps);
    etaAfterStep = (nSteps - k) * cfg.StepDuration_s + cfg.StopFlush_s ...
        + roundOverhead(cfg, S) + remainingAfter(cfg, S, iR, nR);
    waitUntilSimTime(cfg, t0 + cfg.StepDuration_s, hdr, etaAfterStep);

    sl = S('stepLog');
    sl(end+1) = struct('duty_pct', duty, 'pwm_cmd', pwm, ...
        't0', t0, 't1', t0 + cfg.StepDuration_s); %#ok<AGROW>
    S('stepLog') = sl;
    progressRound = r;    % per-step checkpoint of completed steps
    progressSteps = sl;
    save(fullfile(cfg.OutDir, '_inprogress.mat'), 'progressRound', 'progressSteps');
end

setPWM(cfg, S, 0);
waitUntilSimTime(cfg, simNow(cfg) + cfg.StopFlush_s, '', 0);
S('boardMayBeDriving') = false;          % PWM = 0 has been streamed
stopSim(cfg, S);
R = extractRound(cfg, S, r, S('stepLog'), false);
S('inRound') = false;
S('roundTimes') = [S('roundTimes'), toc(tRound)];
end

function R = extractRound(cfg, S, r, steps, isPartial)
Ts = S('Ts');
simOut = evalin('base', S('outVar'));
ds = simOut.logsout;
nSig = numel(cfg.Signals);
raw = struct('note', 'Full-round logged samples (sim time). Use to re-slice windows.');
sigT = cell(1, nSig);
sigY = cell(1, nSig);
for s = 1:nSig
    el = ds.getElement(cfg.Signals(s).name);
    if isempty(el)
        error('Lab21:noSignal', 'Signal "%s" not found in logsout.', cfg.Signals(s).name);
    end
    if isa(el, 'Simulink.SimulationData.Dataset')
        el = el{1};
    end
    sigT{s} = el.Values.Time(:);
    sigY{s} = double(squeeze(el.Values.Data));
    sigY{s} = sigY{s}(:);
    raw.(['t_' cfg.Signals(s).field]) = sigT{s};
    raw.(cfg.Signals(s).field) = sigY{s};
end

nSamp = round(cfg.StepDuration_s / Ts);
tRel = (0:nSamp-1)' * Ts;
stepTpl = struct('set', r.setIdx, 'freq_Hz', r.freqCmd, 'freq_applied_Hz', r.freqApplied, ...
    'round', r.round, 'duty_pct', NaN, 'pwm_cmd', NaN, 't', tRel, 't_abs', [], ...
    'omega', [], 'loadcell', [], 'current', [], 'pwm_echo', [], 'echo_ok', false);
st = repmat(stepTpl, numel(steps), 1);
nBadEcho = 0;
for k = 1:numel(steps)
    st(k).duty_pct = steps(k).duty_pct;
    st(k).pwm_cmd  = steps(k).pwm_cmd;
    st(k).t_abs    = steps(k).t0 + tRel;
    for s = 1:nSig
        st(k).(cfg.Signals(s).field) = zohSample(sigT{s}, sigY{s}, st(k).t_abs, Ts);
    end
    tail = st(k).pwm_echo(tRel >= cfg.StepDuration_s - 2);   % last 2 s of the step
    echoVal = median(tail, 'omitnan');
    st(k).echo_ok = abs(echoVal - steps(k).pwm_cmd) <= cfg.EchoTolerance;
    if ~st(k).echo_ok
        nBadEcho = nBadEcho + 1;
        fprintf(2, '   [WARN] Duty %d%%: PWM echo %.0f vs command %d - check the serial link.\n', ...
            steps(k).duty_pct, echoVal, steps(k).pwm_cmd);
    end
end

R.meta = struct('model', cfg.ModelName, 'set', r.setIdx, 'freq_Hz', r.freqCmd, ...
    'freq_applied_Hz', r.freqApplied, 'round', r.round, 'Ts', Ts, ...
    'step_duration_s', cfg.StepDuration_s, 'n_steps', numel(steps), ...
    'partial', isPartial, 'echo_mismatch_steps', nBadEcho, ...
    'units', {{cfg.Signals.field; cfg.Signals.unit}}, 'saved', char(datetime('now')));
R.steps = st;
R.raw = raw;
R.cfg = cfg;
end

function saveRound(cfg, r, R)
fFinal = fullfile(cfg.OutDir, r.file);
fTmp = [fFinal(1:end-4) '_tmp.mat'];
save(fTmp, '-struct', 'R', '-v7.3');
movefile(fTmp, fFinal, 'f');          % atomic: a crash never leaves a half-written round file
deleteIfExists(fullfile(cfg.OutDir, '_inprogress.mat'));
deleteIfExists(strrep(fFinal, '.mat', '_PARTIAL.mat'));

idxFile = fullfile(cfg.OutDir, 'Lab21_index.mat');
row = table(r.setIdx, r.freqCmd, r.freqApplied, r.round, string(r.file), ...
    datetime('now'), R.meta.echo_mismatch_steps, 'VariableNames', ...
    {'Set', 'Freq_Hz', 'FreqApplied_Hz', 'Round', 'File', 'Finished', 'EchoMismatchSteps'});
if isfile(idxFile)
    old = load(idxFile, 'index');
    index = [old.index; row];
else
    index = row;
end
save(idxFile, 'index');
fprintf('   Saved %s\n', fFinal);
end

function cooldown(cfg, S, iR, nR)
setPWM(cfg, S, 0);
tc = tic;
etaAfter = remainingAfter(cfg, S, iR, nR) - cfg.Cooldown_s;
while true
    left = cfg.Cooldown_s - toc(tc);
    if left <= 0
        break
    end
    liveLine(sprintf('   Cooldown (PWM = 0): %s left | ETA remaining %s', ...
        fmtDur(left), fmtDur(left + etaAfter)));
    pause(min(1, left));
end
liveLine([]);
end

% ============================== Simulink helpers ================================

function waitUntilSimTime(cfg, target, hdr, etaAfter)
mdl = cfg.ModelName;
tw = tic;
tPrint = -Inf;
maxWall = 3 * max(target - simNow(cfg), 0) + 15;   % detects a stalled model
while true
    st = get_param(mdl, 'SimulationStatus');
    if ~strcmp(st, 'running')
        error('Lab21:simStopped', ['Simulation is "%s" (expected "running"). ' ...
            'Check the serial link and the Diagnostic Viewer.'], st);
    end
    tNow = simNow(cfg);
    if tNow >= target
        break
    end
    if toc(tw) > maxWall
        error('Lab21:stall', 'Simulation time stalled at %.3f s.', tNow);
    end
    if ~isempty(hdr) && toc(tw) - tPrint >= 0.25
        tPrint = toc(tw);
        liveLine(sprintf('%s | %4.1f s left | ETA %s', hdr, target - tNow, ...
            fmtDur(etaAfter + target - tNow)));
    end
    pause(0.05);
end
if ~isempty(hdr)
    liveLine(sprintf('%s | done', hdr));
    liveLine([]);
end
end

function startSim(cfg, S)
mdl = cfg.ModelName;
evalin('base', sprintf('clear(''%s'')', S('outVar')));
set_param(mdl, 'SimulationCommand', 'start');
ts = tic;
while ~strcmp(get_param(mdl, 'SimulationStatus'), 'running')
    if toc(ts) > cfg.StartTimeout_s
        error('Lab21:start', 'Model did not reach "running" within %d s.', cfg.StartTimeout_s);
    end
    if toc(ts) > 3 && strcmp(get_param(mdl, 'SimulationStatus'), 'stopped')
        error('Lab21:start', ['Model stopped right after starting (serial port busy or ' ...
            'board unplugged?). See the Diagnostic Viewer.']);
    end
    pause(0.05);
end
end

function stopSim(cfg, S)
mdl = cfg.ModelName;
if any(strcmp(get_param(mdl, 'SimulationStatus'), {'running', 'paused'}))
    set_param(mdl, 'SimulationCommand', 'stop');
end
ts = tic;
while ~strcmp(get_param(mdl, 'SimulationStatus'), 'stopped') && toc(ts) < 30
    pause(0.05);
end
while ~evalin('base', sprintf('exist(''%s'', ''var'')', S('outVar'))) && toc(ts) < 40
    pause(0.05);
end
end

function setPWM(cfg, S, value)
set_param(cfg.BlkPWM, 'Value', sprintf('%d', round(value)));
if value ~= 0 && strcmp(get_param(cfg.ModelName, 'SimulationStatus'), 'running')
    S('boardMayBeDriving') = true;
end
end

function t = simNow(cfg)
t = get_param(cfg.ModelName, 'SimulationTime');
end

% ================================ Safety handler ================================

function safeShutdown(cfg, S, reason)
if S('shutdownDone')
    return
end
S('shutdownDone') = true;
liveLine([]);
mdl = cfg.ModelName;
if ~strcmp(reason, 'done')
    fprintf(2, '\n[SAFETY] %s: forcing PWM Range = 0 and stopping %s ...\n', reason, mdl);
end
try
    if bdIsLoaded(mdl)
        set_param(cfg.BlkPWM, 'Value', '0');
        if strcmp(get_param(mdl, 'SimulationStatus'), 'running')
            tz = simNow(cfg);
            tw = tic;
            while strcmp(get_param(mdl, 'SimulationStatus'), 'running') ...
                    && simNow(cfg) < tz + cfg.StopFlush_s && toc(tw) < 3
                pause(0.05);
            end
            if simNow(cfg) >= tz + cfg.StopFlush_s
                S('boardMayBeDriving') = false;   % zero was streamed to the board
            end
        end
        if ~strcmp(get_param(mdl, 'SimulationStatus'), 'stopped')
            stopSim(cfg, S);
        end
    end
catch err
    fprintf(2, '[SAFETY] Could not stop the model cleanly: %s\n', err.message);
end

if S('inRound') && ~isempty(S('stepLog'))
    cur = S('cur');
    try
        R = extractRound(cfg, S, cur, S('stepLog'), true);
        fPart = fullfile(cfg.OutDir, strrep(cur.file, '.mat', '_PARTIAL.mat'));
        save(fPart, '-struct', 'R', '-v7.3');
        fprintf(2, '[SAFETY] Saved %d completed step(s) of the interrupted round to %s\n', ...
            numel(S('stepLog')), fPart);
    catch err
        fprintf(2, '[SAFETY] Partial round not saved (%s). Step log: %s\n', ...
            err.message, fullfile(cfg.OutDir, '_inprogress.mat'));
    end
end
S('inRound') = false;

if S('boardMayBeDriving')
    sendZeroPacketDirect(cfg, S);
end
if ~strcmp(reason, 'done')
    fprintf(2, '[SAFETY] Done. Make sure the motor has stopped.\n');
end
end

function sendZeroPacketDirect(cfg, S)
% Fallback when the model cannot stream PWM = 0: write one Host Serial Tx
% packet by hand. Layout taken from MotorXploer/Host Serial Tx (binary mode):
%   0x25 | single PWM, MotorFreq, StepperFreq | int16 M0 M1 M2 DIR EN | 0x4E
mdl = cfg.ModelName;
fprintf(2, '[SAFETY] Model did not confirm PWM = 0, writing it straight to the serial port ...\n');
try
    port   = strtrim(strrep(get_param(cfg.BlkSetup, 'port'), '''', ''));
    fMotor = min(evalNum(get_param(cfg.BlkFreq, 'Value')), S('freqLimit'));
    fStep  = evalNum(get_param([mdl '/Stepper Frequency'], 'Value'));
    ints   = cellfun(@(n) evalNum(get_param([mdl '/' n], 'Value')), {'M0', 'M1', 'M2', 'DIR', 'EN'});
    pkt = [uint8(hex2dec('25')), typecast(single([0 fMotor fStep]), 'uint8'), ...
           typecast(int16(ints), 'uint8'), uint8(hex2dec('4E'))];
    sp = serialport(port, 2000000);
    for n = 1:50
        write(sp, pkt, 'uint8');
        pause(0.002);
    end
    delete(sp);
    S('boardMayBeDriving') = false;
    fprintf(2, '[SAFETY] PWM = 0 packet sent on %s.\n', port);
catch err
    fprintf(2, ['[SAFETY] !!! Could not send PWM = 0 (%s).\n' ...
        '[SAFETY] !!! CUT MOTOR POWER or press RESET on the Nucleo now.\n'], err.message);
end
end

% =============================== Small helpers ==================================

function s = nominalRound(cfg)
s = cfg.PreRoll_s + numel(cfg.DutyPct) * cfg.StepDuration_s + cfg.StopFlush_s;
end

function s = roundOverhead(cfg, S)
rt = S('roundTimes');
if isempty(rt)
    s = 5;   % compile + serial open guess until measured
else
    s = max(mean(rt) - nominalRound(cfg), 0);
end
end

function s = remainingAfter(cfg, S, iR, nR)
% Seconds left after round iR ends, including the cooldowns still to come.
nLeft = nR - iR;
s = nLeft * (nominalRound(cfg) + roundOverhead(cfg, S)) + nLeft * cfg.Cooldown_s;
if iR == 0
    s = s - cfg.Cooldown_s;   % no cooldown after the very last round
end
end

function liveLine(txt)
% liveLine(txt) rewrites the current Command Window line; liveLine([]) ends it.
persistent len
if isempty(len)
    len = 0;
end
if isempty(txt)
    if len > 0
        fprintf('\n');
    end
    len = 0;
    return
end
fprintf([repmat('\b', 1, len) '%s'], txt);
len = numel(txt);
end

function s = emptyStepLog()
s = struct('duty_pct', {}, 'pwm_cmd', {}, 't0', {}, 't1', {});
end

function y = zohSample(t, v, tq, Ts)
% Zero-order-hold sample of logged (t, v) at query times tq. Works for signals
% logged every step and for signals logged only when they change.
[tu, ia] = unique(t, 'last');
vu = v(ia);
tq = tq + Ts * 1e-6;          % guard against float round-off at sample instants
y = NaN(size(tq));
if isempty(tu)
    return
elseif isscalar(tu)
    y(tq >= tu) = vu;
    return
end
idx = interp1(tu, (1:numel(tu))', tq, 'previous');
idx(tq >= tu(end)) = numel(tu);
ok = ~isnan(idx);
y(ok) = vu(idx(ok));
end

function v = evalNum(str)
v = str2double(str);
if isnan(v)
    v = evalin('base', str);   % parameter given as a workspace variable
end
end

function deleteIfExists(f)
if isfile(f)
    delete(f);
end
end

function s = fmtDur(sec)
sec = max(0, round(sec));
s = sprintf('%dh %02dm %02ds', floor(sec / 3600), floor(mod(sec, 3600) / 60), mod(sec, 60));
end
