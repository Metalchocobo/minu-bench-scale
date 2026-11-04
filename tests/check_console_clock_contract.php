<?php

// Host-only contract check. The bridge links the production firmware Session.
// No database, MQTT broker, firmware build, flash or physical device is used.
$options = getopt('', ['manager-root:', 'bridge:']);
$managerRoot = realpath($options['manager-root'] ?? '');
$bridge = realpath($options['bridge'] ?? '');
if (! $managerRoot || ! is_file($managerRoot.'/vendor/autoload.php') || ! $bridge || ! is_file($bridge)) {
    fwrite(STDERR, "Usage: php tests/check_console_clock_contract.php --manager-root=<Manager> --bridge=<host executable>\n");
    exit(2);
}
require $managerRoot.'/vendor/autoload.php';

use App\Services\ScaleConsole\ScaleConsoleDeviceClock;
use App\Services\ScaleConsole\ScaleConsoleRequestEncoder;
use App\Services\ScaleConsole\ScaleConsoleResponseDecoder;

$process = proc_open([$bridge], [0 => ['pipe', 'r'], 1 => ['pipe', 'w'], 2 => ['pipe', 'w']], $pipes, __DIR__, null, ['bypass_shell' => true]);
if (! is_resource($process)) {
    throw new RuntimeException('Unable to start the host console bridge.');
}
stream_set_timeout($pipes[1], 5);
$assertions = 0;
$assert = static function (bool $condition, string $message) use (&$assertions): void {
    ++$assertions;
    if (! $condition) {
        throw new RuntimeException($message);
    }
};
$exchange = static function (array $request, int $epoch, int $nowMs, ?string $reset = null) use ($pipes): array {
    $envelope = ['wire' => ScaleConsoleRequestEncoder::encode($request), 'now_epoch' => $epoch, 'now_ms' => $nowMs];
    if ($reset !== null) {
        $envelope['reset'] = $reset;
    }
    fwrite($pipes[0], json_encode($envelope, JSON_THROW_ON_ERROR | JSON_UNESCAPED_UNICODE)."\n");
    fflush($pipes[0]);
    $response = fgets($pipes[1]);
    if ($response === false) {
        throw new RuntimeException('The host console bridge did not respond.');
    }

    return json_decode($response, true, 512, JSON_THROW_ON_ERROR);
};
$request = static fn (string $action, string $sessionId, string $requestId): array => [
    'v' => 1, 'action' => $action, 'request_id' => $requestId, 'session_id' => $sessionId,
    'boot_id' => '', 'expires_at' => 1793829656 + 10, 'cursor' => 0,
];
$status = static fn (int $epoch, string $boot = 'a4b94eaa'): string => json_encode([
    'type' => 'status', 'scale_id' => 'aabbccddeeff', 'state' => 'online',
    'boot_id' => $boot, 'published_at' => $epoch,
], JSON_THROW_ON_ERROR);
try {
    // Exact production incident: device twelve seconds ahead rejects server+10.
    $oldProbe = $request('probe', '', 'old-probe');
    $oldResult = $exchange($oldProbe, 1793829668, 100, 'a4b94eaa');
    $assert($oldResult['error'] === 'expired', 'The original clock skew was not reproduced.');
    $assert($oldResult['active'] === false, 'A rejected probe unexpectedly opened a session.');

    foreach ([-120, 0, 12, 120] as $clockSkew) {
        $serverEpoch = 1793829656;
        $deviceEpoch = $serverEpoch + $clockSkew;
        // Retained heartbeat is five seconds old; connection/SUBACK takes three.
        $clock = new ScaleConsoleDeviceClock('aabbccddeeff', '');
        $clock->observe($status($deviceEpoch - 5), 1000000000);
        $probe = $clock->finalize($request('probe', '', 'probe-'.$clockSkew), 4000000000);
        $result = $exchange($probe, $deviceEpoch + 3, 100, 'a4b94eaa');
        $assert($result['state'] === 'ok', 'A bounded device-clock probe failed.');
        $assert(ScaleConsoleResponseDecoder::decode('aabbccddeeff', $probe, json_encode($result), false) !== null, 'PHP rejected the real firmware probe response.');

        $sessionId = 'session-'.($clockSkew + 200);
        $clock = new ScaleConsoleDeviceClock('aabbccddeeff', 'a4b94eaa');
        $clock->observe($status($deviceEpoch + 3), 4000000000);
        $open = $clock->finalize($request('open', $sessionId, 'open-'.$clockSkew), 5000000000);
        $result = $exchange($open, $deviceEpoch + 4, 200);
        $assert($result['state'] === 'ok' && $result['active'], 'The real firmware session did not open.');

        $poll = $clock->finalize($request('poll', $sessionId, 'poll-'.$clockSkew), 6000000000);
        $assert($exchange($poll, $deviceEpoch + 5, 300)['state'] === 'ok', 'A live poll failed.');
        $exec = $clock->finalize($request('exec', $sessionId, 'exec-'.$clockSkew) + ['seq' => 1, 'command' => 'scale status'], 7000000000);
        $result = $exchange($exec, $deviceEpoch + 6, 400);
        $assert($result['state'] === 'done' && $result['executions'] === 1, 'A command did not execute once.');
        $replay = $exchange($exec, $deviceEpoch + 6, 401);
        $assert($replay['state'] === 'done' && $replay['executions'] === 1, 'A command replay executed twice.');
        $close = $clock->finalize($request('close', $sessionId, 'close-'.$clockSkew), 8000000000);
        $assert(! $exchange($close, $deviceEpoch + 7, 500)['active'], 'Close did not release the session.');
        $assert($exchange($open, $deviceEpoch + 7, 501)['error'] === 'session_closed', 'Delayed open bypassed the closed-session replay fence.');
    }

    $clock = new ScaleConsoleDeviceClock('aabbccddeeff', '');
    $clock->observe($status(1793829656 - 40), 1000000000);
    $stale = $clock->finalize($request('probe', '', 'stale-probe'), 2000000000);
    $assert($exchange($stale, 1793829657, 100, 'a4b94eaa')['error'] === 'expired', 'A stale retained anchor became actionable.');
    echo 'PASS console clock wire contract: '.$assertions." assertions\n";
} finally {
    foreach ($pipes as $pipe) {
        fclose($pipe);
    }
    proc_close($process);
}
